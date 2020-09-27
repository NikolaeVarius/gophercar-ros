#!/usr/bin/env python

# Based off 
## https://github.com/JetsonHacksNano/CSI-Camera/blob/master/simple_camera.py
## http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
## http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
## https://github.com/nrsyed/computer-vision
import cv2
import sys, time
import roslib
import rospy
import os
import argparse
from threading import Thread
# from queue import Queue

from cv_bridge import CvBridge
import numpy as np
from scipy.ndimage import filters

# For compressed image
from sensor_msgs.msg import CompressedImage, Image
# For non compressed image
from cv_bridge import CvBridge, CvBridgeError

# Logging
VERBOSE=True
LOG_AFTER_FRAMES=100 # Number of frames per log emitted reporting number of frames processed. Setting this to 0 should disable it

# Toggle if a window showing camera output should pop up
ENABLE_DISPLAY=False

# Image Frame Settings
GRAYSCALE=False # Convert to greyscale
SCALING_PERCENT=50 # How much to scale image size
ENABLE_FPS=False

# TODO?
# Cut Percentage off each side of an image. This is useful for cutting out uneeded image data a.k.a anything not road.
# Generally its probably safe to remove a bit of the top of the image
# CUT_LEFT_IMAGE=0 #
# CUT_RIGHT_IMAGE=0
# CUT_TOP_IMAGE=0
# CUT_LEFT_IMAGE=0
# SHOW_CUT_LINES=False


def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=59,
    flip_method=2,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=%d, height=%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=%d, height=%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

class VideoStream:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):    
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()

    def stop(self):
        self.stopped = True

class VideoShow:
    def __init__(self, frame=None):
        self.frame = frame
        self.stopped = False

    def start(self):
        Thread(target=self.show, args=()).start()
        return self

    def show(self):
        while not self.stopped:
            cv2.imshow("CSI Camera", self.frame);
            keyCode = cv2.waitKey(30) & 0xFF
            # Kills on Escape
            if keyCode == 27:
                self.stopped = True
                break

    def stop(self):
        self.stopped = True

def getCurrentFPS(currentTime, previousTime):
        return 1/(currentTime - previousTime)


def resizeFrame(frame, scalePercent=100):
    newWidth = int(frame.shape[1] * scalePercent / 100)
    newHeight = int(frame.shape[0] * scalePercent / 100)
    return cv2.resize(frame, (newWidth, newHeight))


def cv2_to_imgmsg(self, cvim, encoding = "passthrough"):

    if not isinstance(cvim, (np.ndarray, np.generic)):
        raise TypeError('Your input type is not a numpy array')
    img_msg = sensor_msgs.msg.Image()
    img_msg.height = cvim.shape[0]
    img_msg.width = cvim.shape[1]
    print("here2")
    if len(cvim.shape) < 3:
        cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, 1)
    else:
        cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, cvim.shape[2])
    if encoding == "passthrough":
        img_msg.encoding = cv_type
    else:
        img_msg.encoding = encoding
        # Verify that the supplied encoding is compatible with the type of the OpenCV image
        if self.cvtype_to_name[self.encoding_to_cvtype2(encoding)] != cv_type:
            raise CvBridgeError("encoding specified as %s, but image has incompatible type %s" % (encoding, cv_type))
    print("here3")
    if cvim.dtype.byteorder == '>':
        img_msg.is_bigendian = True
    img_msg.data = cvim.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height

    return img_msg

def main():
    # ap = argparse.ArgumentParser()
    # ap.add_argument("--enable-display", "-e", default="true", help="Activate OpenCV Display Window")
    # args = vars(ap.parse_args())

    # if args["enable-display"] == "false":
    #     ENABLE_DISPLAY = "false"
    
    # Compressed Image
    #image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=30)
    
    # Non Compressed Image
    image_pub = rospy.Publisher("/output/image_raw/image", Image, queue_size=30)
    
    print(gstreamer_pipeline(flip_method=2))
    # Keep track of how many frames hav been generated
    frames = 0
    # Keep track of FPS emitted by camera
    previousTime = 0

    stream = VideoStream().start()

    bridge = CvBridge()
    show = VideoShow(stream.frame)

    if ENABLE_DISPLAY is True:
        show.start()

    while True:
        if stream.stopped or show.stopped:
            stream.stop()
            show.stop()
            break

        frame = stream.frame
        
        if frame is not None:
            frame = np.uint8(frame)
        else:
            print("Recieved null frame. Skipping")
            continue

        # Cut down resolution of frame
        if SCALING_PERCENT != 100:
            frame = resizeFrame(frame, SCALING_PERCENT)

        # Convert to Greyscale
        if GRAYSCALE is True:
            frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        if ENABLE_DISPLAY is True and ENABLE_FPS is True:
            currentTime = time.time()
            frame = overlayFPS(frame, currentTime, previousTime, frames)
            previousTime = currentTime

        # Update Frame
        show.frame = frame

        # ROS Image
        # Compressed Image
        #msg = CompressedImage()
        #msg.header.stamp = rospy.Time.now()
        #msg.format = "jpeg"
        #msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        #image_pub.publish(msg)

        # Non Compressed Image
        try:
            # msg = Image()
            # msg.header.stamp = rospy.Time.now()
            # msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
            # msg.encoding = "bgr8"
            # msg.height = 1
            # msg.width = len(msg.data)
            # msg.step = len(msg.data) * 3
            # image = cv2_to_imgmsg(frame, encoding="bgr8")
            # image_pub.publish(image)
            cvim = frame
            ############## HACKING THIS SHIT IN. LIB DOESNT WORK DONT KNOW WHY. COMMENTED OUT AREAS ARE BROKEN
            #### THIS IS SHAMEFUL AND RIPPED OUT THE THE LIB

            if not isinstance(cvim, (np.ndarray, np.generic)):
                raise TypeError('Your input type is not a numpy array')
            # img_msg = sensor_msgs.msg.Image()
            img_msg = Image()
            img_msg.height = cvim.shape[0]
            img_msg.width = cvim.shape[1]
            # if len(cvim.shape) < 3:
            #     print("here3")
            #     cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, 1)
            # else:
            #     print("here4")
            #     cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, cvim.shape[2])
            img_msg.encoding = "bgr8"
            # if encoding == "passthrough":
            #     img_msg.encoding = cv_type
            # else:
            #     img_msg.encoding = "bgr8"
            #     # Verify that the supplied encoding is compatible with the type of the OpenCV image
            #     if self.cvtype_to_name[self.encoding_to_cvtype2(encoding)] != cv_type:
            #         raise CvBridgeError("encoding specified as %s, but image has incompatible type %s" % (encoding, cv_type))
            if cvim.dtype.byteorder == '>':
                img_msg.is_bigendian = True
            img_msg.data = cvim.tostring()
            img_msg.step = len(img_msg.data) // img_msg.height
    #######################
            image_pub.publish(img_msg)
        except CvBridgeError as e:
            print(e)
            sys.exit(1)
        except:
            sys.exit(1)
        
        
        frames = frames + 1
        if LOG_AFTER_FRAMES != 0 and frames % LOG_AFTER_FRAMES == 0:
            print("Processed " + str(LOG_AFTER_FRAMES) + " frames for a total of " + str(frames))

        # https://stackoverflow.com/questions/35372700/whats-0xff-for-in-cv2-waitkey1/39203128#39203128
        keyCode = cv2.waitKey(30) & 0xFF
        # Kills on Escape
        if rospy.is_shutdown() or keyCode == 27:
            print('shutdown')
            stream.stop()
            show.stop()
            break
    cv2.destroyAllWindows()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print( "Shutting down ROS Camera Publisher")

def overlayFPS(frame, currentTime, previousTime, currentFrames):
    fps = getCurrentFPS(currentTime, previousTime)
    fps_display_string = "FPS : %0.1f" % fps
    cv2.putText(frame, fps_display_string, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0))
    cv2.putText(frame, "Frame: " + str(currentFrames), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0))
    return frame

if __name__ == "__main__":
    print("ROS_MASTER_URI: " + os.environ['ROS_MASTER_URI'])
    rospy.init_node('camera0', anonymous="false")
    main()
