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

from sensor_msgs.msg import CompressedImage
VERBOSE="true"
# Toggle if a window showing camera output should pop up
SHOW_OUTPUT_WINDOW="true"
ENABLE_DISPLAY="true"
# Toggle convering to greyscale
GRAYSCALE="false"


def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=60,
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
    

def main():
    # ap = argparse.ArgumentParser()
    # ap.add_argument("--enable-display", "-e", default="true", help="Activate OpenCV Display Window")
    # args = vars(ap.parse_args())

    # if args["enable-display"] == "false":
    #     ENABLE_DISPLAY = "false"

    print("ROS_MASTER_URI: " + os.environ['ROS_MASTER_URI'])
    rospy.init_node('image_capture', anonymous="false")
    image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=30)
    print(gstreamer_pipeline(flip_method=2))
    # Keep track of how many frames hav been generated
    frames = 0
    # Keep track of FPS emitted by camera
    previousTime = 0

    stream = VideoStream().start()

    # if ENABLE_DISPLAY is true:
    show = VideoShow(stream.frame).start()

    while True:
        if stream.stopped or show.stopped:
            stream.stop()
            show.stop()
            break

        frame = stream.frame
        if frame is not None:
            frame = np.uint8(frame)

        # Cut down resolution of frame
        resizedFrame = resizeFrame(frame, 50)

        # if DISPLAY_OUTPUT_METADATA:
        # To Display FPS
        currentTime = time.time()
        fps = getCurrentFPS(currentTime, previousTime)
        previousTime = currentTime
        fps_display_string = "FPS : %0.1f" % fps
        cv2.putText(resizedFrame, fps_display_string, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0))
        cv2.putText(resizedFrame, "Frame: " + str(frames), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0))
        
        # Update Frame
        show.frame = resizedFrame

        # ROS Image
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        frames = frames + 1
        image_pub.publish(msg)

        # https://stackoverflow.com/questions/35372700/whats-0xff-for-in-cv2-waitkey1/39203128#39203128
        # keyCode = cv2.waitKey(30) & 0xFF
        # # Kills on Escape
        # if keyCode == 27:
        #     break
    cv2.destroyAllWindows()


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print( "Shutting down ROS Camera Publisher")


if __name__ == "__main__":
    main()
