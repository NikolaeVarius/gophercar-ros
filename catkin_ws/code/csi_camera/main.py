#!/usr/bin/env python

# Based off 
## https://github.com/JetsonHacksNano/CSI-Camera/blob/master/simple_camera.py
## http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
## http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
import cv2
import sys, time
import roslib
import rospy
import os

from cv_bridge import CvBridge
import numpy as np
from scipy.ndimage import filters

from sensor_msgs.msg import CompressedImage
VERBOSE="true"
# Toggle if a window showing camera output should pop up
SHOW__CAMERA="true"
# Toggle convering to greyscale
GRAYSCALE="false"

def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=30,
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

def main(args):
    print("ROS_MASTER_URI: " + os.environ['ROS_MASTER_URI'])
    rospy.init_node('image_capture', anonymous="false")
    image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, 30)
    # Annoyingly bridge does not support compressed image
    bridge = CvBridge()
    print(gstreamer_pipeline(flip_method=2))
    # rate = rospy.Rate(0.5)

    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
    frames = 0

    if cap.isOpened():
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)

        while cv2.getWindowProperty("CSI Camera", 0) >= 0:
            ret_val, img = cap.read()
            while ret_val:
                # this is slow
                rval, frame = cap.read()
                # cv2.imshow("CSI Camera", img)
                
                if frame is not None:
                    frame = np.uint8(frame)
                # image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format = "jpeg"
                msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
                frames = frames + 1
                print(frames)
                image_pub.publish(msg)

                keyCode = cv2.waitKey(1000) & 0xFF
                if keyCode == 27:
                    break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print( "Shutting down ROS Camera Publisher")


if __name__ == "__main__":
  main(sys.argv)
