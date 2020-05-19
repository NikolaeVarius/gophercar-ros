#!/usr/bin/env python

# Based off 
## https://github.com/JetsonHacksNano/CSI-Camera/blob/master/simple_camera.py
## http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
## http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
import cv2
import sys, time
import roslib
import rospy
from cv_bridge import CvBridge
import numpy as np
from scipy.ndimage import filters

from sensor_msgs.msg import CompressedImage
VERBOSE="true"
# Toggle if a window showing camera output should pop up
SHOW__CAMERA="true"
# Toggle convering to greyscale
GRAYSCALE="false"


class image_publisher:
    def __init__(self):
        self.bridge = CvBridge()

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        

        time2 = time.time()

        for featpoint in featPoints:
            x,y = featpoint.pt
            cv2.circle(image_np,(int(x),int(y)), 3, (0,0,255), -1)
        
        cv2.imshow('cv_img', image_np)
        cv2.waitKey(2)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        self.image_pub.publish(msg)
        #self.subscriber.unregister()

def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=2,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
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
    rospy.init_node('image_capture', anonymous="false")
    image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage)
    print(gstreamer_pipeline(flip_method=2))

    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)

    if cap.isOpened():
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
        while cv2.getWindowProperty("CSI Camera", 0) >= 0:
            ret_val, img = cap.read()
            cv2.imshow("CSI Camera", img)
            keyCode = cv2.waitKey(30) & 0xFF
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
