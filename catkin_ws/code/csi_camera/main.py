#!/usr/bin/env python

# Based off 
## https://github.com/JetsonHacksNano/CSI-Camera/blob/master/simple_camera.py
## http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
import cv2
import sys, time
import roslib
import rospy

import numpy as np
from scipy.ndimage import filters

from sensor_msgs.msg import CompressedImage

class image_feature:
    def __init__(self):
        self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage)
        # self.bridge = CvBridge()
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print("subscribed to /camera/image/compressed")

    def callback(self, ros_data):
        if VERBOSE :
            print('received image of type: "%s"') % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        
        #### Feature detectors using CV2 #### 
        # "","Grid","Pyramid" + 
        # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF"
        method = "GridFAST"
        feat_det = cv2.FeatureDetector_create(method)
        time1 = time.time()

        # convert np image to grayscale
        featPoints = feat_det.detect(
            cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
        time2 = time.time()
        #if VERBOSE :
          #  print '%s detector found: %s points in: %s sec.'%(method,
           #     len(featPoints),time2-time1)

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
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=false)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print( "Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


def show_camera():
    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if cap.isOpened():
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
        # Window
        while cv2.getWindowProperty("CSI Camera", 0) >= 0:
            ret_val, img = cap.read()
            cv2.imshow("CSI Camera", img)
            # This also acts as
            keyCode = cv2.waitKey(30) & 0xFF
            # Stop the program on the ESC key
            if keyCode == 27:
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")



if __name__ == "__main__":
    show_camera()
