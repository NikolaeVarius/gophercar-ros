# Toy with feeding opencv data from ROS compressed image topic

import message_filters
import rospy
import sys
from sensor_msgs.msg import CompressedImage, CameraInfo, Joy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


# cv2.namedWindow('Window', cv2.WINDOW_AUTOSIZE)

def callback(image):
    bridge = CvBridge()
    img_timestamp = image.header.stamp.secs
    print(img_timestamp)
    try:
        cv2_img = bridge.compressed_imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Window", cv2_img)
    cv2.waitKey(1)



def main(args):
    rospy.init_node('follow_node', anonymous=True)
    rospy.Subscriber('output/image_raw/compressed', CompressedImage, callback)
    try:
        rospy.spin()
        cv2.destroyAllWindows()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ == '__main__':
    main(sys.argv)
