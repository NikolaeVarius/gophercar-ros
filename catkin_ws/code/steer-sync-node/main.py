# Writes image and steering data synchronized via timestamp
# mostly since I'm too lazy to want to rewrite this in go at this very second

import message_filters
import rospy
import sys
from sensor_msgs.msg import CompressedImage, CameraInfo, Joy
from cv_bridge import CvBridge, CvBridgeError
import cv2 

# from cv_bridge.boost.cv_bridge_boost import getCvType

bridge = CvBridge()

# Change this to somewhere not here at some point
image_path = "./data/images/"
joy_path = "./data/joy/"

def callback(image, joy):
    img_timestamp = image.header.stamp.secs
    joy_timestamp = joy.header.stamp.secs
    
    try:
        cv2_img = bridge.compressed_imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        image_filename = image_path + str(img_timestamp) + "-image.jpg"
        cv2.imwrite(image_filename, cv2_img)

def main(args):
    rospy.init_node('steer_data_sync', anonymous=True)
    image_sub = message_filters.Subscriber('output/image_raw/compressed', CompressedImage)
    joy_sub = message_filters.Subscriber('joy', Joy)

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, joy_sub], 10, 1)
    ts.registerCallback(callback)
    print("Spinning")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
