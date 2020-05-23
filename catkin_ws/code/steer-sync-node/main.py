# Writes image and steering data synchronized via timestamp
# mostly since I'm too lazy to want to rewrite this in go at this very second

import message_filters
import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo, Joy


rospy.init_node('steer_data_sync', anonymous=True)

def callback(image, joy):
    print(image)
    print(joy)
    # Save here
    return

image_sub = message_filters.Subscriber('/output/image_raw/compressed', Image)
joy_sub = message_filters.Subscriber('joy', Joy)

ts = message_filters.TimeSynchronizer([image_sub, joy_sub], 10)
ts.registerCallback(callback)
rospy.spin()
