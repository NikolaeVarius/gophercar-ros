# Writes image and steering data synchronized via timestamp
# mostly since I'm too lazy to want to rewrite this in go at this very second

import message_filters
import rospy
import sys
from sensor_msgs.msg import CompressedImage, CameraInfo, Joy




def callback(image, joy):
    print(image)
    print(joy)
    print("here")
    # Save here
    return


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
