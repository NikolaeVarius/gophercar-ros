# Writes image and steering data synchronized via timestamp
# mostly since I'm too lazy to want to rewrite this in go at this very second
# Also since I'm not a huge fan of the gocv bindings for image manipulation (is slow)

import message_filters
import rospy
import sys
from sensor_msgs.msg import CompressedImage, CameraInfo, Joy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import json
from rospy_message_converter import json_message_converter


# from cv_bridge.boost.cv_bridge_boost import getCvType

bridge = CvBridge()

# Change this to somewhere not here at some point
image_path = "./data/images/"
joy_path = "./data/joy/"

def callback(image, joy):
    print("handling event")
    img_timestamp = image.header.stamp.secs
    joy_timestamp = joy.header.stamp.secs
    
    try:
        # handle image
        cv2_img = bridge.compressed_imgmsg_to_cv2(image, "bgr8")
        image_filename = image_path + str(img_timestamp) + "-image.jpg"
        cv2.imwrite(image_filename, cv2_img)

        # handle file
        json_str = json_message_converter.convert_ros_message_to_json(joy)
        json_filename = joy_path + str(img_timestamp) + ".json" 
        with open(json_filename, 'w') as json_file:
            json.dump(json_str, json_file)
    except CvBridgeError as e:
        print(e)
    except:
        sys.exit(1)



def main(args):
    image_sub = message_filters.Subscriber('output/image_raw/compressed', CompressedImage)
    joy_sub = message_filters.Subscriber('joy', Joy)

    # http://wiki.ros.org/message_filters/ApproximateTime
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, joy_sub], 10, 1)
    ts.registerCallback(callback)
    
    try:
        print("Spinning")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    rospy.init_node('steerTraining', anonymous=True)
    main(sys.argv)
