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
from record_node.msg import Actuator
# from rospy_message_converter import json_message_converter

count = 0
# from cv_bridge.boost.cv_bridge_boost import getCvType
bridge = CvBridge()

# Change this to somewhere not here at some point
image_path = "./data/images/"
joy_path = "./data/joy/"

def callback(actuator):
    count = count + 1
    
    if count % 10 == 0:
        print("Handled Events: " + str(count))

    try:
        # handle image
        # cv2_img = bridge.compressed_imgmsg_to_cv2(image, "bgr8")
        # image_filename = image_path + str(img_timestamp) + "-image.jpg"
        # cv2.imwrite(image_filename, cv2_img)
        print(actuator)
        # handle file
        # json_str = json_message_converter.convert_ros_message_to_json(joy)
        # json_filename = joy_path + str(img_timestamp) + ".json" 
        # with open(json_filename, 'w') as json_file:
        #     json.dump(json_str, json_file)
    except CvBridgeError:
        print(CvBridgeError)
    except Exception as e:
        print(e)



def main(args):
    # image_sub = message_filters.Subscriber('output/image_raw/compressed', CompressedImage)
    # actuator_sub = message_filters.Subscriber('actuator', Actuator)
    rospy.Subscriber('actuator', Actuator, callback)

    # https://docs.ros.org/api/message_filters/html/python/#message_filters.ApproximateTimeSynchronizer
    # Trying out between ApproximateTimeSynchronizer and TimeSynchronizer
    # I don't think 1 second of approximity is good enough consiering we are generating roughly 30 fps
    # ts = message_filters.ApproximateTimeSynchronizer([image_sub, actuator_sub], 100, 1)
    # ts = message_filters.TimeSynchronizer([actuator_sub], 100)
    # ts.registerCallback(callback)
    
    try:
        print("Spinning")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    rospy.init_node('steerTraining', anonymous=True)
    main(sys.argv)
