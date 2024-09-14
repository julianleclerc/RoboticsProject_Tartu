#!/usr/bin/env python3
import cv2
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge, CvBridgeError

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber('camera_frame', Image, self.image_callback, queue_size=1)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)  # Refresh display

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    node = ImageSubscriber()
    node.run()

#http://192.168.0.102:8080/stream?topic=/camera_frame
