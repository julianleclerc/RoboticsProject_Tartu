#!/usr/bin/env python3
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import rospy
from cv_bridge import CvBridge, CvBridgeError

class CameraFeed:
    def __init__(self):
        rospy.init_node('camera_feed', anonymous=True)
        self.publisher = rospy.Publisher('camera_frame', Image, queue_size=1)
        self.subscriber = rospy.Subscriber('request_frame', Int32, self.capture_and_publish_frame)
        
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

        if not self.cap.isOpened():
            rospy.logerr('Error: Could not open video device /dev/video0.')
            rospy.signal_shutdown('Error: Could not open video device /dev/video0.')
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.br = CvBridge()

    def capture_and_publish_frame(self, msg):
        rospy.loginfo('Received Image Request')
        ret, frame = self.cap.read()
        if ret:
            # Rotate the image 90 degrees clockwise
            frame_rotated = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            
            rospy.loginfo('Publishing video frame')
            try:
                self.publisher.publish(self.br.cv2_to_imgmsg(frame_rotated, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
        else:
            rospy.logerr('Error capturing frame')

    def shutdown(self):
        if self.cap.isOpened():
            self.cap.release()
            rospy.loginfo('Camera released')

if __name__ == '__main__':
    try:
        node = CameraFeed()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
