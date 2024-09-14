#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

class stoppingRobot:
    def __init__(self):
        rospy.init_node('stoppingRobot', anonymous=True)

        self.pub = rospy.Publisher('forceStop', Bool, queue_size=1)
        self.sub = rospy.Subscriber('obstacle_us', Bool, self.obstacle_callback)
        self.sub = rospy.Subscriber('hrl_stop_request', Bool, self.hrl_stop_callback)

        self.hrl_stop_level = False

    def obstacle_callback(self, msg):
        if self.hrl_stop_level == False:
            stop_msg = Bool()
            stop_msg.data = msg.data  

            self.pub.publish(stop_msg)
            if msg.data:
                rospy.loginfo("Obstacle detected, sending stop command.")
            else:
                rospy.loginfo("No obstacle detected, sending continue command.")
        else:
            rospy.loginfo("Stopped through HRL")
            self.pub.publish(True)

    def hrl_stop_callback(self, msg):
        stop_msg = Bool()
        stop_msg.data = msg.data 

        self.pub.publish(stop_msg)
        self.hrl_stop_level = stop_msg.data

        if msg.data:
            rospy.loginfo("Stopping Robot Through HRL Interface")
        else:
            rospy.loginfo("Released Robot Through HRL Interface")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = stoppingRobot()
    node.run()
