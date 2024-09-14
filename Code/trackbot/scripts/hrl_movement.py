#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
import time

class HRL_Movement:
    def __init__(self):
        rospy.init_node('hrl_movement', anonymous=True)

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.sub_x = rospy.Subscriber('hrl_linear_request', Int32, self.set_linear_x)
        self.sub_z = rospy.Subscriber('hrl_angular_request', Int32, self.set_angular_z)
        self.stop_sub = rospy.Subscriber('hrl_stop_request', Bool, self.stop_robot)

        self.speed_sub = rospy.Subscriber("speed", Int32, self.set_speed)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10) 

        self.msg_x = 0
        self.msg_z = 0

        self.dir_x = 0
        self.dir_z = 0
        
        global speed
        speed = 70

        self.stop_state = False

    def set_speed(self, data):
        global speed
        speed = data.data
        rospy.loginfo("Speed changed to: %s", speed)

    def set_linear_x(self, msg):
        global speed
        if msg.data != 0:
            speed_var = (100-speed)*0.2+29
            self.msg_x = int( abs(msg.data) * speed_var / 50)
            rospy.loginfo("msg_x : %s", self.msg_x)
            if msg.data > 0:
                self.dir_x = 1.0
            else:
                self.dir_x = -1.0
            self.linear_x = msg.data
            self.stop_state = False
            rospy.loginfo("Received linear x value : %s", msg.data)
            self.publish_twist()

    def set_angular_z(self, msg):
        global speed
        if msg.data != 0:
            speed_var = (100-speed)*0.2 + 9
            self.msg_z = int( (abs(msg.data) * speed_var) / 180 ) # abs(msg.data) to get only the abs scale then transform from global angle to motor
            if msg.data > 0:
                self.dir_z = 1.0
            else:
                self.dir_z = -1.0
            self.angular_z = msg.data
            self.stop_state = False
            rospy.loginfo("Received angular value : %s", msg.data)
            self.publish_twist()

    def stop_robot(self, msg):
        if msg.data:
            self.linear_x = 0
            self.angular_z = 0
            self.stop_state = True
            rospy.loginfo("Received stop request")
            self.publish_twist()

    def publish_twist(self):
        twist = Twist()
        while self.msg_x > 0 or self.msg_z > 0:
            # initiate
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if self.msg_x > 0:
                twist.linear.x = self.dir_x / 10
                self.msg_x -= 1
            else:
                twist.linear.x = 0.0

            if self.msg_z > 0:
                twist.angular.z = self.dir_z / 10
                self.msg_z -= 1
            else:
                twist.angular.z = 0.0

            # Check for no stop
            if self.stop_state == False:
                rospy.loginfo("Publishing//")
                self.pub.publish(twist)
            else:
                rospy.loginfo("Stopping //")
                break

            time.sleep(0.1)

        rospy.loginfo("Done :)")
        self.stop_state = False

        # Ensure reset of param
        self.msg_x = 0
        self.msg_z = 0
        self.dir_x = 0
        self.dir_z = 0      

        # Ensure the robot stops after moving
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = HRL_Movement()
    node.run()


#for speed 
#   50% -> 23 = 180 rotation
#   60% -> 18
#   70% -> 15 = 180 rotation
#   80% -> 13 
#   90% -> 11
#   100% -> 9 = 180 rot
# under 50 is unsafe for motors, too much torque for the speed
# between 60 and 100 speeds -> rotation = int ( in * speed_var / 180) [in is a value of angle]
# speed_var = (100-speed)*0.2 +9


# now for distances
#   
#
#
#   70 -> 35 = 50cm
#
#   100 -> 35-0.2*30: 29= 50cm 
# should be speed var = (70-speed)*0.2+35*1/50 = normalised for 1cm
# int ( in * speed_var ) unit = 1cm
