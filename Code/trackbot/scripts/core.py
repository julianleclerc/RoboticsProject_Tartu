#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import Bool

import RPi.GPIO as GPIO

# Motor pin assignments
dir1_pin = 5
dir2_pin = 6
pwm1_pin = 13
pwm2_pin = 12

# Setup GPIO mode and pin initial configuration
GPIO.setmode(GPIO.BCM)
GPIO.setup([dir1_pin, dir2_pin, pwm1_pin, pwm2_pin], GPIO.OUT)

# Initialize PWM objects
pwm1 = GPIO.PWM(pwm1_pin, 1000)  # 1000 Hz frequency
pwm2 = GPIO.PWM(pwm2_pin, 1000)

# Start PWM with 0% duty cycle (off)
pwm1.start(0)
pwm2.start(0)

global speed
global stop_robot
stop_robot = False
speed = 70

def stopMovement(data):
    global stop_robot
    stop_robot = data.data
    rospy.loginfo("Robot force stop condition: %s", stop_robot)

def set_speed(data):
    global speed
    speed = data.data
    rospy.loginfo("Speed changed to: %s", speed)

def moveCondition(data):

    rospy.loginfo("Received a /cmd_vel message!")

    if stop_robot == True:
        if data.linear.x == 0:
            roboMovement(data)
        else:
            rospy.loginfo("Remove Obstacle / Clear Stop")
            data.linear.x = 0
            roboMovement(data)
    else:
        roboMovement(data)


def roboMovement(data):

    #rospy.loginfo("Linear Components: [X: %s, Y: %s, Z: %s]" % (data.linear.x, data.linear.y, data.linear.z))
    #rospy.loginfo("Angular Components: [X: %s, Y: %s, Z: %s]" % (data.angular.x, data.angular.y, data.angular.z))

    # initialise
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)

    # Linear Movement
    if data.linear.x != 0: 
        if data.linear.x > 0:
            GPIO.output(dir1_pin, GPIO.LOW)
            GPIO.output(dir2_pin, GPIO.LOW)
            pwm1.ChangeDutyCycle(speed)
            pwm2.ChangeDutyCycle(speed)
        else:
            GPIO.output(dir1_pin, GPIO.HIGH)
            GPIO.output(dir2_pin, GPIO.HIGH)
            pwm1.ChangeDutyCycle(speed)
            pwm2.ChangeDutyCycle(speed)

 
    if data.angular.z != 0:
        turn_speed_adjust = abs(data.angular.z) * 50  # Scale turning influence
        if data.angular.z > 0:
            # Right turn adjustment
            pwm1.ChangeDutyCycle(speed)
            pwm2.ChangeDutyCycle(speed)
            GPIO.output(dir1_pin, GPIO.LOW)
            GPIO.output(dir2_pin, GPIO.HIGH)
        else:
            # Left turn adjustment
            pwm1.ChangeDutyCycle(speed)
            pwm2.ChangeDutyCycle(speed)
            GPIO.output(dir1_pin, GPIO.HIGH)
            GPIO.output(dir2_pin, GPIO.LOW)



def listener():
    rospy.init_node('core', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, moveCondition)
    rospy.Subscriber("speed", Int32, set_speed)
    rospy.Subscriber('forceStop', Bool, stopMovement)
    rospy.spin()

def cleanup():
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        cleanup()
    finally:
        cleanup()
