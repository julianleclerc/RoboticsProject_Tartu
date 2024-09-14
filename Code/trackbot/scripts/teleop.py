#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import curses

def teleop(screen):
    # Initialize the ROS node
    rospy.init_node('teleop', anonymous=True)

    # /cmd_vel publish
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)

    # /speed publish
    pub_speed = rospy.Publisher('/speed', Int32, queue_size=1)

    # Set curses to non-blocking mode
    screen.nodelay(True)  # getch() will be non-blocking
    screen.clear()

    # Print instructions on the screen
    screen.addstr(0, 0, "Use arrow keys to control the robot. Press 'q' to quit.")
    
    
    speed = 50
    speed_jump = 10

    try:
        while not rospy.is_shutdown():
            # Refresh screen and wait for key press
            screen.refresh()
            char = screen.getch()

            # Create Twist message
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            speed_msg = Int32()

            screen.addstr(2, 0, "Last key pressed: {}".format(char).ljust(40))

            # Direction Controll

            if char == 259:
                twist.linear.x = 1.0
                screen.addstr(1, 0, "Moving forward ")
            elif char == 258:
                twist.linear.x = -1.0
                screen.addstr(1, 0, "Moving backward")
            if char == 261:
                twist.angular.z = -1.0
                screen.addstr(1, 0, "Turning right  ")
            elif char == 260:
                twist.angular.z = 1.0
                screen.addstr(1, 0, "Turning left   ")
            if char == ord('q'):
                break  # Exit the loop if 'q' is pressed
            elif char == -1:
                screen.addstr(1, 0, "Stopping       ")

            # Speed Controll
            if char == 46:
                if speed - speed_jump >= 0:
                    speed = speed - 10
                    screen.addstr(5, 0, "Speed: {}".format(speed).ljust(40))
            elif char == 47:
                if speed + speed_jump <= 100:
                    speed = speed + 10
                    screen.addstr(5, 0, "Speed: {}".format(speed).ljust(40))
            
            speed_msg = speed

            # Publish the command
            pub.publish(twist)
            pub_speed.publish(speed_msg)
            rate.sleep()
    finally:
        # Ensure curses is closed properly
        curses.endwin()

# Wrap the teleop function with curses to handle screen, keyboard events
curses.wrapper(teleop)
