#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Define pin numbers for the ultrasonic sensor
pingPin = 23
echoPin = 24

# Set up the GPIO pins
GPIO.setup(pingPin, GPIO.OUT)
GPIO.setup(echoPin, GPIO.IN)

def measure_distance():
    """Measure the distance using the ultrasonic sensor."""
    # Send a pulse
    GPIO.output(pingPin, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(pingPin, False)

    pulse_start_time = None
    pulse_end_time = None

    # Wait for the response
    while GPIO.input(echoPin) == 0:
        pulse_start_time = time.time()
    while GPIO.input(echoPin) == 1:
        pulse_end_time = time.time()

    if pulse_start_time and pulse_end_time:
        # Calculate distance
        pulse_duration = pulse_end_time - pulse_start_time
        distance = round(pulse_duration * 17150, 2)  # Convert to cm
        return distance
    else:
        return None

def us_detection():
    """Detect obstacles and publish the results as a boolean."""
    pub_obstacle = rospy.Publisher('obstacle_us', Bool, queue_size=1)
    rospy.init_node('us_detection', anonymous=True)
    rate = rospy.Rate(7)  # 7hz

    while not rospy.is_shutdown():
        distance = measure_distance()
        stop_distance = 20
        presence_obstacle = (distance is not None and distance <= stop_distance)

        # Publish the presence of an obstacle
        pub_obstacle.publish(presence_obstacle)
        rospy.loginfo(f"Distance: {distance} cm, Obstacle: {'Yes' if presence_obstacle else 'No'}")
        rate.sleep()

if __name__ == '__main__':
    try:
        us_detection()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Ensure GPIO resources are freed
        GPIO.cleanup()
