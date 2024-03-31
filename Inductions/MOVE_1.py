#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    global obstacle_detected
    ranges = msg.ranges
    forward_distance = min(ranges[340:380])
    rospy.loginfo(forward_distance)
    if forward_distance < 0.5:
        obstacle_detected = True
    else:
        obstacle_detected = False

def move_turtlebot(linear_x, angular_z):
    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

def main():
    global obstacle_detected
    global pub
    rospy.init_node('MOVE_1')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if not obstacle_detected:
            move_turtlebot(0.2, 0)  # Move forward at a constant velocity
        else:
            move_turtlebot(0, 0)  # Stop the bot
            rospy.sleep(1)  # Wait for 1 second
            
            # Adjust the angular velocity for a 90-degree turn
            turn_angular_velocity = 1.15# Experiment with this value
            move_turtlebot(0, turn_angular_velocity)  # Turn
            rospy.sleep(1.57 / turn_angular_velocity)  # Wait for the turn
            
            move_turtlebot(0.2, 0)  # Move forward again
            
        rate.sleep()

if _name_ == '_main_':
    obstacle_detected = False
    main()