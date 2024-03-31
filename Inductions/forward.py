#!/usr/bin/env python3

import rospy
# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist

class move(object):

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('spin_circles')
        # setup publisher to the cmd_vel ROS topic
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
        # setup the Twist message we want to send
        vel = Twist()
        vel.linear.x = 0.2

        # allow the publisher enough time to set up before publishing the first msg
        rospy.sleep(1)

        # publish the message
        self.pub.publish(vel)

if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = move()
    node.run()

