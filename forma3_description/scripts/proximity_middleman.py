#!/usr/bin/env python
# todo: license here

# Middleman to break out a slighly idealized 1m proximity sensor (angle & distance, angle unused).
# Used for "collisions"
# Uses the laser sensor to do this...

import math

import rospy
from sensor_msgs.msg import LaserScan
from forma3_description.msg import Proximity

def callback_relay(in_msg):
    global pub
    
    # find and save the minimum
    ang = in_msg.angle_min
    inc = in_msg.angle_increment
    min_range = math.inf
    min_ang = math.inf
    for val in in_msg.ranges:
        if val < min_range:
            min_range = val
            min_ang = ang
        ang = ang + inc

    if min_range > 1:
        min_range = math.inf
        min_ang = math.inf

    # create and publish the message
    msg = Proximity()
    msg.header = in_msg.header
    msg.distance = min_range
    msg.theta = min_ang
    
    pub.publish(msg)


def proximity_middleman():
    global pub

    rospy.init_node('proximity_middleman', anonymous=True)
    
    # get what to subscribe and publish to
    rospy.Subscriber('laser', LaserScan, callback_relay)
    pub = rospy.Publisher('proximity', Proximity, queue_size=10)

    #keep the process alive
    rospy.spin()


if __name__ == '__main__':
    try:
        proximity_middleman()
    except rospy.ROSInterruptException:
        pass
