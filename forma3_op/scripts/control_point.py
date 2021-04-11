#!/usr/bin/env python
# todo: license here

# takes the input from a cmd_vel twist controller and uses it to drive a control point.

import math
import sys

import rospy
import numpy as np
from geometry_msgs.msg import Twist,Pose

fwd = 0.0
left = 0.0
turn = 0.0


# convert the z rotation to incomplete quaternion (z,w)
def rot_quat(rot):
    return np.array([math.sin(rot/2),math.cos(rot/2)])

#convert incomplete quaternion (z,w) to z rotation
def quat_rot(quat):
    return math.atan2(2*(quat[1]*quat[0]),1-2*(quat[0]*quat[0]))

#given two incomplete quaternions (numpy vector of z,w), sum the rotations
def quat_sum(quat1, quat2):
    w = quat1[1]*quat2[1] - quat1[0]*quat2[0]
    z = quat1[1]*quat2[0] + quat2[1]*quat1[0]
    return np.array([z,w])
###

def callback_relay(in_msg):
    global fwd, left, turn
    
    # the only input movements are going to be forwards/backwards, sideways velocity, and rotation
    fwd = in_msg.linear.x
    left = in_msg.linear.y
    turn = in_msg.angular.z


def control_point(x, y, yaw):
    global fwd, left, turn

    rospy.init_node('control_point')
    
    # get the rate
    rate = rospy.get_param('rate', 20)
    
    # setup publishers and subscribers
    pub = rospy.Publisher('control_point', Pose, queue_size=10)

    rospy.Subscriber('cmd_vel', Twist, callback_relay)

    # start running at rate
    timer = rospy.Rate(rate)
    dtime = 1.0/rate
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    orientation = rot_quat(yaw)
    pose.orientation.z = orientation[0]
    pose.orientation.w = orientation[1]
    while not rospy.is_shutdown():
        #update position
        pose.position.x = pose.position.x + fwd*dtime
        pose.position.y = pose.position.y + left*dtime

        #update orientation
        dquat = rot_quat(turn*dtime)
        newquat = quat_sum(np.array([pose.orientation.z,pose.orientation.w]),dquat)
        pose.orientation.z = newquat[0]
        pose.orientation.w = newquat[1]

        #publish new control point
        pub.publish(pose)

        # delay
        timer.sleep()


if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv=sys.argv)
        if len(argv) < 4:
            print("Please provide x y yaw")
        control_point(float(argv[1]), float(argv[2]), float(argv[3]))
    except rospy.ROSInterruptException:
        pass
