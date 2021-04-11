#!/usr/bin/env python
# todo: license here

import random
import time
import sys
import rospy
import math

from xml.dom import minidom
import numpy as np

from forma3_description.msg import Pose2D,Proximity
from geometry_msgs.msg import Pose,Twist
from nav_msgs.msg import Odometry

from forma3_description.srv import GetDist,GetPose

pose_dict = {}
control = np.ones(4)

# given the robot frame, update
def update_pos(msg_pos, robot):
    global pose, state_init
    pose_dict[robot] = msg_pos.pose.pose

def update_control(msg_pos):
    global control
    control[0] = msg_pos.position.x
    control[1] = msg_pos.position.y
    control[2] = msg_pos.orientation.z
    control[3] = msg_pos.orientation.w

###

# Main controller code
def extract_control_ss(graph_file, x, y, yaw):
    global pose_dict, control
    rospy.init_node('extract_control_ss')

    #store initial pose
    control[0] = x
    control[1] = y
    control[2] = math.sin(yaw/2)
    control[3] = math.cos(yaw/2)

    rate = rospy.get_param("controller_rate")
    topic = rospy.get_param("control_point_echo")
    topicroot = rospy.get_param("root_echo")
    correction_topic = rospy.get_param("control_point_correction")
    control_point = rospy.get_param("control_point")

    correction_gain = rospy.get_param("correction_gain")
    platoon = rospy.get_param("platoon")

    # prepare the movement output
    pub = rospy.Publisher(topic, Pose2D, queue_size=10)
    pubroot = rospy.Publisher(topicroot, Pose2D, queue_size=10)
    pubcorrection = rospy.Publisher(correction_topic, Pose, queue_size=10)
    # get the control point
    rospy.Subscriber(control_point, Pose, update_control)

    # get data from the graph
    graph = minidom.parse(graph_file)
    nodes = graph.getElementsByTagName('node')

    # prepare all subscribers
    root = ""
    for node in nodes:
        name = node.getAttribute('name')
        rospy.Subscriber('/'+name+'/move_odom', Odometry, update_pos, name)
        if node.getAttribute('type').casefold() == "root":
            root = name


    # start running at rate 
    timer = rospy.Rate(rate)
    msg = Pose2D()
    msgroot = Pose2D()
    correction = Pose()
    while not rospy.is_shutdown():
        xlist = []
        ylist = []
        for _,pose in pose_dict.items():
            xlist.append(pose.position.x)
            ylist.append(pose.position.y)

        msg.x = np.mean(xlist)
        msg.y = np.mean(ylist)
        msg.theta = math.inf #unused
        pub.publish(msg)

        try:
            msgroot.x = pose_dict[root].position.x
            msgroot.y = pose_dict[root].position.y
            msgroot.theta = math.inf #unused
            pubroot.publish(msgroot)
        except:
            pass

        if platoon:
            # get the root error, and push a new control point
            errorx = control[0] - msgroot.x
            errory = control[1] - msgroot.y
        else:
            # get the centroid error, and push a new control point
            errorx = control[0] - msg.x
            errory = control[1] - msg.y

        correction.position.x = control[0] + errorx*correction_gain
        correction.position.y = control[1] + errory*correction_gain
        correction.orientation.z = control[2]
        correction.orientation.w = control[3]

        pubcorrection.publish(correction)
        # delay
        timer.sleep()


if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv=sys.argv)
        if len(argv) < 5:
            print("Please provide graph_file, x y yaw")
        extract_control_ss(argv[1], float(argv[2]), float(argv[3]), float(argv[4]))
    except rospy.ROSInterruptException:
        pass
