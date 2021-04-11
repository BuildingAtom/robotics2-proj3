#!/usr/bin/env python
# todo: license here

# A service for each robot that can be called upon with the pose of the
# calling robot to get the relative pose or to get the distance... used
# for graph information sharing

import math
import time
import numpy as np

import rospy
from nav_msgs.msg import Odometry
from forma3_description.srv import GetPose,GetPoseResponse,GetDist,GetDistResponse
from forma3_description.msg import Pose2D

init = False
pos = np.zeros(2)
quat = np.zeros(2)

# Some helpers
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


# given the robot frame, update
def pose_callback(msg_pos):
    global pos, quat, init
    pos[0] = msg_pos.pose.pose.position.x
    pos[1] = msg_pos.pose.pose.position.y

    quat[0] = msg_pos.pose.pose.orientation.z
    quat[1] = msg_pos.pose.pose.orientation.w
    
    # if this is the first call to this, say we're ready
    if not init:
        init = True


# callback for the get_pose service
def getpose_callback(req):
    global pos, quat

    rqpos = np.array([req.request_pose.position.x,req.request_pose.position.y])
    rqquat = np.array([req.request_pose.orientation.z,req.request_pose.orientation.w])

    # find the relative pose from this robot to the requesting robot
    dpos = rqpos - pos
    dquat = quat_sum(rqquat,rot_quat(-quat_rot(quat)))

    # rotate accordingly
    rot = -quat_rot(quat)
    c = np.cos(rot)
    s = np.sin(rot)
    dpos = np.matmul(np.array([[c, -s], [s, c]]),dpos)

    # build and return the response
    rval = Pose2D()
    rval.x = dpos[0]
    rval.y = dpos[1]
    rval.theta = quat_rot(dquat)
    return GetPoseResponse(rval)


# callback for the get_dist service
def getdist_callback(req):
    global pos

    # find the relative distance and angle from this robot and return
    rqpos = np.array([req.request_pose.position.x,req.request_pose.position.y])
    rqquat = np.array([req.request_pose.orientation.z,req.request_pose.orientation.w])
    dpos = rqpos - pos 
    dquat = quat_sum(rqquat,rot_quat(-quat_rot(quat)))

    # get the angle from this robot's angle to the delta vector
    dpostheta = np.math.atan2(dpos[1], dpos[0])
    dpostheta = rot_quat(dpostheta)
    angle = quat_rot(quat_sum(dpostheta,rot_quat(-quat_rot(quat))))

    return GetDistResponse(np.linalg.norm(dpos), angle, quat_rot(dquat))


def pose_serv():
    global init

    rospy.init_node('pose_dist_serv', anonymous=True)
    
    # subscribe to the current pose to get the latest
    rospy.Subscriber('move_odom', Odometry, pose_callback)

    # wait for the first callback initialize these services
    while not init:
        time.sleep(.1)

    # start the services
    pserv = rospy.Service('get_pose', GetPose, getpose_callback)
    dserv = rospy.Service('get_dist', GetDist, getdist_callback)

    #keep the process alive
    rospy.spin()


if __name__ == '__main__':
    try:
        pose_serv()
    except rospy.ROSInterruptException:
        pass
