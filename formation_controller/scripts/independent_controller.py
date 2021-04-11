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

pose = Pose()
control = np.ones(3)
proximity = Proximity()
proximity.distance = math.inf

state_init = False

# Added based on the provided example code
def potential_function(actual, desired):
    return math.log(actual/desired)

# barrier tries to keep robot away by barrier amount
def barrier_function(actual, barrier):
    return int(actual < barrier)*(1/actual - 1/barrier)

###
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

# get the delta between two rotations
def delta_rot(rot1, rot2):
    return quat_rot(quat_sum(rot_quat(rot1),-rot_quat(rot2)))

# get the unit vector in the direction of the angle (inc. orientation)
def ang_vec(ang):
    return np.array([math.cos(ang),math.sin(ang),0])

###

# given the robot frame, update
def update_pos(msg_pos):
    global pose, state_init
    pose = msg_pos.pose.pose
    state_init = True

def update_prox(msg_prox):
    global proximity
    proximity = msg_prox

def update_control(msg_pos):
    global control
    control[0] = msg_pos.position.x
    control[1] = msg_pos.position.y
    control[2] = quat_rot([msg_pos.orientation.z,msg_pos.orientation.w])

###

# function to generate graph from the xml file
def build_complete_graph(graph):
    edges = graph.getElementsByTagName('edge')

    # get the information sources
    sources = []
    dests = []
    edge_data = []
    for edge in edges:
        source = edge.getAttribute('source')
        dest = edge.getAttribute('dest')
        edge_type = edge.getAttribute('type')
        # build data
        data = {}
        for information in edge.childNodes:
            if information.nodeType != minidom.Node.ELEMENT_NODE: continue
            tag = information.tagName
            if tag.casefold() == "pose":
                data['x'] = float(information.getAttribute('x'))
                data['y'] = float(information.getAttribute('y'))
                data['theta'] = float(information.getAttribute('theta'))
                data['dist'] = np.linalg.norm([data['x'], data['y']])
                data['type'] = "pose"
            else:
                data[tag] = float(information.getAttribute('value'))
                data['type'] = "dist"
        # store the graph information
        if edge_type.casefold() == "undirected":
            #way 1
            sources.append(source)
            dests.append(dest)
            edge_data.append(data)
            #way 2
            sources.append(dest)
            dests.append(source)
            data = data.copy()
            #update data (correct for the reverse direction)
            if data['type'] == "pose":
                data['x'] = -data['x']
                data['y'] = -data['y']
                data['theta'] = -data['theta']
            else:
                if 'theta' in data:
                    orient = 0.0
                    if 'orient' in data:
                        orient = data['orient']
                    data['theta'] = data['theta'] + math.pi-orient
                    if data['theta'] > math.pi:
                        data['theta'] = data['theta'] - 2*math.pi
                if 'orient' in data:
                    data['orient'] = -data['orient']
            edge_data.append(data)
        elif edge_type.casefold() == "directed":
            sources.append(source)
            dests.append(dest)
            edge_data.append(data)
        else:
            rospy.signal_shutdown("Invalid edge type found. It should be directed or undirected")
            sys.exit()
    #return the completed graph
    return (sources, dests, edge_data)


###

# Main controller code
def independent_controller(graph_file, control_type):
    global pose, control, proximity, state_init
    rospy.init_node('independent_controller', anonymous=True)

    Ke = rospy.get_param("gain_environment")
    Kg = rospy.get_param("gain_graph")
    Kgb = rospy.get_param("gain_graph_barrier")
    Kr = rospy.get_param("gain_rotation")
    Kc = rospy.get_param("gain_centroid")
    Kb = rospy.get_param("gain_barrier")
    barrier_dist = rospy.get_param("barrier_dist_strong")
    barrier_environment = rospy.get_param("barrier_dist_environment")
    barrier_graph = rospy.get_param("barrier_dist_graph")
    speed_limit = rospy.get_param("move_speed_limit")
    speed_limit_rot = rospy.get_param("move_rot_limit")
    rate = rospy.get_param("controller_rate")
    robot = rospy.get_param("robot_name")
    control_point_topic = rospy.get_param("control_point")

    # setup state update
    rospy.Subscriber('/'+robot+'/move_odom', Odometry, update_pos)
    rospy.Subscriber('/'+robot+'/proximity', Proximity, update_prox)
    # prepare the movement output
    drive_cmd = rospy.Publisher('/'+robot+'/cmd_vel', Twist, queue_size=10)

    # get data from the graph
    graph = minidom.parse(graph_file)

    # get the information sources for the robot
    graph_tuple = build_complete_graph(graph)

    # if the node is follow_graph, remove the centroid gain, otherwise subscribe
    if control_type.casefold() == "follow_graph":
        Kc = 0
    else:
        rospy.Subscriber(control_point_topic, Pose, update_control)
    root = control_type.casefold() == "root"

    # isolate the specific sources we care about, wait for each
    sources = []
    shared_data = []
    for source, dest, edge_data in zip(graph_tuple[0], graph_tuple[1], graph_tuple[2]):
        if dest == robot:
            rospy.wait_for_service('/'+source+'/get_dist') # pick arbitary one
            sources.append(source)
            shared_data.append(edge_data)

    # gather the appropriate service calls
    pose_services = []
    pose_data = []
    dist_services = []
    dist_data = []
    for source, data in zip(sources, shared_data):
        if data['type'] == "pose":
            pose_services.append(rospy.ServiceProxy('/'+source+'/get_pose', GetPose))
            pose_data.append(data)
        else:
            dist_services.append(rospy.ServiceProxy('/'+source+'/get_dist', GetDist))
            dist_data.append(data)


    # start running at rate 
    timer = rospy.Rate(rate)
    msg = Twist()
    while not rospy.is_shutdown():
        # hold until there is actually information available
        if not state_init:
            timer.sleep()
            continue

        # environment barrier (if in range)
        env_contrib = np.zeros(3)
        rot = quat_rot([pose.orientation.z,pose.orientation.w])
        if proximity.distance < math.inf:
            c = np.cos(rot)
            s = np.sin(rot)
            env_val = barrier_function(proximity.distance, barrier_environment)*Ke
            env_barrier = barrier_function(proximity.distance, barrier_dist)*Kb
            env_vec = -ang_vec(proximity.theta) #note the negative
            env_contrib = (env_val + env_barrier)*env_vec
            env_contrib = np.matmul(np.array([[c, -s, 0], [s, c, 0], [0,0,1]]),env_contrib)

        # graph potential + barrier pose
        pose_contrib = np.zeros(3)
        for serv, desired in zip(pose_services, pose_data):
            pose_delta = serv(pose).pose_delta
            frame = rot-pose_delta.theta
            x = (desired['x'] - pose_delta.x)*Kg
            y = (desired['y'] - pose_delta.y)*Kg
            theta = -delta_rot(pose_delta.theta,-desired['theta'])*Kr
            contrib = np.array([x,y,theta])
            c = np.cos(frame)
            s = np.sin(frame)
            contrib = np.matmul(np.array([[c, -s, 0], [s, c, 0], [0,0,1]]),contrib)
            #add a barrier element
            barrier_vec = np.array([pose_delta.x, pose_delta.y, 0.0])
            dist = np.linalg.norm(barrier_vec)
            dist_val = barrier_function(dist, barrier_graph*desired['dist'])*Kgb
            dist_val = dist_val + barrier_function(dist, barrier_dist)*Kb
            barrier_vec = barrier_vec/dist*dist_val
            barrier_vec = np.matmul(np.array([[c, -s, 0], [s, c, 0], [0,0,1]]),barrier_vec)
            #add a circulatory element
            postheta = np.math.atan2(pose_delta.y, pose_delta.x)
            dpostheta = np.math.atan2(desired['y'], desired['x'])
            angle_pot = delta_rot(postheta,-dpostheta)*Kg
            theta_vec = ang_vec(postheta-math.pi/2) * angle_pot
            theta_vec = np.matmul(np.array([[c, -s, 0], [s, c, 0], [0,0,1]]),theta_vec)
            pose_contrib = pose_contrib + contrib + barrier_vec + theta_vec

        # graph potential + barrier dist
        dist_contrib = np.zeros(3)
        for serv, desired in zip(dist_services, dist_data):
            dist_delta = serv(pose)
            dist_val = potential_function(dist_delta.distance, desired['dist'])*Kg
            dist_val = dist_val - barrier_function(dist_delta.distance, barrier_graph*desired['dist'])*Kgb
            dist_val = dist_val - barrier_function(dist_delta.distance, barrier_dist)*Kb
            vec = -ang_vec(rot-dist_delta.orientation)
            in_vec = np.copy(vec)
            in_vec[0] = vec[0]*math.cos(dist_delta.angle) - vec[1]*math.sin(dist_delta.angle)
            in_vec[1] = vec[1]*math.cos(dist_delta.angle) + vec[0]*math.sin(dist_delta.angle)
            contrib_vec = in_vec * dist_val
            if 'theta' in desired:
                # the angle from source to this will always be increasing counter clockwise
                theta_val = delta_rot(dist_delta.angle,-desired['theta'])*Kg
                # since angle goes into this robot, away from source, get the vector 90 deg
                circ_vec = np.copy(in_vec)
                circ_vec[0] = in_vec[0]*math.cos(math.pi/2) - in_vec[1]*math.sin(math.pi/2)
                circ_vec[1] = in_vec[1]*math.cos(math.pi/2) + in_vec[0]*math.sin(math.pi/2)
                theta_vec = circ_vec * theta_val
                contrib_vec = contrib_vec + theta_vec
            if 'orient' in desired:
                theta = -delta_rot(dist_delta.orientation,-desired['orient'])*Kr
                contrib_vec[2] = theta
            dist_contrib = dist_contrib + contrib_vec

        # centroid potential (only consider the rotation if root)
        centroid_contrib = np.zeros(3)
        if Kc:
            centroid_contrib[0] = (control[0] - pose.position.x)*Kc
            centroid_contrib[1] = (control[1] - pose.position.y)*Kc
            if root:
                centroid_contrib[2] = -delta_rot(rot,-control[2])*Kr

        # overall vec
        overall = env_contrib + pose_contrib + dist_contrib + centroid_contrib
        # make sure to clip at limit
        speed = np.linalg.norm(overall[0:1])
        if speed > speed_limit:
            mult = speed_limit / speed
            overall[0:1] = overall[0:1]*mult
        if overall[2] > speed_limit_rot:
            overall[2] = speed_limit_rot

        # create and publish message
        msg.linear.x = overall[0]*math.cos(-rot) - overall[1]*math.sin(-rot)
        msg.linear.y = overall[1]*math.cos(-rot) + overall[0]*math.sin(-rot)
        msg.angular.z = overall[2]
        drive_cmd.publish(msg)

        # delay
        timer.sleep()


if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv=sys.argv)
        if len(argv) < 3:
            print("Please provide graph_file and controller_type")
        independent_controller(argv[1], argv[2])
    except rospy.ROSInterruptException:
        pass
