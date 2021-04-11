#!/usr/bin/env python

# Given the graph, spawn all the robot nodes

import sys
import random
import rospy
import roslaunch

from xml.dom import minidom
from skimage.io import imread
import numpy as np

# given a number, return a list of numpy pairs of valid, non-overlapping spawns.
def generate_spawns(num):
    # get bounds of map and min dist between spawn points
    bounds = rospy.get_param('/map/bounds', [0.2, 0.2, 9.8, 9.8])
    min_dist = rospy.get_param('min_spawn_dist')
    max_dist = rospy.get_param('max_spawn_dist')

    # load in the map
    map_path = rospy.get_param('/map/configuration_map', "./room.pgm")
    room_voxel = rospy.get_param('/map/voxel_size', 0.05)
    room_map = imread(map_path)
    # rotate it to match the room (room_map[x][y])
    room_map = np.rot90(room_map, k=-1, axes=(0,1))

    # list of spawn locations
    spawns = []
    for i in range(num):
        # generate values (tbh, I don't care about randomizing rotation that much here)
        while True:
            x = random.random()*(bounds[2]-bounds[0]) + bounds[0];
            y = random.random()*(bounds[2]-bounds[0]) + bounds[0];
            pos = np.array([x,y])

            # test if it is a valid position
            if not room_map[int(x / room_voxel)][int(y / room_voxel)]:
                # if it fails here, then it's in an obstacle, so short circuit
                continue

            # test if it's nearby to others
            closest = max_dist + 1
            for spawn in spawns:
                if np.linalg.norm(spawn-pos) < closest:
                    closest = np.linalg.norm(spawn-pos)
            if (not len(spawns)) or (closest > min_dist and closest < max_dist):
                # if we're good here, break out of randomization
                spawns.append(pos)
                break
    return spawns


# read out and spawn the nodes for the graph
def spawn_nodes(graph_file, spawnx, spawny, spawnyaw):
    graph = minidom.parse(graph_file)
    nodes = graph.getElementsByTagName('node')

    # get the spawner package
    package = rospy.get_param('robot_spawner_pkg')
    # get all the nodes to launch
    spawner = rospy.get_param('robot_spawner_file')

    # get the control package
    package_ctrl = rospy.get_param('robot_control_pkg')
    spawner_ctrl = rospy.get_param('robot_control_file')

    # get relavent parameters
    #description_file

    # generate spawn points
    locations = generate_spawns(len(nodes))

    # start up a new roslaunch process
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # setup each robot for spawn
    launch_files = []
    names = []
    controls = []
    for node, location in zip(nodes, locations):
        name = node.getAttribute('name')
        control = node.getAttribute('type')
        # spawner files
        cli_spawner = [package, spawner, 'robotname:='+name, 'x:='+location.astype(str)[0], 'y:='+location.astype(str)[1]]
        spawner_file = roslaunch.rlutil.resolve_launch_arguments(cli_spawner)
        spawner_args = cli_spawner[2:]
        launch_files.append((spawner_file[0], spawner_args))
        # controller files
        cli_control = [package_ctrl, spawner_ctrl, 'robotname:='+name, 'graphfile:='+graph_file, 'controltype:='+control]
        control_file = roslaunch.rlutil.resolve_launch_arguments(cli_control)
        control_args = cli_control[2:]
        launch_files.append((control_file[0], control_args))

    #controller for extracting centroid and adding error to control point
    ss_ctrl = rospy.get_param('robot_ss_file')
    is_platoon = rospy.get_param('is_platoon')
    control_ss_feedback = [package_ctrl, ss_ctrl, 'graphfile:='+graph_file, 'x:='+str(spawnx), 'y:='+str(spawny), 'yaw:='+str(spawnyaw), 'is_platoon:='+str(is_platoon)]
    control_ss_file = roslaunch.rlutil.resolve_launch_arguments(control_ss_feedback)
    control_ss_args = control_ss_feedback[2:]
    launch_files.append((control_ss_file[0], control_ss_args))

    nodes = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

    nodes.start()

    return nodes


def spawn_graph(graph_file, spawnx, spawny, spawnyaw):
    rospy.init_node('spawn_graph', anonymous=True)

    nodes = spawn_nodes(graph_file, spawnx, spawny, spawnyaw)

    # Start another node for control of graph control point
    #node = roslaunch.core.Node(package, executable)
    #launch.launch(node)
    try:
        nodes.spin()
    finally:
        # After Ctrl+C, stop all nodes from running
        nodes.shutdown()

if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv=sys.argv)
        if len(argv) < 5:
            print("Please enter a file name and position x y Yaw")
        else:
            spawn_graph(argv[1], float(argv[2]), float(argv[3]), float(argv[4]))
    except rospy.ROSInterruptException:
        pass
