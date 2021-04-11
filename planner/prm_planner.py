#!/usr/bin/env python

# Probability roadmap planner based on latin-hypercube sampling

from skimage.io import imread
from skimage.io import imsave
from skimage import measure
from scipy import ndimage
import matplotlib.pyplot as plt
import numpy as np

import time
import sys
import os
import math

import pickle
import matplotlib.pyplot as plt
import numpy as np

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose
from forma3_description.msg import Pose2D

from skimage.io import imread
from skimage.draw import line_aa
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import shortest_path
import numpy as np

import roslaunch

control = np.zeros(2)
ready = False

#with the help of image routines, see if paths are valid
def does_path_intersect(config_map, point1, point2):
    # make a line and extract the map along that line
    point1 = point1.astype(int)
    point2 = point2.astype(int)
    rr, cc, val = line_aa(point1[0], point1[1], point2[0], point2[1])
    # add 1 so the free space as 255 overflows to 0 (uint8)
    intersect = np.sum((config_map[rr, cc]+1).astype(int))
    # if the value is non-zero, the path must have intersected with something somewhere.
    return bool(intersect)


#build the prm graph, and allow extensions if needed
def build_prm_graph(config_map, voxel_size, divisions, subset_size, graph=None):
    # for now, we know our room size is 10x10
    sample_square = 10.0/divisions;

    rng = np.random.default_rng()

    x_box = np.arange(divisions, dtype=float)
    y_box = np.arange(divisions, dtype=float)
    rng.shuffle(x_box)
    rng.shuffle(y_box)
    box_points = np.stack((x_box, y_box), axis=-1)
    internal_points = rng.uniform(0,sample_square, (divisions, 2))
    # completed sampling
    lhs_sample = box_points * sample_square + internal_points

    # delete anything that's invalid
    mask = np.transpose((lhs_sample / voxel_size).astype(int))
    mask = (config_map[tuple(mask)]).astype(bool)
    sample = lhs_sample[mask]

    # if we don't have an existing graph, make it. otherwise append
    if not graph:
        graph = []
        ids = list(np.arange(np.size(sample)/2, dtype=int))
        ids = dict(zip(ids, tuple(sample)))
        # id's:positions, starts, ends, lengths        
        graph.append(ids)
        graph.append([])
        graph.append([])
        graph.append([])
    else:
        last = max(graph[0])
        ids = list(last+1+np.arange(np.size(sample)/2, dtype=int))
        ids = dict(zip(ids, tuple(sample)))
        graph[0].update(ids)

    end = max(ids)

    # for each sample left, get a sample of other nodes, and try to build a path.
    # if a path exists, push it to the graph
    for key in ids:
        subset = rng.integers(0, end, size=subset_size)
        #we'll just ignore duplicates here because they get ignored in the pathfinding algorithm anyway
        for node in subset:
            if node == key:
                continue
            if does_path_intersect(config_map, graph[0][key]/voxel_size, graph[0][node]/voxel_size):
                continue
            # compute path length
            length = np.linalg.norm(graph[0][key] - graph[0][node])
            # push to graph
            graph[1].append(key)
            graph[2].append(node)
            graph[3].append(length)
    # the graph is sparsely defined to later use scipy coo matrix and shortest path
    # also nice because it can handle duplicates
    return graph

def current_pos(in_msg):
    global control, ready
    control[0] = in_msg.x
    control[1] = in_msg.y
    ready = True

def run_plan(plan):
    global control
    #move a control point and wait for arrival
    control_point_topic = rospy.get_param("control_point_topic")
    feedback_topic = rospy.get_param("feedback_topic")
    tolerance = rospy.get_param("control_tolerance")

    rospy.Subscriber(feedback_topic, Pose2D, current_pos)
    controller = rospy.Publisher(control_point_topic, Pose, queue_size=10)

    target = Pose()
    for next in plan:
        target.position.x = next[0]
        target.position.y = next[1]
        dist = np.linalg.norm(control-next)
        while dist > tolerance:
            controller.publish(target)
            dist = np.linalg.norm(control-next)
            rospy.sleep(0.02)
    print("DONE!")


# run the main planner routine
def prm_planner(targetx, targety):
    global control, ready

    rospy.init_node('prm_planner', anonymous=True)

    feedback_topic = rospy.get_param("feedback_topic")
    rospy.Subscriber(feedback_topic, Pose2D, current_pos)

    # give some time for everything to settle
    rospy.sleep(10)
    print("starting planner")

    # load in the map
    map_path = rospy.get_param('/map/free_space_map', "./room.pgm")
    free_space = imread(map_path)
    map_path = rospy.get_param('/map/configuration_map', "./room.pgm")
    config_space = imread(map_path)
    room_voxel = rospy.get_param('/map/voxel_size', 0.05)
    # rotate it to match the room (room_map[x][y])
    free_space = np.rot90(free_space, k=-1, axes=(0,1))
    config_space = np.rot90(config_space, k=-1, axes=(0,1))

    divisions = rospy.get_param('prm_divisions', 25)
    subset_size = rospy.get_param('prm_nodes_to_try', 12)
    # how close the robot and goal need to be to a graph point to "get on the path"
    hookup_dist = rospy.get_param('prm_hookup_dist', 1.0)

    # get the state & goal
    # we're running off the assumption that everything is already running
    #get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    #robot = get_model_state('maru2','')
    while not rospy.is_shutdown():
        if ready:
            break
        rospy.sleep(0.2)
    initial_pos = np.copy(control)
    goal = np.array([targetx, targety], dtype=float)

    # start timing
    time_planning_start = rospy.Time.now()
    graph = build_prm_graph(config_space, room_voxel, divisions, subset_size)
    # keep building onto the graph until there's a close hookup point and route
    start_id = 0
    close_goal_ids = []
    closest_goal_dist = 10.0
    close_robot_ids = []
    closest_robot_dist = 10.0
    while True:
        new_start_id = max(graph[0])+1
        for ids in range(start_id, new_start_id):
            dist = np.linalg.norm(initial_pos - np.array(graph[0][ids], dtype=float))
            if dist < closest_robot_dist:
                closest_robot_dist = dist
            if dist < hookup_dist:
                close_robot_ids.append(ids)

            dist = np.linalg.norm(goal - np.array(graph[0][ids], dtype=float))
            if dist < closest_goal_dist:
                closest_goal_dist = dist
            if dist < hookup_dist:
                close_goal_ids.append(ids)
        if closest_goal_dist < hookup_dist and closest_robot_dist < hookup_dist:
            # see if there is a complete path
            # get the np arrays for each element, and duplicate them to make it symmetric
            start = np.append(graph[1], graph[2])
            end = np.append(graph[2], graph[1])
            length = np.append(graph[3], graph[3])
            size = max(graph[0])+1
            coo_graph = coo_matrix((length, (start, end)), shape=(size,size))
            dist, pred = shortest_path(coo_graph, directed=False, indices=close_goal_ids, return_predecessors=True)
            if max((pred[:,close_robot_ids]).flatten()) > -9999:
                # if there exists some path, break out of the while loop
                break
        # add more to the graph otherwise
        start_id = new_start_id
        graph = build_prm_graph(config_space, room_voxel, divisions, subset_size, graph)

    # create a motion plan from the graph
    # find the minimum distance path
    goal_id_index = np.argmin(np.amin(dist[:,close_robot_ids], axis=1))
    best_goal = close_goal_ids[goal_id_index]
    best_pred = pred[goal_id_index]
    best_start = np.argmin(dist[goal_id_index,close_robot_ids])
    best_start = close_robot_ids[best_start]

    path = [best_start]
    current = best_start
    while current != best_goal:
        path.append(best_pred[current])
        current = best_pred[current]

    #save the time of planning completion
    time_planning_done = rospy.Time.now()
    time_difference = time_planning_done.to_sec()-time_planning_start.to_sec()
    plan = [graph[0][key] for key in path]
    plan.append(np.asarray(goal))
    print("Planning took", time_difference)
    print("Path is", plan)

    #create a visualization of the points and lines
    
    #save the visualizations
    #and dump the stats locally
    # for each list, get the numpy representation
    date = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())

    x = np.array(plan[0:-1])[:,0]
    y = np.array(plan[0:-1])[:,1]
    

    all_points = np.array([graph[0][key] for key in range(0,max(graph[0])+1)])/room_voxel

    if not os.path.isdir("plans/"):
        os.makedirs("plans/")

    #plot path
    plt.figure()
    plt.imshow(free_space)
    for row in zip(graph[1],graph[2]):
        sp = graph[0][row[0]]/room_voxel
        ep = graph[0][row[1]]/room_voxel
        plt.plot([sp[1],ep[1]], [sp[0],ep[0]], 'r:', linewidth=.5)
    plt.plot(all_points[:,1], all_points[:,0], 'b.')
    plt.plot(y/room_voxel, x/room_voxel, 'k')
    plt.plot(goal[1]/room_voxel, goal[0] / room_voxel,'go')
    plt.plot(initial_pos[1]/room_voxel, initial_pos[0]/room_voxel,'co')
    plt.title('Plan generated by algorithm (with all points and connections)\nTime to generate: '+str(time_difference)+'s')
    plt.savefig("plans/"+date+"_prm_plan.png")
    plt.draw()

    run_plan(plan)


if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv=sys.argv)
        if len(argv) < 3:
            print("Please enter a position x y")
        prm_planner(argv[1], argv[2])
    except rospy.ROSInterruptException:
        pass

