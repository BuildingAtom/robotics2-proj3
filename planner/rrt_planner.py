#!/usr/bin/env python

# rapid random trees
#todo: cleanup these imports

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

from skimage.io import imread
from skimage.draw import line_aa
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import shortest_path
import numpy as np

import roslaunch


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
def build_rrt_graph(config_map, voxel_size, start_pos, end_pos, max_length):
    # following the same graph format, we make two, graph_start, graph_end
    # I imagine sampling with something like an expanding wald distribution may
    # be better, but for time, sample uniformly across the space and build the tree
    rng = np.random.default_rng()

    graph_start = []
    graph_start.append([np.asarray(start_pos)])
    graph_start.append([])
    graph_start.append([])
    graph_start.append([])
    graph_end = []
    graph_end.append([np.asarray(end_pos)])
    graph_end.append([])
    graph_end.append([])
    graph_end.append([])

    graph_final = [[],[],[],[]]
    end_node = 0

    while True:
        #sample 20 points randomly
        internal_points = rng.uniform(0,10.0,(20, 2))
        #remove any invalids
        mask = np.transpose((internal_points / voxel_size).astype(int))
        mask = (config_map[tuple(mask)]).astype(bool)
        sample = internal_points[mask]

        #test each one (only test against the last 20 points added to the graph)
        offset_start = 0
        offset_end = 0
#        if len(graph_start[0]) > 20:
#            graph_start_points = np.array(graph_start[0][-20:])
#            offset_start = len(graph_start[0])-20
#        else:
#            graph_start_points = np.array(graph_start[0])
#
#        if len(graph_end[0]) > 20:
#            graph_end_points = np.array(graph_end[0][-20:])
#            offset_end = len(graph_end[0])-20
#        else:
#            graph_end_points = np.array(graph_end[0])
        graph_start_points = np.array(graph_start[0])
        graph_end_points = np.array(graph_end[0])

        delta_start = np.linalg.norm(np.expand_dims(sample, axis=1) - np.array(graph_start_points), axis=2)
        delta_end = np.linalg.norm(np.expand_dims(sample, axis=1) - np.array(graph_end_points), axis=2)
        # column indexes the tree point, row indexes the sample point

        #threshold and identify any overlap
        delta_start[delta_start>max_length] = -1
        delta_end[delta_end>max_length] = -1
        

        start_mask = np.max(delta_start, axis=1)
        start_mask[start_mask<=0] = 0
        start_mask = start_mask.astype(bool)

        end_mask = np.max(delta_end, axis=1)
        end_mask[end_mask<=0] = 0
        end_mask = end_mask.astype(bool)

        delta_start[delta_start<0] = float('inf')
        delta_end[delta_end<0] = float('inf')

        overlap_mask = np.copy(start_mask)
        overlap_mask[~end_mask] = 0
        if np.any(overlap_mask):
            # we have overlap, so we fill it in and break
            ov_s = sample[overlap_mask]
            # update the start tree
            ov_l_start = delta_start[overlap_mask]
            ov_i_start = np.argmin(ov_l_start, axis=1)
            offset = len(graph_start[0])
            added_start = []
            added_end = []
            graph_final[0] = graph_start[0]
            graph_final[1] = graph_start[1]
            graph_final[2] = graph_start[2]
            graph_final[3] = graph_start[3]
            ids = []
            for index in range(len(ov_i_start)):
                #check and skip if there's a barrier
                if does_path_intersect(config_map, graph_final[0][ov_i_start[index] + offset_start]/voxel_size, sample[index]/voxel_size):
                    ids.append(-1)
                    offset = offset - 1
                    continue
                graph_final[0].append(sample[index])
                graph_final[1].append(ov_i_start[index] + offset_start)
                graph_final[2].append(index + offset)
                graph_final[3].append(ov_l_start[index,ov_i_start[index]])
                ids.append(index + offset)
                # note the id to make sure we don't fail by accident
                added_start.append(index)

            # grab new offset
            new_offset_end = offset_end + len(graph_final[0])-1
            end_node = len(graph_final[0])
            # append the end tree
            graph_final[0].extend(graph_end[0])
            graph_final[1].extend([x + end_node for x in graph_end[1]])
            graph_final[2].extend([x + end_node for x in graph_end[2]])
            graph_final[3].extend(graph_end[3])
            offset = len(graph_final[0])

            # update the tree with the final connections
            ov_l_end = delta_end[overlap_mask]
            ov_i_end = np.argmin(ov_l_end, axis=1)
            for index in range(len(ov_i_end)):
                #check and skip if there's a barrier
                if does_path_intersect(config_map, graph_final[0][ov_i_end[index] + new_offset_end]/voxel_size, sample[index]/voxel_size):
                    offset = offset - 1
                    continue
                # don't need to readd the same nodes
                graph_final[1].append(ov_i_end[index] + new_offset_end)
                if (ids[index] != -1):
                    graph_final[2].append(ids[index])
                    offset = offset - 1
                else:
                    graph_final[0].append(sample[index])
                    graph_final[2].append(index + offset)
                graph_final[3].append(ov_l_end[index,ov_i_end[index]])
                # note the id to make sure we don't fail by accident
                added_end.append(index)
            # as long as at least one node is in common...
            if set(added_start) & set(added_end):
                # stop building the tree here if we have overlap
                break

        #build the start tree points
        if np.any(start_mask):
            # we have overlap, so we fill it in and break
            ov_s = sample[start_mask]
            # update the start tree
            ov_l_start = delta_start[start_mask]
            ov_i_start = np.argmin(ov_l_start, axis=1)
            offset = len(graph_start[0])
            for index in range(len(ov_i_start)):
                #check and skip if there's a barrier
                if does_path_intersect(config_map, graph_start[0][ov_i_start[index] + offset_start]/voxel_size, sample[index]/voxel_size):
                    offset = offset - 1
                    continue
                graph_start[0].append(sample[index])
                graph_start[1].append(ov_i_start[index] + offset_start)
                graph_start[2].append(index + offset)
                graph_start[3].append(ov_l_start[index,ov_i_start[index]])

        #build the end tree points
        if np.any(end_mask):
            # we have overlap, so we fill it in and break
            ov_s = sample[end_mask]
            # update the start tree
            ov_l_end = delta_end[end_mask]
            ov_i_end = np.argmin(ov_l_end, axis=1)
            offset = len(graph_end[0])
            for index in range(len(ov_i_end)):
                #check and skip if there's a barrier
                if does_path_intersect(config_map, graph_end[0][ov_i_end[index] + offset_end]/voxel_size, sample[index]/voxel_size):
                    offset = offset - 1
                    continue
                graph_end[0].append(sample[index])
                graph_end[1].append(ov_i_end[index] + offset_end)
                graph_end[2].append(index + offset)
                graph_end[3].append(ov_l_end[index,ov_i_end[index]])

    # the graph is sparsely defined to later use scipy coo matrix and shortest path
    # also nice because it can handle duplicates
    graph_start[0] = {index: value for index, value in enumerate(graph_start[0])}
    print(graph_start)
    print(end_node)
    return (graph_start, 0, end_node)


def run_plan(plan):
    #use bug2 to execute the plan
    package = rospy.get_param('local_motion_package')
    exe = rospy.get_param('local_motion_exe')
    extra_args = rospy.get_param('local_motion_extra_args')
    evaluator = rospy.get_param('local_motion_eval')
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    nodes = []
    for next in plan:
        nodes.append(roslaunch.core.Node(package, exe, args=str(next[0])+' '+str(next[1])+' '+extra_args))

    #launch the evaluation node!
    eval_node = launch.launch(roslaunch.core.Node(package, evaluator, cwd="node"))
    for node in nodes:
        process = launch.launch(node)
        while process.is_alive():
            time.sleep(0.02)
    eval_node.stop()
#        cli_args = [launch_file, 'x:='+str(next[0]), 'y:='+str(next[1]), 'stopdone:=false']
#        roslaunch_args = cli_args[1:]
#        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
#        roslaunch_file = [(roslaunch_file[0], roslaunch_args)]
#
#        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
#
#        parent.start(auto_terminate=False)
#        parent.spin()


# run the main planner routine
def rrt_planner(targetx, targety):

    rospy.init_node('rrt_planner', anonymous=True)

    # load in the map
    map_path = rospy.get_param('/map/free_space_map', "./room.pgm")
    free_space = imread(map_path)
    map_path = rospy.get_param('/map/configuration_map', "./room.pgm")
    config_space = imread(map_path)
    room_voxel = rospy.get_param('/map/voxel_size', 0.05)
    # rotate it to match the room (room_map[x][y])
    free_space = np.rot90(free_space, k=-1, axes=(0,1))
    config_space = np.rot90(config_space, k=-1, axes=(0,1))

    # get the only parameter for rrt
    max_length = rospy.get_param('rrt_max_length', 1.0)

    # get the state & goal
    # we're running off the assumption that everything is already running
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    robot = get_model_state('maru2','')
    initial_pos = np.array([robot.pose.position.x, robot.pose.position.y], dtype=float)
    goal = np.array([targetx, targety], dtype=float)

    # start timing
    time_planning_start = rospy.Time.now()
    graph, start_node, end_node = build_rrt_graph(config_space, room_voxel, initial_pos, goal, max_length)

    # Find shortest path
    start = np.append(graph[1], graph[2])
    end = np.append(graph[2], graph[1])
    length = np.append(graph[3], graph[3])
    size = max(graph[0])+1
    coo_graph = coo_matrix((length, (start, end)), shape=(size,size))
    dist, pred = shortest_path(coo_graph, directed=False, indices=end_node, return_predecessors=True)

    print(dist, pred)

    path = [pred[start_node]]
    current = pred[start_node]
    while current != end_node:
        path.append(pred[current])
        current = pred[current]

    #save the time of planning completion
    time_planning_done = rospy.Time.now()
    time_difference = time_planning_done.to_sec()-time_planning_start.to_sec()
    plan = [graph[0][key] for key in path]
    print("Planning took", time_difference)
    print("Path is", plan)

    #create a visualization of the points and lines
    
    #save the visualizations
    #and dump the stats locally
    # for each list, get the numpy representation
    date = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())

    x = np.array(plan)[:,0]
    y = np.array(plan)[:,1]
    

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
    plt.savefig("plans/"+date+"_rrt_plan.png")
    plt.draw()

    run_plan(plan)


if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv=sys.argv)
        if len(argv) < 3:
            print("Please enter a position x y")
        rrt_planner(argv[1], argv[2])
    except rospy.ROSInterruptException:
        pass

