#!/usr/bin/env python
# todo: license here

import random
import time
import sys
import rospy

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from tf.transformations import quaternion_from_euler

from skimage.io import imread
import numpy as np

# mini helper function to move the robot to a provided starting position.

def move_position(spawnx, spawny, spawntheta):

    # get the random noise and location
    xyY_mean = [spawnx, spawny, spawntheta]
    xyY_stddev = rospy.get_param('spawn_stddev_gauss', [0.1, 0.1, 0.01])
    robot = rospy.get_param('robot_name')
    bounds = rospy.get_param('/map/spawn_bounds', [0.2, 0.2, 9.8, 9.8])

    # load in the map
    map_path = rospy.get_param('/map/configuration_map', "./room.pgm")
    room_voxel = rospy.get_param('/map/voxel_size', 0.05)
    room_map = imread(map_path)
    # rotate it to match the room (room_map[x][y])
    room_map = np.rot90(room_map, k=-1, axes=(0,1))

    while True:
        # generate values
        xyY = []
        for mean, stddev in zip(xyY_mean, xyY_stddev):
            xyY.append(random.gauss(mean, stddev))

        # don't let it get out of bounds otherwise robot spawns in a broken position
        if xyY[0] < bounds[0]: #xmin
            xyY[0] = bounds[0]
        if xyY[1] < bounds[1]: #ymin
            xyY[1] = bounds[1]
        if xyY[0] > bounds[2]: #xmax
            xyY[0] = bounds[2]
        if xyY[1] > bounds[3]: #ymax
            xyY[1] = bounds[3]

        # check the configuration space
        if room_map[int(xyY[0] / room_voxel)][int(xyY[1] / room_voxel)]:
            break

    print("-x " + str(xyY[0]) + " -y " + str(xyY[1]) + " -Y " + str(xyY[2]))

    q = quaternion_from_euler(0, 0, xyY[2])

    # generate the modelstate information
    state = ModelState()
    state.model_name = robot
    state.pose.position.x = xyY[0]
    state.pose.position.y = xyY[1]
    state.pose.position.z = 0
    state.pose.orientation.x = q[0]
    state.pose.orientation.y = q[1]
    state.pose.orientation.z = q[2]
    state.pose.orientation.w = q[3]

    # now try to add it to the robot
    print("waiting for gazeobo")
    rospy.wait_for_service('/gazebo/set_model_state')
    success = False
    while not success: # and keep trying
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_model_state(state)
            success = True
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        except rospy.ROSInterruptException:
            # stop this loop if we catch a ROS Interrupt (launch script shutting down)
            return
        time.sleep(0.1)


if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv=sys.argv)
        if len(argv) < 4:
            print("Please enter a position x y Y")
        else:
            move_position(float(argv[1]), float(argv[2]), float(argv[3]))
    except rospy.ROSInterruptException:
        pass
