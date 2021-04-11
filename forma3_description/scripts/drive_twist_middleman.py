#!/usr/bin/env python
# todo: license here

# A middleman that subscribes to the cmd_vel topic and publishes to a "hidden" controller/cmd_vel and controller/cmd_vel_raw topic.
# It peturbs the inputs based on variances stored into the parameter server.

import random
import time
import math

import rospy
from geometry_msgs.msg import Twist
from forma3_description.msg import ControlInput
from gazebo_msgs.srv import GetModelState


state_changed = False
fwd = 0.0
left = 0.0
turn = 0.0

def callback_relay(in_msg):
    global state_changed, fwd, left, turn
    
    # the only input movements are going to be forwards/backwards and sideways velocity
    fwd = in_msg.linear.x
    left = in_msg.linear.y
    turn = in_msg.angular.z
    state_changed = True


def drive_twist_middleman():
    global state_changed, fwd, left, turn

    rospy.init_node('drive_twist_middleman', anonymous=True)
    
    # get the rate
    rate = rospy.get_param('controller/drive_rate', 20)
    
    # get the random noise (holonomic, no turning)
    drive_var = rospy.get_param('controller/drive_var', [0.2, 0.2, 0.2])
    
    # setup publishers and subscribers
    pub = rospy.Publisher('controller/cmd_vel', Twist, queue_size=10)
    pub_raw = rospy.Publisher('controller/cmd_vel_raw', ControlInput, queue_size=10)
    #In case the raw data is needed at rate.

    rospy.Subscriber('cmd_vel', Twist, callback_relay)

    # store the parameters back so that the parameter server can be updated with default values if not present
    rospy.set_param('controller/drive_rate', rate)
    rospy.set_param('controller/drive_var', drive_var)

    # convert variance to stddev ro reduce complexity in loop
    forward_var = math.sqrt(drive_var[0])
    left_var    = math.sqrt(drive_var[1])
    turn_var    = math.sqrt(drive_var[2])

    # start running at rate
    timer = rospy.Rate(rate)
    msg = Twist()
    msg_raw = ControlInput()
    while not rospy.is_shutdown():
        # update control state if changed
        if state_changed:
            msg_raw.forward = fwd
            msg_raw.left = left
            msg_raw.turn = turn
            state_changed = False

        #time
        msg_raw.header.stamp = rospy.Time.now()

        # generate noise based on input
        fwd_noise = random.gauss(0.0, forward_var) * msg_raw.forward
        left_noise = random.gauss(0.0, left_var) * msg_raw.left
        turn_noise = random.gauss(0.0, turn_var) * msg_raw.turn
        
        # update and publish the message with noise
        msg.linear.x = msg_raw.forward + fwd_noise
        msg.linear.y = msg_raw.left + left_noise
        msg.angular.z = msg_raw.turn + turn_noise
        
        pub.publish(msg)
        pub_raw.publish(msg_raw)
        
        # delay
        timer.sleep()


if __name__ == '__main__':
    try:
        drive_twist_middleman()
    except rospy.ROSInterruptException:
        pass
