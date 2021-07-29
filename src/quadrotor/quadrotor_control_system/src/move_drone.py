#!/usr/bin/env python2.7

from threading import Thread

import sys
import rospy
import roslib
roslib.load_manifest('quadrotor_control_system')

from actionlib import SimpleActionClient
from tf.transformations import quaternion_from_euler
from trajectory_action_pkg.msg import ExecuteDroneApproachAction, ExecuteDroneApproachGoal

if __name__=="__main__":

    rospy.init_node("move_drone", anonymous=True)
    
    dest = ExecuteDroneApproachGoal()
    quad_name = sys.argv[1]                               # quadrotor name (UAV_1, UAV_2, ...)
    dest.goal.position.x = float(sys.argv[2])              # quadrotor X position
    dest.goal.position.y = float(sys.argv[3])              # quadrotor Y position
    dest.goal.position.z = float(sys.argv[4])              # quadrotor Z position

    # Translate theta to quartenion
    theta = float(sys.argv[5])                             # quadrotor orientation
    q = quaternion_from_euler(0,0,theta,'ryxz')
    dest.goal.orientation.x = q[0]
    dest.goal.orientation.y = q[1]
    dest.goal.orientation.z = q[2]
    dest.goal.orientation.w = q[3]

    trajectory_client = SimpleActionClient("{}/approach_server".format(quad_name), ExecuteDroneApproachAction)
    trajectory_client.wait_for_server()

    trajectory_client.send_goal(dest)                      # Send the goal
    trajectory_client.wait_for_result()                    # Wait for the result
    state = trajectory_client.get_state()                  # Get the state of the action

    rospy.loginfo("Motion result: {}".format(state))