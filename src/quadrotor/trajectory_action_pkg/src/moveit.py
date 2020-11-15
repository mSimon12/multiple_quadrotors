#!/usr/bin/env python2.7

import copy
import rospy

from actionlib import SimpleActionClient, GoalStatus
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MotionPlanRequest, Constraints, JointConstraint 
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, AllowedCollisionEntry
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest

from geometry_msgs.msg import Pose
from octomap_msgs.msg import Octomap


class MoveGroup(object):

    def __init__(self, frame, ns = ''):
        # self.scene_pub = PlanningScenePublisher(ns)

        self.move_group_client = SimpleActionClient("/{}/move_group".format(ns), MoveGroupAction)     
        self.move_group_client.wait_for_server()
        
        self.move_group_msg = MoveGroupGoal()

        #Create the message for the request
        self.move_group_msg.request = MotionPlanRequest()

        self.move_group_msg.request.workspace_parameters.header.frame_id = frame
        self.move_group_msg.request.workspace_parameters.min_corner.x = -50
        self.move_group_msg.request.workspace_parameters.min_corner.y = -50
        self.move_group_msg.request.workspace_parameters.min_corner.z = 0
        self.move_group_msg.request.workspace_parameters.max_corner.x = 50
        self.move_group_msg.request.workspace_parameters.max_corner.y = 50
        self.move_group_msg.request.workspace_parameters.max_corner.z = 50

        # Target definition      
        self.move_group_msg.request.goal_constraints.append(Constraints())

        self.constraints = JointConstraint()
        self.constraints.tolerance_above = 0.001
        self.constraints.tolerance_below = 0.001
        self.constraints.weight = 1.0

        for i in range(0,7):
            self.move_group_msg.request.goal_constraints[0].joint_constraints.append(copy.deepcopy(self.constraints))

        self.move_group_msg.planning_options.planning_scene_diff.is_diff = True
        self.move_group_msg.planning_options.plan_only = True

    def set_planner(self, planner_id = 'RRTConnectkConfigDefault', group = 'DroneBody', attempts = 10, allowed_time = 5):
        self.move_group_msg.request.planner_id = planner_id
        self.move_group_msg.request.group_name = group
        self.move_group_msg.request.num_planning_attempts = attempts
        self.move_group_msg.request.allowed_planning_time = allowed_time
        self.move_group_msg.request.max_velocity_scaling_factor = 1.0
        self.move_group_msg.request.max_acceleration_scaling_factor = 1.0  

    def set_start_state(self, robot_state):
        self.move_group_msg.request.start_state = robot_state
        
        # self.scene_pub.publishScene(robot_state)

    def set_target(self,target):
        self.move_group_msg.request.goal_constraints[0].joint_constraints[0].joint_name = "virtual_joint/trans_x"
        self.move_group_msg.request.goal_constraints[0].joint_constraints[0].position = target[0]

        self.move_group_msg.request.goal_constraints[0].joint_constraints[1].joint_name = "virtual_joint/trans_y"
        self.move_group_msg.request.goal_constraints[0].joint_constraints[1].position = target[1]

        self.move_group_msg.request.goal_constraints[0].joint_constraints[2].joint_name = "virtual_joint/trans_z"
        self.move_group_msg.request.goal_constraints[0].joint_constraints[2].position = target[2]

        self.move_group_msg.request.goal_constraints[0].joint_constraints[3].joint_name = "virtual_joint/rot_x"
        self.move_group_msg.request.goal_constraints[0].joint_constraints[3].position = target[3]

        self.move_group_msg.request.goal_constraints[0].joint_constraints[4].joint_name = "virtual_joint/rot_y"
        self.move_group_msg.request.goal_constraints[0].joint_constraints[4].position = target[4]

        self.move_group_msg.request.goal_constraints[0].joint_constraints[5].joint_name = "virtual_joint/rot_z"
        self.move_group_msg.request.goal_constraints[0].joint_constraints[5].position = target[5]

        self.move_group_msg.request.goal_constraints[0].joint_constraints[6].joint_name = "virtual_joint/rot_w"
        self.move_group_msg.request.goal_constraints[0].joint_constraints[6].position = target[6]
    
    def plan(self):

        # print("\nMOVE_GROUP_MSG:")
        # print(self.move_group_msg)

        self.move_group_client.send_goal(self.move_group_msg)
        self.move_group_client.wait_for_result()
        result = self.move_group_client.get_result()
        print(result)

        return result

    
class PlanningScenePublisher(object):

    def __init__(self, name):
        self.drone_name = name

        self.scene_publisher = rospy.Publisher('/{}/planning_scene'.format(name), PlanningScene, queue_size=10)
        self.scene_msg = PlanningScene()
        self.scene_msg.name = "Drone_scene"

        self.scene_msg.world.octomap.header.frame_id = '{}/map'.format(name)
        self.scene_msg.world.octomap.origin = Pose()

        self.scene_msg.allowed_collision_matrix.entry_names = ["base_link", "camera_link", "sonar_link"]

        self.scene_msg.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry())
        self.scene_msg.allowed_collision_matrix.entry_values[0].enabled = [False,False,True]

        self.scene_msg.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry())
        self.scene_msg.allowed_collision_matrix.entry_values[1].enabled = [False,False,False]

        self.scene_msg.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry())
        self.scene_msg.allowed_collision_matrix.entry_values[2].enabled = [True,False,False]


    def publishScene(self,robot_state):
        self.octomap = None
        self.octo_sub = rospy.Subscriber("/{}/octomap_binary".format(self.drone_name),Octomap,self.octomap_callback, queue_size=10)

        while(not self.octomap):
            pass
        self.scene_msg.robot_state = robot_state
        self.scene_msg.world.octomap.octomap = self.octomap

        # print("\nPLANNING_SCENE_MSG:")
        # print(self.scene_msg)
        self.scene_publisher.publish(self.scene_msg)

        self.octo_sub.unregister()


    def octomap_callback(self, msg):
        self.octomap = msg
        self.octomap.id="OcTree"
