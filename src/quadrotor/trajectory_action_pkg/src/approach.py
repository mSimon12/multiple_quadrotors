#!/usr/bin/env python2.7

import copy
from numpy import arange
from math import sqrt, atan2
from threading import Condition

import rospy
import roslib
roslib.load_manifest('trajectory_action_pkg')

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from actionlib import SimpleActionServer, SimpleActionClient, GoalStatus
from trajectory_action_pkg.msg import ExecuteDroneApproachAction, ExecuteDroneApproachFeedback, ExecuteDroneApproachResult 
from hector_uav_msgs.msg import PoseGoal, PoseAction

from geometry_msgs.msg import Pose, Transform
from nav_msgs.msg import Odometry

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_msgs.msg import RobotState, DisplayTrajectory
from moveit_msgs.srv import  GetStateValidityRequest, GetStateValidity

#My library
from moveit import MoveGroup, PlanningScenePublisher


PLANNING_GROUP = "DroneBody"
XMIN = -50.0
XMAX = 50.0
YMIN = -50.0
YMAX = 50.0
ZMIN = 0.0
ZMAX = 50.0
RESOLUTION = 0.05
SENSOR_RANGE = 5.0

class Approach(object):

    def __init__(self, name):
        self.robot_name = name

        # Mutual exclusion odometry
        self.odometry_me = Condition()

        # Create trajectory server
        self.trajectory_server = SimpleActionServer('approach_server', ExecuteDroneApproachAction, self.goCallback, False)
        self.server_feedback = ExecuteDroneApproachFeedback()
        self.server_result = ExecuteDroneApproachResult()
        
        # Get client from hector_quadrotor_actions
        self.move_client = SimpleActionClient("/{}/action/pose".format(name), PoseAction)     
        self.move_client.wait_for_server()

        # Subscribe to ground_truth to monitor the current pose of the robot
        rospy.Subscriber("/{}/ground_truth/state".format(name), Odometry, self.poseCallback)

        # Subscribe to topic to receive the planned trajectory
        rospy.Subscriber("/{}/move_group/display_planned_path".format(name), DisplayTrajectory, self.planCallback)

        #Auxiliary variables
        self.trajectory = []                        # Array with the trajectory to be executed
        self.trajectory_received = False            # Flag to signal trajectory received
        self.odom_received = False                  # Flag to signal odom received

        self.robot = RobotCommander(robot_description="{}/robot_description".format(name), ns="/{}".format(name))
        self.display_trajectory_publisher = rospy.Publisher('/{}/move_group/display_planned_path'.format(name),
                                               DisplayTrajectory,
                                               queue_size=20)

        # Variables for collision callback
        self.validity_srv = rospy.ServiceProxy('/{}/check_state_validity'.format(name), GetStateValidity)
        self.validity_srv.wait_for_service()
        self.collision = False

        # Set planning algorithm
        # self.move = MoveGroupCommander(PLANNING_GROUP, robot_description="{}/robot_description".format(name), ns="/UAV_1")        # Set group from srdf
        # self.move_group.set_planner_id("RRTConnectkConfigDefault")                      # Set planner type  (RRTConnectkConfigDefault)
        # self.move_group.set_num_planning_attempts(10)                                   # Set planning attempts
        # self.move_group.set_workspace([XMIN,YMIN,ZMIN,XMAX,YMAX,ZMAX])                  # Set the workspace size

        #Start move_group
        self.move_group = MoveGroup('earth', name)
        self.move_group.set_planner()

        #Start planningScenePublisher
        self.scene_pub = PlanningScenePublisher(name)

        # Get current robot position to define as start planning point
        self.current_pose = self.robot.get_current_state()

        # Start trajectory server
        self.trajectory_server.start()


    def goCallback(self,pose):
        '''
            Require a plan to go to the desired target and try to execute it 5 time or return erro
        '''
        self.target = pose.goal

        rospy.loginfo("Try to start from [{},{},{}]".format(self.odometry.position.x, self.odometry.position.y, self.odometry.position.z))
        rospy.loginfo("Try to go to [{},{},{}]".format(self.target.position.x, self.target.position.y, self.target.position.z))
        
        trials = 0
        while trials < 5:
            rospy.logwarn("Attempt {}".format(trials+1))
            result = self.go(self.target)
            if (result == 'replan') or (result == 'no_plan'):
                trials += 1
            else:
                trials = 10
            self.collision = False

        if result == 'ok':
            self.trajectory_server.set_succeeded()
        elif (result == 'preempted'):
            self.trajectory_server.set_preempted() 
        else:
            self.trajectory_server.set_aborted() 

        self.trials = 0


    def go(self, target_):
        '''
            Function to plan and execute the trajectory one time
        '''
        # Insert goal position on an array
        target = []
        target.append(target_.position.x)
        target.append(target_.position.y)
        target.append(target_.position.z)
        target.append(target_.orientation.x)
        target.append(target_.orientation.y)
        target.append(target_.orientation.z)
        target.append(target_.orientation.w)

        #Define target for move_group
        # self.move_group.set_joint_value_target(target)
        self.move_group.set_target(target)

        self.odometry_me.acquire()
        self.current_pose.multi_dof_joint_state.transforms[0].translation.x = self.odometry.position.x
        self.current_pose.multi_dof_joint_state.transforms[0].translation.y = self.odometry.position.y
        self.current_pose.multi_dof_joint_state.transforms[0].translation.z = self.odometry.position.z
        self.current_pose.multi_dof_joint_state.transforms[0].rotation.x = self.odometry.orientation.x
        self.current_pose.multi_dof_joint_state.transforms[0].rotation.x = self.odometry.orientation.y
        self.current_pose.multi_dof_joint_state.transforms[0].rotation.x = self.odometry.orientation.z
        self.current_pose.multi_dof_joint_state.transforms[0].rotation.x = self.odometry.orientation.w
        self.odometry_me.release()

        #Set start state
        self.move_group.set_start_state(self.current_pose)

        #Update PlanningSene
        self.scene_pub.publishScene(self.current_pose)

        #Insert start state on move_group
       
        # self.move_group.set_start_state_to_current_state()

        # Plan a trajectory till the desired target
        plan = self.move_group.plan()

        if plan.planned_trajectory.multi_dof_joint_trajectory.points:                      # Execute only if has points on the trajectory
        # if plan.multi_dof_joint_trajectory.points:                                           # Execute only if has points on the trajectory
            while (not self.trajectory_received):
                rospy.loginfo("Waiting for trajectory!")
                rospy.sleep(0.2)

            # rospy.loginfo("TRAJECTORY: {}".format(self.trajectory))

            #Execute trajectory with action_pose
            last_pose = self.trajectory[0]

            for pose in self.trajectory:

                # Verify preempt call
                if self.trajectory_server.is_preempt_requested():
                    self.move_client.send_goal(last_pose)
                    self.trajectory_received = False
                    self.odom_received = False
                    return 'preempted'
                
                #Send next pose to move
                self.next_pose = pose.target_pose.pose

                self.move_client.send_goal(pose,feedback_cb=self.collisionCallback)
                self.move_client.wait_for_result()
                result = self.move_client.get_state()
                
                # Abort if the drone can not reach the position
                if result == GoalStatus.ABORTED:
                    self.move_client.send_goal(last_pose)       #Go back to the last pose
                    self.trajectory_received = False
                    self.odom_received = False
                    return 'aborted'
                elif result == GoalStatus.PREEMPTED:
                    return 'replan'
                last_pose = pose
                self.server_feedback.current_pose = self.odometry
                self.trajectory_server.publish_feedback(self.server_feedback)
            
            # Reset control variables
            self.trajectory_received = False
            self.odom_received = False
            rospy.loginfo("Trajectory is traversed!")
            return 'ok'
        else:
            rospy.logerr("Trajectory is empty. Planning was unsuccessful.")
            return 'no_plan'


    def planCallback(self, msg):
        '''
            Receive planned trajectories and insert it into an array of waypoints
        '''
        if(not self.odom_received):
            return
        
        # Variable to calculate the distance difference between 2 consecutive points
        last_pose = PoseGoal()
        last_pose.target_pose.pose.position.x = self.odometry.position.x
        last_pose.target_pose.pose.position.y = self.odometry.position.y
        last_pose.target_pose.pose.position.z = self.odometry.position.z
        last_pose.target_pose.pose.orientation.x = self.odometry.orientation.x
        last_pose.target_pose.pose.orientation.y = self.odometry.orientation.y
        last_pose.target_pose.pose.orientation.z = self.odometry.orientation.z
        last_pose.target_pose.pose.orientation.w = self.odometry.orientation.w

        self.trajectory = []
        for t in msg.trajectory:
            for point in t.multi_dof_joint_trajectory.points:
                waypoint = PoseGoal()
                waypoint.target_pose.header.frame_id = "{}/world".format(self.robot_name)
                waypoint.target_pose.pose.position.x = point.transforms[0].translation.x
                waypoint.target_pose.pose.position.y = point.transforms[0].translation.y
                waypoint.target_pose.pose.position.z = point.transforms[0].translation.z

                # Orientate the robot always to the motion direction
                delta_x = point.transforms[0].translation.x - last_pose.target_pose.pose.position.x
                delta_y = point.transforms[0].translation.y - last_pose.target_pose.pose.position.y
                motion_theta = atan2(delta_y, delta_x)

                # Make the robot orientation fit with the motion orientation if the movemente on xy is bigger than 0.2 
                if (abs(delta_x) > RESOLUTION) or (abs(delta_y) > RESOLUTION):
                    q = quaternion_from_euler(0,0,motion_theta)
                    waypoint.target_pose.pose.orientation.x = q[0]
                    waypoint.target_pose.pose.orientation.y = q[1]
                    waypoint.target_pose.pose.orientation.z = q[2]
                    waypoint.target_pose.pose.orientation.w = q[3]
                else:
                    waypoint.target_pose.pose.orientation.x = point.transforms[0].rotation.x
                    waypoint.target_pose.pose.orientation.y = point.transforms[0].rotation.y
                    waypoint.target_pose.pose.orientation.z = point.transforms[0].rotation.z
                    waypoint.target_pose.pose.orientation.w = point.transforms[0].rotation.w

                last_pose = copy.copy(waypoint)         # Save pose to calc the naxt delta

                self.trajectory.append(waypoint)

            #Insert a last point to ensure that the robot end at the right position
            waypoint = PoseGoal()
            waypoint.target_pose.header.frame_id = "{}/world".format(self.robot_name)
            waypoint.target_pose.pose.position.x = point.transforms[0].translation.x
            waypoint.target_pose.pose.position.y = point.transforms[0].translation.y
            waypoint.target_pose.pose.position.z = point.transforms[0].translation.z

            waypoint.target_pose.pose.orientation.x = point.transforms[0].rotation.x
            waypoint.target_pose.pose.orientation.y = point.transforms[0].rotation.y
            waypoint.target_pose.pose.orientation.z = point.transforms[0].rotation.z
            waypoint.target_pose.pose.orientation.w = point.transforms[0].rotation.w
            self.trajectory.append(waypoint)

        self.trajectory_received = True


    def poseCallback(self,odometry):
        '''
            Monitor the current position of the robot
        '''
        self.odometry_me.acquire()
        self.odometry = odometry.pose.pose
        # print(self.odometry)
        self.odometry_me.release()
        self.odom_received = True


    def collisionCallback(self,feedback):
        '''
            This callback runs on every feedback message received
        '''
        validity_msg = GetStateValidityRequest()                        # Build message to verify collision
        validity_msg.group_name = PLANNING_GROUP

        if self.next_pose and (not self.collision):
            self.odometry_me.acquire()

            x = self.odometry.position.x
            y = self.odometry.position.y
            z = self.odometry.position.z

            # Distance between the robot and the next position
            dist = sqrt((self.next_pose.position.x - x)**2 +
                        (self.next_pose.position.y - y)**2 +
                        (self.next_pose.position.z - z)**2)

            # Pose to verify collision
            pose = Transform()
            pose.rotation.x = self.odometry.orientation.x
            pose.rotation.y = self.odometry.orientation.y
            pose.rotation.z = self.odometry.orientation.z
            pose.rotation.w = self.odometry.orientation.w
            self.odometry_me.release()

            #Verify possible collisions on diferent points between the robot and the goal point
            for d in arange(RESOLUTION, dist, RESOLUTION):
                pose.translation.x = (self.next_pose.position.x - x)*(d/dist) + x
                pose.translation.y = (self.next_pose.position.y - y)*(d/dist) + y
                pose.translation.z = (self.next_pose.position.z - z)*(d/dist) + z

                self.current_pose.multi_dof_joint_state.transforms[0] = pose                # Insert the correct odometry value
                validity_msg.robot_state = self.current_pose

                #Update PlanningSene
                self.scene_pub.publishScene(self.current_pose)

                # Call service to verify collision
                collision_res = self.validity_srv.call(validity_msg)
                # print("\nCollision response:")
                # print(collision_res)

                # Check if robot is in collision
                if not collision_res.valid:
                    # print(validity_msg)
                    rospy.logwarn('Collision in front [x:{} y:{} z:{}]'.format(pose.translation.x,pose.translation.y,pose.translation.z))
                    # print(collision_res)
                    self.move_client.cancel_goal()
                    self.collision = True
                    return


if __name__=="__main__":
    NAME = rospy.get_param("robot_name", default = "")

    rospy.init_node("trajectory_server", anonymous=False)           # Initialize approach node
    app = Approach(NAME)

    rospy.spin()
