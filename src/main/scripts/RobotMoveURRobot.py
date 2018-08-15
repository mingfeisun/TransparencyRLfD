#!/usr/bin/env python
import rospy
import actionlib

from main.srv import *

from CupPoseControl import CupPoseControl
from QLearningModel import QLearningModel

from autonomous_learning import action2Position
from autonomous_learning import action2Goal

from main.msg import CupMoveAction

import sys
import copy
import random
import numpy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from math import pi, cos, sin
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from control_msgs.msg import *
from trajectory_msgs.msg import *

from main.msg import CupMoveAction, CupMoveGoal, CupMoveResult, CupMoveFeedback

from spawn_table_models import getXYZFromIJ

class RobotMoveURRobot:
    def __init__(self):
        self.cup_pos_ctrl = CupPoseControl()

        self.client = actionlib.SimpleActionClient('teleop_cup_server', CupMoveAction)
        self.client.wait_for_server()

        rospy.wait_for_service('update_learning')
        self.update_learning = rospy.ServiceProxy('update_learning', LearningDemo)

        rospy.wait_for_service('query_action')
        self.query_action = rospy.ServiceProxy('query_action', QueryAction)

        rospy.wait_for_service('query_action_confidence')
        self.query_action_confidence = rospy.ServiceProxy('query_action_confidence', QueryActionConfidence)

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_end = moveit_commander.MoveGroupCommander("endeffector")
        self.group_man = moveit_commander.MoveGroupCommander("manipulator")

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        self.current_pose = self.group_man.get_current_pose().pose


        self.joint_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.joint_client.wait_for_server()

        self.orien_x = 0.00709209889026
        self.orien_y = 0.731524912106
        self.orien_z = 0.0131916335966
        self.orien_w = 0.681650193211

        self.robot_pose = Pose() 
        self.robot_pose.position.x = -0.75 
        self.robot_pose.position.y = 0.0
        self.robot_pose.position.z = 0.8

        self.addCollision()

        self.grid_size = rospy.get_param('table_params/grid_size')
        self.table_size = rospy.get_param('table_params/table_size')
        self.margin_size = rospy.get_param('table_params/margin_size')

    def initDemo(self):
        self.initCup()
        self.moveArmToCupTop()

    def initCup(self):
        # set to init position
        self.cup_pos_ctrl.setPoseDefault()

    def initRobotPose(self):
        JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # [144.593816163399, 5.754934304529601, 7.194142435155028, 10.61821127013265, 4.675844406769917, 7.934736338099062]
        Q1 = [0.009653124896662924, -0.6835756311532828, 1.0619281313412259, -0.3737989105267019, 0.009994925707914604, 0]
        Q2 = [0.009653124896662924, -0.6835756311532828, 1.170799852990027, -1.9876127002995183, -1.541749171284383, 0]
        # Q3 = [0.009653124896662924, -0.6835756311532828, 1.170799852990027, -1.9876127002995183, 4.681749171284383, 1.8825401280344316]
        # Q2 = [1.5,0,-1.57,0,0,0]
        # Q3 = [1.5,-0.2,-1.57,0,0,0]

        g = FollowJointTrajectoryGoal()

        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES

        g.trajectory.points = [
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(3.0)), 
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(6.0))
            # JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(6.0))
        ]

        self.joint_client.send_goal(g)
        try:
            self.joint_client.wait_for_result()
        except KeyboardInterrupt:
            self.joint_client.cancel_goal()
    
    def addCollision(self):
        collision_bottom_pose = geometry_msgs.msg.PoseStamped()
        collision_bottom_pose.header.frame_id = "world"
        collision_bottom_pose.pose.orientation.w = 1.0
        collision_bottom_pose.pose.position.x = 0.4
        collision_bottom_pose.pose.position.z = -0.1
        collision_bottom_name = "collision_bottom"
        self.scene.add_box(collision_bottom_name, collision_bottom_pose, size=(2, 2, 0.2))

        collision_back_pose = geometry_msgs.msg.PoseStamped()
        collision_back_pose.header.frame_id = "world"
        collision_back_pose.pose.orientation.w = 1.0
        collision_back_pose.pose.position.x = -0.5
        collision_back_pose.pose.position.z = 0.4
        collision_back_name = "collision_back"
        self.scene.add_box(collision_back_name, collision_back_pose, size=(0.2, 2, 1))

        collision_top_pose = geometry_msgs.msg.PoseStamped()
        collision_top_pose.header.frame_id = "world"
        collision_top_pose.pose.orientation.w = 1.0
        collision_top_pose.pose.position.x = 0.4
        collision_top_pose.pose.position.z = 1.0
        collision_top_name = "collision_top"
        self.scene.add_box(collision_top_name, collision_top_pose, size=(2, 2, 0.2))

    def moveArmToCupTop(self):
        cup_pose = self.cup_pos_ctrl.getPose()
        target_x, target_y, target_z = self.cupPoseToRobotPose(cup_pose)
        self.moveArmTo(target_x, target_y, target_z)

    def moveArmToPose(self, _pose_goal):
        self.group_man.set_pose_target(_pose_goal)
        self.group_man.go(wait=True)
        self.group_man.stop()
        self.group_man.clear_pose_targets()
        self.current_pose = self.group_man.get_current_pose().pose

    def moveArmTo(self, _x, _y, _z):
        pose_goal = self.generateRobotPose(_x, _y, _z)
        self.group_man.clear_pose_targets()
        self.group_man.set_pose_target(pose_goal)
        self.group_man.go(wait=True)
        self.group_man.stop()
        self.group_man.clear_pose_targets()
        self.current_pose = self.group_man.get_current_pose().pose

    def test_moveArmTo(self, _pose_goal):
        pose_goal = _pose_goal
        pose_goal.position.x += 0.45
        # pose_goal.position.y += 0.45
        pose_goal.position.z = 1.05
        self.group_man.set_pose_target(pose_goal)
        self.group_man.go(wait=True)
        # self.group_right_arm.stop()
        # self.group_right_arm.clear_pose_targets()

    # for testing
    def test_moveArmRandom(self):
        pose_goal = self.group_man.get_current_pose().pose
        print pose_goal

        x_diff = 0.1
        y_diff = 0.1
        if pose_goal.position.x > 0.9:
            x_diff = -0.1
        if pose_goal.position.x < 0.1:
            x_diff = 0.1
        if pose_goal.position.y > 0.5:
            y_diff = -0.1
        if pose_goal.position.y < -0.5:
            x_diff = 0.1
        pose_goal.position.x += x_diff
        pose_goal.position.y += y_diff
        pose_goal.position.z = 0.2
        pose_goal.orientation.x = self.orien_x
        pose_goal.orientation.y = self.orien_y
        pose_goal.orientation.z = self.orien_z
        pose_goal.orientation.w = self.orien_w

        self.group_man.set_pose_target(pose_goal)
        self.group_man.go(wait=True)
        self.group_man.stop()
        self.group_man.clear_pose_targets()

    def cb_action_request(self, _feedback):
        cup_pose = self.cup_pos_ctrl.getPose()
        target_x, target_y, target_z = self.cupPoseToRobotPose(cup_pose)
        self.moveArmTo(target_x, target_y, target_z)

    def cb_action_request_dummy(self, _feedback):
        pass

    def moveCup(self):
        # set cup pose to init position
        self.cup_pos_ctrl.setPoseDefault()
        # self.initRobotPose()
        self.moveArmToCupTop()

        # 0: left, 1: up, 2: right, 3: down
        if rospy.has_param('table_params'):
            beg_pos = rospy.get_param('table_params/cup_pos_init')
            dst_pos = rospy.get_param('table_params/mat_pos')
        else:
            rospy.loginfo('Table not configured yet')
            sys.exit(1)

        curr_state = str((beg_pos[0], beg_pos[1]))
        goal_state = str((dst_pos[0], dst_pos[1]))

        max_action_num = 500

        action_num = 0

        while curr_state != goal_state:
            curr_action = self.query_action(curr_state).action
            curr_goal = action2Goal(curr_action)

            # self.client.send_goal(curr_goal, feedback_cb=self.cb_action_request)
            self.client.send_goal(curr_goal, feedback_cb=self.cb_action_request_dummy)
            self.client.wait_for_result()

            action_num = action_num + 1

            result = self.client.get_result()
            reward = result.reward

            next_state = str((result.state_x, result.state_y))

            self.update_learning(curr_state, curr_action, reward, next_state)
            curr_state = next_state

            if action_num > max_action_num:
                break

        rospy.loginfo("Robot's turn over")
        rospy.loginfo("Starting human's turn")

        self.cup_pos_ctrl.setPoseDefault()
        self.moveArmToCupTop()

    def cupPoseToRobotPose(self, _cup_pose):
        delta_x = _cup_pose.position.x - self.robot_pose.position.x
        delta_y = _cup_pose.position.y - self.robot_pose.position.y
        delta_z = _cup_pose.position.z - self.robot_pose.position.z + 0.10

        return delta_x, delta_y, delta_z

    def xyzToRobotPose(self, _x, _y, _z):
        delta_x = _x - self.robot_pose.position.x
        delta_y = _y - self.robot_pose.position.y
        delta_z = _z - self.robot_pose.position.z + 0.10

        return delta_x, delta_y, delta_z
    
    def moveArmToState(self, _state):
        curr_state_tuple = eval(_state)

        x, y, z = getXYZFromIJ(curr_state_tuple[0], 
                                curr_state_tuple[1], 
                                self.grid_size, 
                                self.table_size, 
                                self.margin_size)

        x_, y_, z_ = self.xyzToRobotPose(x, y, z)
        self.moveArmTo(x_, y_, z_)

    def generateRobotPose(self, _x, _y, _z):
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.orientation.x = self.orien_x
        pose_goal.orientation.y = self.orien_y
        pose_goal.orientation.z = self.orien_z
        pose_goal.orientation.w = self.orien_w

        pose_goal.position.x = _x
        pose_goal.position.y = _y
        pose_goal.position.z = _z

        return pose_goal

    def stateToRobotPose(self, _state):
        curr_state_tuple = eval(_state)

        x, y, z = getXYZFromIJ(curr_state_tuple[0], 
                                curr_state_tuple[1], 
                                self.grid_size, 
                                self.table_size, 
                                self.margin_size)

        x_, y_, z_ = self.xyzToRobotPose(x, y, z)
        return self.generateRobotPose(x_, y_, z_)

    def gesturing(self, _state):
        # 0: left, 1: up, 2: right, 3: down
        state_stack = []
        waypoints = []

        # self.pausing(_state, 0.1)

        # waypoints.append(copy.deepcopy(self.current_pose))

        dst_pos = rospy.get_param('table_params/mat_pos')

        curr_state = _state
        goal_state = str((dst_pos[0], dst_pos[1]))

        confidence_level = 1.2

        result = self.query_action_confidence(curr_state)

        # self.moveArmToState(curr_state)
        state_stack.append(curr_state)
        # waypoints.append(copy.deepcopy(self.stateToRobotPose(curr_state)))

        while result.confidence <= confidence_level and curr_state != goal_state:
            this_state = result.next_state
            # self.moveArmToState(curr_state)
            state_stack.append(this_state)
            waypoints.append(copy.deepcopy(self.stateToRobotPose(this_state)))
            result = self.query_action_confidence(this_state)

        # (plan, _) = self.group_man.compute_cartesian_path(waypoints, 0.01, 0.0)
        # self.group_man.execute(plan, wait=True)

        # waypoints = []
        # self.group_man.stop()
        # self.group_man.clear_pose_targets()

        state_stack.pop()
        
        while len(state_stack) != 0:
            this_state = state_stack.pop()
            waypoints.append(copy.deepcopy(self.stateToRobotPose(this_state)))
            # self.moveArmToState(this_state)

        (plan, _) = self.group_man.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.group_man.execute(plan, wait=True)
        # self.group_man.go(wait=True)
        # self.group_man.retime_trajectory(self.group_man.get_current_pose(), plan, 1.0)

    def generateCircle(self, _center, _radius=0.02):
        center_x = _center.position.x
        center_y = _center.position.y
        center_z = _center.position.z

        circle_pose = []
        for theta in range(0, 360, 5):
            # sin, cos takes rad as input
            delta_x = _radius * cos(theta*pi/180)
            delta_y = _radius * sin(theta*pi/180)

            circle_pose.append(
                copy.deepcopy(self.generateRobotPose(center_x + delta_x, center_y + delta_y, center_z))
                )
        return circle_pose

    def pausing(self, _state, _confidence):
        waypoints = []
        circle_center = self.stateToRobotPose(_state)
        waypoints.append(copy.deepcopy(circle_center))

        waypoints.extend(self.generateCircle(circle_center))

        waypoints.append(copy.deepcopy(circle_center))

        (plan, _) = self.group_man.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.group_man.execute(plan, wait=True)

    def mixedMotion(self, _state):
        # 0: left, 1: up, 2: right, 3: down
        state_stack = []
        waypoints = []
        confidence_array = []

        dst_pos = rospy.get_param('table_params/mat_pos')

        curr_state = _state
        goal_state = str((dst_pos[0], dst_pos[1]))

        confidence_level = 1.2

        result = self.query_action_confidence(curr_state)
        confidence_array.append(result.confidence)

        state_stack.append(curr_state)

        while result.confidence <= confidence_level and curr_state != goal_state:
            this_state = result.next_state
            state_stack.append(this_state)
            # waypoints.append(copy.deepcopy(self.stateToRobotPose(this_state)))
            result = self.query_action_confidence(this_state)
            confidence_array.append(result.confidence)

        idx_max = numpy.argmax(state_stack)
        for i in range(len(state_stack)):
            each = state_stack[i]
            this_robot_pose = self.stateToRobotPose(each)
            waypoints.append(copy.deepcopy(this_robot_pose))
            if i == idx_max and len(state_stack) != 1:
                waypoints.extend(copy.deepcopy(self.generateCircle(this_robot_pose)))
            result = self.query_action_confidence(each)

        state_stack.pop()
        
        while len(state_stack) != 0:
            this_state = state_stack.pop()
            waypoints.append(copy.deepcopy(self.stateToRobotPose(this_state)))

        (plan, _) = self.group_man.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.group_man.execute(plan, wait=True)


if __name__ == "__main__":
    # for testing
    rospy.init_node('robot_move_ur', anonymous=True)
    test = RobotMoveURRobot()
    test.initRobotPose()
    test.moveArmToCupTop()
    test.gesturing('(0, 0)')
    # freq = rospy.Rate(1)
    # while not rospy.is_shutdown():
    #     test.test_moveArmRandom()
    #     freq.sleep()