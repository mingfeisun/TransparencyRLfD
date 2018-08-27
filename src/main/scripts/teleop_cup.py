#!/usr/bin/env python
import sys
import rospy
import rosbag
import curses
import math
import actionlib
from main.msg import CupMoveAction, CupMoveGoal, CupMoveActionResult, CupMoveActionFeedback

from terminal_input import TextWindow

from std_msgs.msg import Int16
from gazebo_msgs.msg import ModelState

from CupPoseControl import CupPoseControl
from LearningFromDemo import LearningFromDemo
# from RobotMove import RobotMove
from RobotMoveURRobot import RobotMoveURRobot

from main.srv import *

from threading import Lock

REWARD_GOAL = 10

# adapted from https://github.com/ros-teleop/teleop_tools/blob/melodic-devel/key_teleop/scripts/key_teleop.py
class SimpleKeyTeleop():
    def __init__(self):
        self._hz = 50

        self._forward_rate = 0.8
        self._backward_rate = 0.5
        self._rotation_rate = 1.0
        self._last_pressed = {}
        self._x = 0
        self._y = 0

        self._lock = Lock()

        self.client = actionlib.SimpleActionClient('teleop_cup_server', CupMoveAction)
        self.client.wait_for_server()
        self.goal = CupMoveGoal()

        lfd = LearningFromDemo()

        rospy.wait_for_service('update_learning_demo')
        self.update_learning = rospy.ServiceProxy('update_learning_demo', LearningDemo)

        rospy.wait_for_service('reset_demo')
        self.reset_demo = rospy.ServiceProxy('reset_demo', LearningDemo)

        rospy.loginfo('Connect to teleop_cup server: finished')


    movement_bindings = {
        curses.KEY_UP:    (-1,  0),
        curses.KEY_DOWN:  ( 1,  0),
        curses.KEY_LEFT:  ( 0, -1),
        curses.KEY_RIGHT: ( 0,  1),
    }

    def run(self):
        rate = rospy.Rate(self._hz)

        self.cupCtrl = CupPoseControl()
        rospy.sleep(1)
        self.cupCtrl.setPoseDefault()

        self.robot_move = RobotMoveURRobot()
        self.robot_move.initRobotPose()
        self.robot_move.moveArmToCupTop()

        rospy.loginfo('Waiting for coming keys')
        self._pub_cmd = rospy.Subscriber('key_input', Int16, self.cb_key_in)

    def cb_key_in(self, _feedback):
        if self._lock.acquire(False):
            keycode = _feedback.data
            if keycode is -1:
                self._set_x_y()
                self._send_goal()
            self._key_pressed(keycode)
            self._lock.release()

    def _set_x_y(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 0.4:
                keys.append(a)
        x = 0.0
        y = 0.0
        for k in keys:
            l, a = self.movement_bindings[k]
            x += l
            y += a
        self._x = x
        self._y = y

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            rospy.signal_shutdown('Shutting down. See you later')
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()

    def _send_goal(self):

        if self._x != 0 or self._y != 0:
            self.goal.x = self._x
            self.goal.y = self._y
            self.goal.time_factor = 1

            state = self.getState()
            self.client.send_goal(self.goal)

            self.client.wait_for_result()

            result = self.client.get_result()

            reward = result.reward
            action = self.goalToAction(self.goal)
            next_state = self.getState()

            self.update_learning(state, action, reward, next_state)
            rospy.set_param('demo_params/human_goal', [self._x, self._y])

            r_x, r_y, r_z = self.robot_move.cupPoseToRobotPose(result.cup_pose)
            self.robot_move.moveArmTo(r_x, r_y, r_z)
            self.robot_move.showStatus(next_state)

            if reward == REWARD_GOAL:
                self.robot_move.autoLearn(_rounds=3)
                self.robot_move.initDemo()

    def goalToAction(self, _goal):
        if _goal.x == 0 and _goal.y == -1:
            return 0
        if _goal.x == -1 and _goal.y == 0:
            return 1
        if _goal.x == 0 and _goal.y == 1:
            return 2
        if _goal.x == 1 and _goal.y == 0:
            return 3

    def getState(self):
        cup_pos = rospy.get_param('table_params/cup_pos')
        state = (cup_pos[0], cup_pos[1])
        return str(state)

if __name__ == "__main__":
    rospy.init_node('teleop_cup', anonymous=True, disable_signals=True)
    try:
        app = SimpleKeyTeleop()
        app.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
