#!/usr/bin/env python
import sys
import rospy
import rosbag
import curses
import math
import actionlib
from main.msg import CupMoveAction, CupMoveGoal, CupMoveActionResult, CupMoveActionFeedback

from terminal_input import TextWindow

from std_msgs.msg import Int16, String
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

        self._last_pressed = {}
        self._x = 0
        self._y = 0

        self._buffer_size = 50
        self._key_buffer = []

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
        curses.KEY_UP:   (-1,  0),
        curses.KEY_DOWN:    ( 1,  0),
        curses.KEY_LEFT:      ( 0, -1),
        curses.KEY_RIGHT:    ( 0,  1),
    }

    def init(self):
        rate = rospy.Rate(self._hz)

        self.cupCtrl = CupPoseControl()
        rospy.sleep(1)
        self.cupCtrl.setPoseDefault()

        self.robot_move = RobotMoveURRobot()
        self.robot_move.initRobotPose()
        self.robot_move.moveArmToCupTop()

        rospy.loginfo('Waiting for coming keys')
        self._pub_cmd = rospy.Subscriber('key_input', Int16, self.cb_key_in)

    def run(self):
        running_code = 0
        rospy.set_param('current_status', 'listening')
        while running_code != -1:
            running_code = self._set_x_y()
            if running_code == 1:
                self._lock.acquire()
                self._send_goal()
                self._lock.release()
            if running_code == 2:
                self._lock.acquire()
                self._yes_to_go()
                self._start_next_demo()
                self._lock.release()
            if running_code == 3:
                self._lock.acquire()
                self._yes_to_go()
                self._lock.release()

    def cb_key_in(self, _feedback):
        if self._lock.acquire():
            rospy.set_param('current_status', 'listening')
            keycode = _feedback.data
            if keycode == -1:
                return
            self._key_pressed(keycode)
            self._lock.release()

    def _set_x_y(self):
        now = rospy.get_time()
        flag_changed = False
        self._lock.acquire() # lock to avoid modifying _last_pressed
        # for a in self._last_pressed:
        #     if now - self._last_pressed[a] < 0.4:
        #         keys.append(a)
        # self._lock.release()
        if len(self._key_buffer) != 0:
            first_pair = self._key_buffer.pop(0)
            key = first_pair["keycode"]
            if key == ord('q'):
                self._lock.release()
                return -1
            if key == ord('y'):
                self._lock.release()
                return 2 
            if key == ord('j'):
                self._lock.release()
                return 3
            x = 0.0
            y = 0.0
            l, a = self.movement_bindings[key]
            x += l
            y += a
            self._x = x
            self._y = y
            self._lock.release()
            return 1
        self._lock.release()
        return 0

    def _key_pressed(self, keycode):
        if len(self._key_buffer) > self._buffer_size:
            self._key_buffer.pop(0)
        self._key_buffer.append({"keycode": keycode, "timestamp": rospy.get_time() })

    def _start_next_demo(self):
        rospy.set_param("current_status", 'training')
        self.robot_move.initDemo_step1()
        self.robot_move.autoLearn(_rounds=5)
        self.robot_move.initDemo_step2()
        rospy.set_param("current_status", 'listening')

    def _yes_to_go(self):
        states_to_go = self.robot_move.pre_states
        tmp_goal = CupMoveGoal()
        for i in range(1, len(states_to_go)):
            tmp_goal.x = eval(states_to_go[i])[0] - eval(states_to_go[i-1])[0] 
            tmp_goal.y = eval(states_to_go[i])[1] - eval(states_to_go[i-1])[1] 
            tmp_goal.time_factor = 5

            state = states_to_go[i-1]
            self.client.send_goal(tmp_goal)
            self.client.wait_for_result()
            result = self.client.get_result()
            reward = result.reward
            action = self.goalToAction(self.goal)
            next_state = states_to_go[i]

            self.update_learning(state, action, reward, next_state)
            rospy.set_param('demo_params/human_goal', [self._x, self._y])

    def _send_goal(self):
        self.goal.x = self._x
        self.goal.y = self._y
        self.goal.time_factor = 5

        state = self.getState()
        self.client.send_goal(self.goal)

        self.client.wait_for_result()

        result = self.client.get_result()

        reward = result.reward
        action = self.goalToAction(self.goal)
        next_state = self.getState()

        self.update_learning(state, action, reward, next_state)
        rospy.set_param('demo_params/human_goal', [self._x, self._y])

        # r_x, r_y, r_z = self.robot_move.cupPoseToRobotPose(result.cup_pose)
        # self.robot_move.moveArmTo(r_x, r_y, r_z)

        if reward != REWARD_GOAL:
            self.robot_move.showStatus(next_state)
        else:
            self._start_next_demo()

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
    
    NULL = -1
    GESTURING = 0
    GESTURING_PAUSING = 1
    GESTURING_SPEED = 2

    ADAPTIVE = 3
    SHOW_UNCERTAINTY = 4
    SHOW_POLICY = 5

    TRACE_TYPE_CONSERVATIVE =  1
    TRACE_TYPE_BOLD = 2
    TRACE_TYPE_ADAPTIVE = 3

    rospy.init_node('teleop_cup', anonymous=True, disable_signals=True)

    rospy.set_param('status_completed', False)
    rospy.set_param('showing_mode', NULL)
    # rospy.set_param('showing_mode', ADAPTIVE)

    rospy.set_param('match_trace_type', TRACE_TYPE_CONSERVATIVE)
    # rospy.set_param('match_trace_type', TRACE_TYPE_BOLD)
    # rospy.set_param('match_trace_type', TRACE_TYPE_ADAPTIVE)

    try:
        app = SimpleKeyTeleop()
        app.init()
        app.run()
    except rospy.ROSInterruptException:
        pass
