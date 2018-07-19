#!/usr/bin/env python
import sys
import rospy
import curses
import math
import actionlib
from main.msg import CupMoveAction, CupMoveGoal, CupMoveActionResult, CupMoveActionFeedback

from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

# from https://github.com/ros-teleop/teleop_tools/blob/melodic-devel/key_teleop/scripts/key_teleop.py
class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError, 'lineno out of bounds'
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()

# from https://github.com/ros-teleop/teleop_tools/blob/melodic-devel/key_teleop/scripts/key_teleop.py
class SimpleKeyTeleop():
    def __init__(self, interface):
        self._interface = interface
        self._pub_cmd = rospy.Publisher('key_input', Pose)

        self._hz = 50

        self._forward_rate = 0.8
        self._backward_rate = 0.5
        self._rotation_rate = 1.0
        self._last_pressed = {}
        self._x = 0
        self._y = 0

        self.client = actionlib.SimpleActionClient('teleop_cup', CupMoveAction)
        self.client.wait_for_server()
        self.goal = CupMoveGoal()
        rospy.loginfo('Connect to teleop_cup server: finished')

    movement_bindings = {
        curses.KEY_UP:    (-1,  0),
        curses.KEY_DOWN:  ( 1,  0),
        curses.KEY_LEFT:  ( 0, -1),
        curses.KEY_RIGHT: ( 0,  1),
    }

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        rospy.loginfo('Waiting for coming keys')
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_x_y()
            self._send_goal()
            rate.sleep()

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
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()

    def _send_goal(self):
        self._interface.clear()
        output_str = ""
        if self._x > 0:
            output_str = "backward "
        if self._x < 0:
            output_str = output_str + "forward "
        if self._y > 0:
            output_str = output_str + "to right "
        if self._y < 0:
            output_str = output_str + "to left "

        if len(output_str) != 0:
            self._interface.write_line(2, 'Moving %s' %output_str)
        self._interface.write_line(5, 'Use arrow keys to move, q to exit.')
        self._interface.refresh()

        if self._x != 0 or self._y != 0:
            self.goal.x = self._x
            self.goal.y = self._y
            self.goal.time_factor = 1
            self.client.send_goal(self.goal)
            self.client.wait_for_result()


def main(stdscr):
    rospy.init_node('teleop_cup', anonymous=True)
    app = SimpleKeyTeleop(TextWindow(stdscr))
    app.run()

if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
