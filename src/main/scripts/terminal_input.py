import sys
import curses
import rospy

from std_msgs.msg import Int16

# from https://github.com/ros-teleop/teleop_tools/blob/melodic-devel/key_teleop/scripts/key_teleop.py
class TextWindow():
    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._pub_cmd = rospy.Publisher('key_input', Int16)
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def pub_key(self):
        keycode_ch = self._screen.getch()
        keycode = keycode_ch if keycode_ch != -1 else -1

        keycode_msg = Int16()
        keycode_msg.data = keycode

        try:
            self._pub_cmd.publish(keycode_msg)
        except rospy.exceptions.ROSException:
            pass

        return keycode

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

if __name__ == "__main__":
    import time
    import rosbag
    import datetime
    tmp = time.time()
    file_name = datetime.datetime.fromtimestamp(tmp).strftime('%Y-%m-%d-%H-%M-%S')
    bag = rosbag.Bag("%s.bag"%file_name, "w")