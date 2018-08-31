import os
import sys
import curses
import rospy
import rosbag

from std_msgs.msg import Int16
from time import gmtime, strftime

# adapted from https://github.com/ros-teleop/teleop_tools/blob/melodic-devel/key_teleop/scripts/key_teleop.py
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

        if keycode != -1:
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

## def main_rosbag(stdscr):
##     wnd = TextWindow(stdscr)
## 
##     while not rospy.is_shutdown():
##         wnd.clear()
##         output_str = ""
##         if keycode == curses.KEY_DOWN:
##             output_str = "backward "
##         if keycode == curses.KEY_UP:
##             output_str = output_str + "forward "
##         if keycode == curses.KEY_RIGHT:
##             output_str = output_str + "to right "
##         if keycode == curses.KEY_LEFT:
##             output_str = output_str + "to left "
## 
##         if len(output_str) != 0:
##             wnd.write_line(2, 'Moving %s' %output_str)
##         wnd.write_line(5, 'Use arrow keys to move, q to exit.')
##         wnd.refresh()

def main_key_in(stdscr):
    wnd = TextWindow(stdscr)

    timestamp = strftime("%Y%m%d%H%M%S", gmtime())
    prefix = 'bag'
    if not os.path.exists(prefix):
        os.mkdir(prefix)
    bag_filename = os.path.join(prefix, "%s.bag"%timestamp)
    saved_bag = rosbag.Bag(bag_filename, 'w')

    ## def bagClose():
    ##     saved_bag.reindex()
    ##     saved_bag.close()
    ##     rospy.loginfo('Done')

    ## rospy.on_shutdown(bagClose)

    wnd.clear()
    wnd.write_line(2, 'Hello')
    wnd.write_line(5, 'Use arrow keys to move, q to exit.')
    wnd.refresh()

    while not rospy.is_shutdown():
        keycode = wnd.pub_key()

        if keycode == -1:
            continue
        keycode_msg = Int16()
        keycode_msg.data = keycode

        saved_bag.write('key_input', keycode_msg)

        if keycode == ord('q'):
            saved_bag.close()
            rospy.signal_shutdown('Shutting down. See you later')
            break

        wnd.clear()
        output_str = ""
        if keycode == curses.KEY_DOWN:
            output_str = "backward "
        if keycode == curses.KEY_UP:
            output_str = output_str + "forward "
        if keycode == curses.KEY_RIGHT:
            output_str = output_str + "to right "
        if keycode == curses.KEY_LEFT:
            output_str = output_str + "to left "

        if len(output_str) != 0:
            wnd.write_line(2, 'Moving %s' %output_str)
        wnd.write_line(5, 'Use arrow keys to move, q to exit.')
        wnd.refresh()

    # rospy.loginfo('Done')

if __name__ == "__main__":
    # import time
    # import rosbag
    # import datetime
    # tmp = time.time()
    # file_name = datetime.datetime.fromtimestamp(tmp).strftime('%Y-%m-%d-%H-%M-%S')
    # bag = rosbag.Bag("%s.bag"%file_name, "w")
    rospy.init_node('terminal_input', anonymous=True)
    curses.wrapper(main_key_in)
    # curses.wrapper(main_rosbag)