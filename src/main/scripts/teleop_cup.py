#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

current_twist = Twist()

def keyListening(data):
    current_twist = data.data

if __name__ == "__main__":
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    sub = rospy.Subscriber('cmd_vel', Twist, keyListening)

    rospy.init_node('teleop_cup', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        data_to_pub = ModelState()
        data_to_pub.Data["model_name"] = "cup"
        data_to_pub.Data["pose"] = Pose()
        data_to_pub.Data["twist"] = current_twist

        pub.publish(data_to_pub)
        rate.sleep()
