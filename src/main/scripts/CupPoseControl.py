#!/usr/bin/env python
import rospy
import actionlib

from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

from spawn_table_models import getXYZFromIJ

class CupPoseControl:
    def __init__(self):
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    def setPose(self, _pose):
        model_info = ModelState()
        model_info.model_name = 'cup'
        model_info.reference_frame = 'world'
        model_info.pose = _pose
        self.pub.publish(model_info)

    def setPoseDefault(self):
        pre_index = rospy.get_param('table_params/cup_pos_init')
        rospy.set_param('table_params/cup_pos', pre_index)
        grid_size = rospy.get_param('table_params/grid_size')
        table_size = rospy.get_param('table_params/table_size')
        margin_size = rospy.get_param('table_params/margin_size')
        x, y, z = getXYZFromIJ(pre_index[0], pre_index[1], grid_size, table_size, margin_size)

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        self.setPose(target_pose)

