#!/usr/bin/env python
import copy
import rospy
import actionlib

from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState

from spawn_table_models import getXYZFromIJ

class CupPoseControl:
    def __init__(self):
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        
        rospy.wait_for_service('/gazebo/get_model_state')
        self.model_state_client = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.idx_current_pos = 0

    def setPose(self, _pose_cup):
        model_info_cup = ModelState()
        model_info_cup.model_name = 'cup'
        model_info_cup.reference_frame = 'world'
        model_info_cup.pose = _pose_cup
        self.pub.publish(model_info_cup)

    def setPoseDefault(self):
        self.list_cup_pos = rospy.get_param('table_params/list_cup_pos')
        cup_index = self.list_cup_pos[self.idx_current_pos]

        # rospy.loginfo('Current index: %s'%str(self.idx_current_pos))
        # rospy.loginfo('Current cup position: %s'%str(pre_index))

        rospy.set_param('table_params/cup_pos', cup_index)
        grid_size = rospy.get_param('table_params/grid_size')
        table_size = rospy.get_param('table_params/table_size')
        margin_size = rospy.get_param('table_params/margin_size')
        x, y, z = getXYZFromIJ(cup_index[0], cup_index[1], grid_size, table_size, margin_size)

        target_pose_cup = Pose()
        target_pose_cup.position.x = x
        target_pose_cup.position.y = y
        target_pose_cup.position.z = z

        mat_index = rospy.get_param('table_params/mat_pos')
        x, y, z = getXYZFromIJ(mat_index[0], mat_index[1], grid_size, table_size, margin_size)
        target_pose_mat = Pose()
        target_pose_mat.position.x = x
        target_pose_mat.position.y = y
        target_pose_mat.position.z = z

        model_info_mat = ModelState()
        model_info_mat.model_name = 'mat'
        model_info_mat.reference_frame = 'world'
        model_info_mat.pose = target_pose_mat
        self.pub.publish(model_info_mat)

        self.setPose(target_pose_cup)

    def changeDefaultPose(self, _new_pose):
        rospy.set_param('table_params/cup_pos_init', _new_pose)

    def changeDefaultPoseToNext(self):
        self.list_cup_pos = rospy.get_param('table_params/list_cup_pos')
        num_pos = len(self.list_cup_pos)
        self.idx_current_pos += 1
        if self.idx_current_pos > num_pos - 1:
            self.idx_current_pos = self.idx_current_pos % num_pos
        self.changeDefaultPose(self.list_cup_pos[self.idx_current_pos])
        return num_pos - 1 - self.idx_current_pos

    def getPose(self):
        cup_pose = self.model_state_client('cup', "").pose
        return cup_pose

if __name__ == "__main__":
    rospy.init_node('test')
    rate = rospy.Rate(1)
    test = CupPoseControl()
    while not rospy.is_shutdown():
        # print test.getPose()
        rate.sleep()
