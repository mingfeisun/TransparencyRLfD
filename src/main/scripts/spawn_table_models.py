#!/usr/bin/env python
import tf
import rospy
from gazebo_msgs.srv import *
from geometry_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node("spawn_table_models")

    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    s = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))

    package_folder = "/home/mingfei/Documents/ExpressiveRLfD/src/main/"

    xml_cup = ""
    xml_cubic = ""

    with open(package_folder + "worlds/cup/model.sdf", "r") as f:
        xml_cup = f.read()

    with open(package_folder + "worlds/cubic/model.sdf", "r") as f:
        xml_cubic = f.read()

    cup_name = "cup_0"
    cubic_name = "cubic_0"

    # cup size: 0.13 height
    # cubic size: 0.05 * 0.05 * 0.05
    # table size: 0.913 * 0.913 * 0.755 (0.04 thickness)

    pos_cup_x = 0
    pos_cup_y = 0
    pos_cup_z = 0.755

    pose_cup = Pose(Point(x=pos_cup_x, y=pos_cup_y, z=pos_cup_z), orient)

    pos_cubic_x = 0.3
    pos_cubic_y = 0
    pos_cubic_z = 0.755 + 0.04/2 + 0.05/2

    pose_cubic = Pose(Point(x=pos_cubic_x, y=pos_cubic_y, z=pos_cubic_z), orient)

    delete_model(cup_name)
    delete_model(cubic_name)

    s(cup_name, xml_cup, "", pose_cup, "world")
    s(cubic_name, xml_cubic, "", pose_cubic, "world")
