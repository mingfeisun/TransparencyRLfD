#!/usr/bin/env python
import tf
import pandas
import numpy
import rospy
from gazebo_msgs.srv import *
from geometry_msgs.msg import *

package_folder = "/home/mingfei/Documents/ExpressiveRLfD/src/main/"

def loadPlacementMap():
    filename = "configs/placement_map.txt"
    df_map = pandas.read_csv(package_folder+filename, sep=' ', header=None)
    array_map = df_map.values
    num_grid = numpy.shape(array_map)[0]
    return num_grid, array_map

def placeObjects(_x, _y, _z, _type, _xml):
    item_name = _type
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    pose = Pose(Point(x=_x, y=_y, z=_z), orient)
    s(item_name, _xml, "", pose, "world")

if __name__ == '__main__':
    rospy.init_node("spawn_table_models")

    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    s = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    xml_cup = ""
    xml_cubic = ""
    xml_mat = ""

    with open(package_folder + "worlds/cup/model.sdf", "r") as f:
        xml_cup = f.read()
    with open(package_folder + "worlds/cubic/model.sdf", "r") as f:
        xml_cubic = f.read()
    with open(package_folder + "worlds/mat/model.sdf", "r") as f:
        xml_mat = f.read()

    # cup size: 0.13 height
    # cubic size: 0.05 * 0.05 * 0.05
    # table size: 0.913 * 0.913 * 0.755 (0.04 thickness)

    table_size = 0.913
    margin_distance = 0.05

    num_grid, array_map = loadPlacementMap()
    grid_size = (table_size - 2*margin_distance)/num_grid

    item_name = []
    pos_cup = []
    pos_mat = []

    if rospy.has_param('table_params/item_name'):
        array_item_name = rospy.get_param('table_params/item_name')
        rospy.delete_param('table_params')
        for each in array_item_name:
            delete_model(each)

    for i in xrange(num_grid):
        for j in xrange(num_grid):
            x = grid_size*i + grid_size/2 - table_size/2
            y = grid_size*j + grid_size/2 - table_size/2
            z = 0.80
            if array_map[i, j] == 1:
                placeObjects(x, y, z, "cup", xml_cup)
                item_name.append("cup")
                pos_cup = [x, y, z]
            if array_map[i, j] == 2:
                placeObjects(x, y, z, "mat", xml_mat)
                item_name.append("mat")
                pos_mat = [x, y, z]
            if array_map[i, j] == 3:
                placeObjects(x, y, z, "cubic-%d-%d"%(i, j), xml_cubic)
                item_name.append("cubic-%d-%d"%(i, j))

    params = {}
    params['item_name'] = item_name
    params['cup_pos'] = pos_cup
    params['mat_pos'] = pos_mat
    params['grid_size'] = grid_size

    rospy.set_param('table_params', params)
