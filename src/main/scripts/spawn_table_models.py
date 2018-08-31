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

def getXYZFromIJ(i, j, _grid_size, _table_size, _margin_size):
    x = _grid_size*i + _grid_size/2 - (_table_size - 2*_margin_size)/2
    y = _grid_size*j + _grid_size/2 - (_table_size - 2*_margin_size)/2
    z = 0.80
    return x, y, z

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
    # mat size: 0.03 (radius), 0.005 (thickness)

    table_size = 0.913
    cubic_size = 0.040
    mat_size = 0.03 # radius
    cup_size = 0.035 # radius

    margin_size = 0.05

    flag_first_cup = False

    num_grid, array_map = loadPlacementMap()
    grid_size = (table_size - 2*margin_size)/num_grid

    item_name = []
    pos_cup = []
    list_pos_cup = []
    pos_mat = []

    if rospy.has_param('table_params/item_name'):
        array_item_name = rospy.get_param('table_params/item_name')
        rospy.delete_param('table_params')
        for each in array_item_name:
            delete_model(each)

    table_config = {}

    for i in xrange(num_grid):
        table_config[str(i)] = {}
        for j in xrange(num_grid):
            x, y, z = getXYZFromIJ(i, j, grid_size, table_size, margin_size)
            table_config[str(i)][str(j)] = ""
            if array_map[i, j] == 1:
                table_config[str(i)][str(j)] = "cup"
                list_pos_cup.append([i, j])
                if not flag_first_cup:
                    placeObjects(x, y, z, "cup", xml_cup)
                    item_name.append("cup")
                    flag_first_cup = True
            if array_map[i, j] == 2:
                table_config[str(i)][str(j)] = "mat"
                placeObjects(x, y, z, "mat", xml_mat)
                item_name.append("mat")
                pos_mat = [i, j]
            if array_map[i, j] == 3:
                table_config[str(i)][str(j)] = "cubic"
                placeObjects(x, y, z, "cubic-%d-%d"%(i, j), xml_cubic)
                item_name.append("cubic-%d-%d"%(i, j))

    params = {}
    params['item_name'] = item_name
    params['cup_pos'] = pos_cup
    params['cup_pos_init'] = list_pos_cup[0]
    params['list_cup_pos'] = list_pos_cup
    params['mat_pos'] = pos_mat
    params['grid_size'] = grid_size
    params['margin_size'] = margin_size
    params['table_size'] = table_size
    params['cubic_size'] = cubic_size
    params['mat_size'] = mat_size
    params['cup_size'] = cup_size
    params['table_config'] = table_config

    rospy.set_param('table_params', params)
