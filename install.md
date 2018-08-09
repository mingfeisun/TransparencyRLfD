## Required packages

### PR2 robot
* [pr2_controllers](https://github.com/PR2/pr2_controllers)
* [pr2_simulator](https://github.com/PR2/pr2_simulator)
* [pr2_apps](https://github.com/PR2/pr2_apps)
* [pr2_common_actions](https://github.com/pr2/pr2_common_actions)

### Gazebo 
* [gazebo](https://github.com/mingfeisun/gazebo) 
**Required to be compiled from source**
``` bash
cd gazebo
mkdir build & cd build
cmake ..
sudo make install -j12 # 12 is the maximum process number supported by cpu
```
*Gazebo 8 - SDFormat 5.0 (SDF protocol <= 1.6)*

* [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs)

### UR10 Robot arm

After catkin_make, see the commented codes in the ur_gazebo ur10.launch, I comment some codes to about the world.

Run
``` bash
roslaunch ur_gazebo ur10.launch
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true
roslaunch ur10_moveit_config moveit_rviz.launch config:=true
rosrun main ur10_move_test.py
``` 
to see the effects from the original codes.



### Other requirements
Install from apt:
``` bash
sudo apt-get install ros-kinetic-pr2-msgs
sudo apt-get install ros-kinetic-pr2-controllers-msgs
sudo apt-get install ros-kinetic-common-msgs 
```

### Demonstration task
* [Grid World](https://github.com/rlcode/reinforcement-learning/tree/master/1-grid-world)
* [Pinball Domain](http://irl.cs.brown.edu/software.php)

### Reinforcement learning algorithms
* [RLCode Reinforcement Learning](https://github.com/rlcode/reinforcement-learning)
