# APIs
<img src="https://raw.githubusercontent.com/yding25/pic_share/master/segbot_ur5_vacuum_sucker.png" width="300" />

## Grasp and ungrasp object
Load segbot_ur5, controllers, rviz, ...
```
roslaunch tamp_perception segbot_ur5_vacuum.launch
```
Load vacuum gripper plugin
```
rosrun tamp_perception vacuum_control.py
```
Run script to execute tasks
```
rosrun tamp_perception task_grasp_putdown.py
```

## Create map
Load segbot_ur5, controllers, rviz, ...
```
roslaunch tamp_perception segbot_ur5_gmapping.launch
```

Load gmapping package
```
rosrun gmapping slam_gmapping
```

Control segbot using keyboard
```
rosrun segbot_bringup teleop_twist_keyboard
```

Save created map
```
cd /home/yan/catkin_ws1/src/tamp_perception/maps
```
```
rosrun map_server map_saver -f two_tables
```
