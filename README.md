# line-following-AGV-gazebo-sim

## Environment 
gazebo 9
Ubuntu 18.04
ROS melodic
Python 2.7

## Run 
### one AGV: 

```
roslaunch agv_gazebo agv_world.launch
```

```
python src/line-following-AGV-gazebo-sim/agv_gazebo/src/line_following_agv.py
```

### five AGVs
```
roslaunch agv_gazebo multi_agv.launch
```

```
python src/line-following-AGV-gazebo-sim/agv_gazebo/src/line_following_agv_multi.py
```

## Test 

### simple test cmd_vel

```
roslaunch agv_gazebo agv_world.launch
```
```
rostopic pub -r 10 agv_0/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}'
```

### teleop agv
```
roslaunch agv_gazebo agv_world.launch
```
```
python src/line-following-AGV-gazebo-sim/agv_gazebo/src/teleop_agv.py
```

