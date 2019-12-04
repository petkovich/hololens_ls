LocationServer and FMS need to be in workspace folder or change these lines in project's CMakeLists:
```

# Path to the safelog_common directory.
set(SAFELOG_COMMON ${PROJECT_SOURCE_DIR}/../../safelog_common)

# Path to the safelog_fleet_management_system directory.
set(SAFELOG_FMS ${PROJECT_SOURCE_DIR}/../../safelog_fleet_management_system)
```

Run simulation with:
```

cd /catkin_ws/safelog_common/LocationServer/Test/build
./server ../../Config/socket.config 
./fms ../../Config/socket.config 
./sim ../../Config/socket.config 10
roslaunch hololens_ls hololens_ls.launch 
rostopic pub /human_pose geometry_msgs/Pose "{position: {x: 1.0, y: 5.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"

rosservice call /human_path_srv 

rostopic pub /human_pose geometry_msgs/Pose "{position: {x: 10.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"

rosservice call /close_robot_srv 
```

Run experiment with (check LS IP in config file):
```

roslaunch hololens_ls augsburg.launch 

rosservice call /human_path_srv 

rosservice call /close_robot_srv 
```
