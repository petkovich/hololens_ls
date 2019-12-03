roscore
cd /catkin_ws/safelog_common/LocationServer/Test/build
./server ../../Config/socket.config 
./fms ../../Config/socket.config 
./sim ../../Config/socket.config 10
roslaunch hololens_ls hololens_ls.launch 
rostopic pub /human_pose geometry_msgs/Pose "{position: {x: 1.0, y: 5.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"

rosservice call /human_path_srv 
rosservice call /close_robot_srv 
