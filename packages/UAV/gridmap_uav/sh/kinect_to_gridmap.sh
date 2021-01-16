##kinect pointcloud_to_gridmap
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; rosbag play /home/danny/catkin_ws/src/UAV/gridmap_uav/data/2021-01-13-11-04-31.bag; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch gridmap_uav kinect_to_gridmap.launch; exec bash"' \

