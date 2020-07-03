gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; cd /media/danny/Danny-移动硬盘/rosbag && rosbag play --clock 2020-06-01-16-50-11imu.bag --topic /velodyne_points; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch octomap_scout aloam_velodyne_VLP_16.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch octomap_scout tf_trans_aloam.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch gridmap_scout pointcloud_to_gridmap.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch gridmap_scout test_navi_map.launch; exec bash"' 
# --tab -e 'bash -c "sleep 10; rosrun gridmap_scout process_kinect_pointcloud; exec bash"' \

