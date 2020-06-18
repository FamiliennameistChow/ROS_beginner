##depth_to_gridmap
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; cd /home/bornchow/Downloads/rosbag/outside && rosbag play --clock 2020-05-12-16-02-30.bag --topic /velodyne_points; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch octomap_scout aloam_velodyne_VLP_16.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch octomap_scout tf_trans_aloam.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch octomap_scout test_navi_map.launch; exec bash"' \

