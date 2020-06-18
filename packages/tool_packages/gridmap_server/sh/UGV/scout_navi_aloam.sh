##depth_to_gridmap
gnome-terminal --window -e 'bash -c "roslaunch octomap_scout scout_moon_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch octomap_scout aloam_velodyne_VLP_16.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch octomap_scout tf_trans_aloam.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch gridmap_server pointcloud_to_gridmap.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch gridmap_server test_navi_map.launch; exec bash"' 

