gnome-terminal --window -e 'bash -c "roslaunch octomap_scout scout_moon_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch octomap_scout aloam_velodyne_VLP_16.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; rosrun gridmap_scout process_kinect_pointcloud; exec bash"' \
--tab -e 'bash -c "sleep 6; rosrun gridmap_scout merge_occupy_map; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch gridmap_scout pointcloud_to_gridmap.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch gridmap_scout test_navi_map.launch; exec bash"' 

