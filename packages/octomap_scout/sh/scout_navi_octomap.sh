##depth_to_gridmap
gnome-terminal --window -e 'bash -c "roslaunch octomap_scout scout_moon_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun octomap_scout tf_slam_publisher; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch octomap_scout hector_loam_velodyne.launch; exec bash"' \
#--tab -e 'bash -c "sleep 15; roslaunch octomap_scout scout_navi_map.launch; exec bash"' \

