##depth_to_gridmap
gnome-terminal --window -e 'bash -c "roslaunch octomap_scout scout_moon_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 10;  rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash"' \
--tab -e 'bash -c "sleep 15; roslaunch octomap_scout lego_loam.launch; exec bash"' \
# --tab -e 'bash -c "sleep 10; roslaunch octomap_scout test_octomap.launch; exec bash"' 



