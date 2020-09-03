## navi using aloam with preperded map 
gnome-terminal --window -e 'bash -c "roslaunch octomap_scout ndt_localizer.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch octomap_scout scout_moon_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch octomap_scout aloam_velodyne_VLP_16.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch octomap_scout scout_aloam_navi.launch; exec bash"' 

