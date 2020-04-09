##　turtlebot navigation demo
gnome-terminal --window -e 'bash -c "roslaunch test_octomap turtlebot_sitl_moon.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch turtlebot_teleop keyboard_teleop.launch --screen; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch test_octomap turtlebot_navi_map.launch; exec bash"' \