##　turtlebot navigation demo
gnome-terminal --window -e 'bash -c "roslaunch navi_ros scout_sitl_moon.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch scout_bringup scout_teleop_keyboard.launch --screen; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch navi_ros scout_navi_map.launch; exec bash"' \