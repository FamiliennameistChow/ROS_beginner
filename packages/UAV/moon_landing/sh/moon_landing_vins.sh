##moon_landing
gnome-terminal --window -e 'bash -c "roslaunch moon_landing get_downward_depth.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun moon_landing moon_landing_node; exec bash"' \
--tab -e 'bash -c "sleep 25; roslaunch moon_landing start_vins.launch; exec bash"' \
