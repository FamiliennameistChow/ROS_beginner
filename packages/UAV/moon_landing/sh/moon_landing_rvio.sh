##moon_landing
gnome-terminal --window -e 'bash -c "roslaunch moon_landing get_downward_depth.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun px4_mavros new_offb_keyctl2; exec bash"' \
--tab -e 'bash -c "sleep 20; roslaunch moon_landing start_rvio.launch; exec bash"' \
