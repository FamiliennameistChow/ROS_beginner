##moon_landing
gnome-terminal --window -e 'bash -c "roslaunch moon_landing moon_landing_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch moon_landing start_safe_landing.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun moon_landing moon_landing_node; exec bash"' \
