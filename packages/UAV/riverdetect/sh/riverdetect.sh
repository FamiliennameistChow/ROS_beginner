##　uav navigation demo
gnome-terminal --window -e 'bash -c "roslaunch riverdetect mavros_river_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun riverdetect pid_adjust; exec bash"' \
--tab -e 'bash -c "sleep 30; rosrun riverdetect river_detect; exec bash"' \

