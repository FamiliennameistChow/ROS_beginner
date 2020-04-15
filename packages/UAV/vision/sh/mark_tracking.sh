##mark_tracking
gnome-terminal --window -e 'bash -c "roslaunch vision mavros_sitl_mark.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch vision mark_tracking.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; rosrun vision gazebo_reset.py; exec bash"' \
--tab -e 'bash -c "sleep 6; rqt_image_view; exec bash"' \
