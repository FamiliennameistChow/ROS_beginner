##mark_tracking
gnome-terminal --window -e 'bash -c "roslaunch vision mavros_posix_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch vision auto_landing.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; rqt_image_view; exec bash"' \
