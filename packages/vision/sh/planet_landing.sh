##planet_landing
gnome-terminal --window -e 'bash -c "roslaunch vision mavros_sitl_planet.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch vision planet_landing.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun vision red_detect; exec bash"' \
--tab -e 'bash -c "sleep 6; rqt_image_view; exec bash"' \
