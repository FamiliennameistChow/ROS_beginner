##　uav navigation demo
gnome-terminal --window -e 'bash -c "roslaunch test_octomap mavros_sitl_binocular.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun px4_simulation offb_keyctl; exec bash"' \
--tab -e 'bash -c "sleep 20; roslaunch test_octomap test_octomap.launch; exec bash"' \

