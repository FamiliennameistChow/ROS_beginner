##depth_to_gridmap
gnome-terminal --window -e 'bash -c "roslaunch gridmap_server get_downward_depth.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun px4_mavros new_offb_keyctl2; exec bash"' \
--tab -e 'bash -c "sleep 26; roslaunch gridmap_server pointcloud_to_gridmap.launch; exec bash"' \

