##depth_to_gridmap
gnome-terminal --window -e 'bash -c "roslaunch grid_map_uav get_downward_depth.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun px4_mavros new_offb_keyctl; exec bash"' \
--tab -e 'bash -c "sleep 15; rosrun grid_map_uav tf_camera2map; exec bash"' \
--tab -e 'bash -c "sleep 30; roslaunch grid_map_uav depth_to_gridmap.launch; exec bash"' \

