##uav pointcloud_to_gridmap
gnome-terminal --window -e 'bash -c "roslaunch gridmap_uav get_downward_depth.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun px4_mavros new_offb_keyctl2; exec bash"' \
--tab -e 'bash -c "sleep 25; roslaunch gridmap_uav pointcloud_to_gridmap.launch; exec bash"' \

