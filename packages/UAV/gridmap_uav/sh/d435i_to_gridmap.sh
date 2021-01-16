##d435i pointcloud_to_gridmap
gnome-terminal --window -e 'bash -c "roslaunch realsense2_camera rs_camera.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch gridmap_uav d435i_to_gridmap.launch; exec bash"' \

