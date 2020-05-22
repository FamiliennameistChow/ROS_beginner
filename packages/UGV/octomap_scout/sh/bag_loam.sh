##depth_to_gridmap
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; cd /media/bornchow/play/BaiduNetdiskDownload/rosbag/outside && rosbag play 2020-05-12-16-02-30.bag; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch octomap_scout aloam_velodyne_VLP_16.launch; exec bash"' \
# --tab -e 'bash -c "sleep 5; roslaunch octomap_scout hector_loam_velodyne.launch; exec bash"' \

