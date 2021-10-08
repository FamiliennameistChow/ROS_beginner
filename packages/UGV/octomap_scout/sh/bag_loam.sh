##depth_to_gridmap
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; cd /media/bornchow/YSC/ROSBAGS/UGV/20201221 && rosbag play -r 0.9 --clock 004.big_ring.bag --topic /zed_node/left_raw/image_raw_color /velodyne_points; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch octomap_scout aloam_velodyne_VLP_16.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch octomap_scout tf_trans_aloam.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch octomap_scout scout_aloam_navi.launch; exec bash"' \
# --tab -e 'bash -c "sleep 5; roslaunch octomap_scout lego_loam.launch; exec bash"' \

# --tab -e 'bash -c "sleep 5; cd /home/bornchow/Downloads/rosbag && rosbag play -s 60  --clock 2020-06-01-16-50-11imu.bag --topic /velodyne_points; exec bash"' \
# --tab -e 'bash -c "sleep 5; cd /home/bornchow/Downloads/rosbag/outside && rosbag play -s 60  --clock 2020-05-12-16-02-30.bag --topic /velodyne_points; exec bash"' \
# 2020-08-10-12-10-31.bag 全场测试

