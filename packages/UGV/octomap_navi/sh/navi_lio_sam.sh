## 3D rrt path planning in octomap
gnome-terminal --window -e 'bash -c "roslaunch octomap_navi octomap_convert.launch slamType:=lio-sam; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch octomap_navi plan_opt.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch octomap_navi traj_server.launch; exec bash"' 

