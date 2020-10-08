## 3D rrt path planning in octomap
gnome-terminal --window -e 'bash -c "roslaunch octomap_navi pc_to_octomap.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch octomap_navi plan_opt.launch > test.txt; exec bash"' 

