## 3D rrt path planning in octomap
gnome-terminal --window -e 'bash -c "rosrun target_landing attacker _uav_num:=8 __name:=uav8_attacker; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun target_landing attacker _uav_num:=2 __name:=uav2_attacker; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun target_landing attacker _uav_num:=3 __name:=uav3_attacker; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun target_landing attacker _uav_num:=4 __name:=uav4_attacker; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun target_landing attacker _uav_num:=5 __name:=uav5_attacker; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun target_landing attacker _uav_num:=6 __name:=uav6_attacker; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun target_landing attacker _uav_num:=7 __name:=uav7_attacker; exec bash"' \
# --tab -e 'bash -c "sleep 20; rosrun target_landing attacker _uav_num:=8 __name:=uav8_attacker; exec bash"' \

