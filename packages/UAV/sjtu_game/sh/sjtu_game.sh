##sjtu_game
gnome-terminal --window -e 'bash -c "roslaunch sjtu_game game_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch sjtu_game kino_replan.launch > test.txt; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch darknet_ros yolo_v3.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch sjtu_game sjtu_game.launch; exec bash"' \
