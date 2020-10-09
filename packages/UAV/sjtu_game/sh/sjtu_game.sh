##sjtu_game
gnome-terminal --window -e 'bash -c "roslaunch sjtu_game game_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch sjtu_game kino_replan.launch; exec bash"' \
