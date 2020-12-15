for pname in measure_cpu dag_controller operator navigator mapper joy_node shimfreqnode rviz remote_joy stage get_map_client explore_client set_goal_client move_dynamic_obstacles_nav2d
do
	echo "Killing_$pname"
	kill -15 $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') 
	kill -9 $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') 
done
