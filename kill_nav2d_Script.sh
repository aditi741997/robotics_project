j=1
j=$((2*j))
echo $j
j=0
j=$((2*j))
echo $j
for pname in campreprocess objdetect rosc rosout darknet_ros rosmaster measure_cpu dag_controller operator navigator mapper joy_node shimstreamnode campreprocess shimfreqnode rviz remote_joy stage get_map_client explore_client set_goal_client move_dynamic_obst
do
	echo "Killing_$pname"
	kill -15 $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') 
	kill -9 $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') 
done
