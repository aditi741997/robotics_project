export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
source devel/setup.bash
# nohup roscore &
sleep 2s
for scanF in 14.70588235 #F6N:12.30769230 #F1:10.29601029 
do
    for r in 42 43 44 45 46 #29 30 31 32 33 34 35 36 37 38 39 40 #14 15 16 #17 18 19 20 #6 7 8 9 10 2 3 4 5 # 
    do
	rm ../robot_nav2d_obstacleDist_logs_.txt
	rosclean purge -y
        ename="FO_Default_2c_run$r"

	echo "DELETING OLD LOGFILES For this expt:"
	rm "../robot_nav2d_obstacleDist_logs_${ename}.txt"
	rm "../robot_nav2d_${ename}_rt_stats.txt"
	for thing in local_map navigator_plan navigator_cmd mapper_mapUpdate mapper_scanCB operator_loop
	do
		rm "../robot_nav2d_${thing}_stats_${ename}.txt"
	done
	echo "DELETEC OLD LOGFILES for this expt."

        # launch stage simulator:
        taskset -a -c 7-12 roslaunch nav2d_tutorials tutorial4_stage_largemap.launch &
        sleep 2s
	echo $ename
	rosparam set /expt_name $ename
        sleep 2s

        # shim node to control frequency of laser scans: 
        #taskset -a -c 1 chrt -f 4 rosrun beginner_tutorials shimfreqnode $scanF "/robot_0/base_scan1" "/robot_0/base_scan" "scan" > "nav2d_shim_logs_${ename}.out" 2> "nav2d_shim_logs_${ename}.err" &
	sleep 2s
        
        # launch the robot itself: [merged 3 nodes into one launch for now.]
        robofname="nav2d_robot_logs_${ename}.err"
        taskset -a -c 1-2 roslaunch nav2d_tutorials tutorial4_robot.launch 2> "nav2d_robot_logs_OpeMap_${ename}.err" & #> "nav2d_robot_logs_${ename}.out" 2> $robofname &
        sleep 2s
	# taskset -a -c 1 chrt -f 1 roslaunch nav2d_tutorials tutorial4_robot2.launch & # has the getmap, explore clients.
	
	# rosparam load ~/catkin_ws/src/navigation_2d/nav2d_tutorials/param/operator.yaml Operator
        # rosparam load ~/catkin_ws/src/navigation_2d/nav2d_tutorials/param/costmap.yaml Operator/local_map
        # taskset -a -c 0-4 rosrun nav2d_operator operator __name:=Operator scan:=base_scan > "robot_nav2d_operator_logs_${ename}.out" 2> "robot_nav2d_operator_logs_${ename}.err" &
        
        # rosparam load ~/catkin_ws/src/navigation_2d/nav2d_tutorials/param/mapper.yaml Mapper
        # taskset -a -c 0-4 rosrun nav2d_karto mapper __name:=Mapper scan:=base_scan > "robot_nav2d_mapper_logs_${ename}.out" 2> "robot_nav2d_mapper_logs_${ename}.err" &

        # rosparam load ~/catkin_ws/src/navigation_2d/nav2d_tutorials/param/navigator.yaml Navigator
        # nfname="robot_nav2d_navigator_logs_${ename}.err"
        # taskset -a -c 0-4 rosrun nav2d_navigator navigator __name:=Navigator > "robot_nav2d_navigator_logs_${ename}.out" 2> $nfname &

        # taskset -a -c 5-7 rosrun nav2d_navigator get_map_client __name:=GetMap &
        # taskset -a -c 5-7 rosrun nav2d_navigator explore_client __name:=Explore &

        sleep 5s

	taskset -a -c 1-2 roslaunch nav2d_tutorials tutorial4_robot2.launch 2> $robofname &
        sleep 9s

	echo "SENDING Startmapping goal"
        # call mapping, then explore.
        rosservice call /robot_0/StartMapping

        # start moving obstacles:
        # params: freq to publish vel, vel, distance_to_be_travelled/speedup
	taskset -a -c 5-6 python src/rbx/src/move_dynamic_obstacles_nav2d.py 200 0.2 0.9  > "nav2d_moveObst_logs_${ename}.txt" &
        
	failct="0"
	success="0"
	iter=0
	while [ $success -lt 1 ]
	do
		success=`grep "MAPPING Successful." $robofname | wc -l`
		if [ $success -lt 1 ]; then
			fails=`grep "MAPPING Failed." $robofname | wc -l`
			if [ $failct -lt $fails ]; then
				echo "CALLING Startmapping again cuz last one failed", $failct, $fails
				rosservice call /robot_0/StartMapping
			fi
			failct=`grep "MAPPING Failed." $robofname | wc -l`
		fi
		# give up after 3 Startmapping tries.
		if [ $failct -gt 2 ]; then
			echo "MAPPING DIDNT WORK EVEN AFTER 3 TRIES!!! For ", $ename, $td, $ccF, $mcbF, $muF, $navcF, $navpF
			success="1"
		fi
		sleep 2s
		echo "iter: ", $iter
		iter=$((iter+1))
		echo $iter, $failct, $success
	done

	sleep 20s
	echo "SENDING StartExpl goal"
        rosservice call /robot_0/StartExploration
        sleep 2s

        # measuring cpu:
        taskset -a -c 5-6 python measure_cpu.py 0 0 40 $ename $robofname &
        
        # script to measure robot, obstacle positions :
        # NO NEED, Incorporated into stageros.cpp itself.

        echo "STARTED everything. NOW waiting for Exploration to FINISH/FAIL."
        j="0"
        t=0
        while [ $j -lt 2 ]
        do
            sleep 5s
            j=`grep "Exploration has finished." $robofname | wc -l`
            a="${j}_$t"
            if [ $j -lt 2 ]; then
                j=`grep "Exploration has failed." $robofname | wc -l`
            fi
	    #Removing "Is the robot out of the map" check cuz this can be mitigated for startMapping:
            if [ $j -lt 2 ]; then
                j=`grep "Exploration failed" $robofname | wc -l`
                j=$((2*j))
		echo "j is still<2, Checked for Exploration failed. j: ", $j
            fi
            echo $a
            t=$((t+1))
            #Loop upto 400 times. Then just exit.
            timelimit=240 # divide timelimit=800s by sleeptime=5s.
	    if [ $t = $timelimit ]; then
                j=2
                echo "EXPLORATION DIDNT FINISH EVEN IN 240*5sec!!!"
            fi
        done
        sleep 5s
        echo "SAVING map to file!!"
        rosrun map_server map_saver -f $ename map:=/robot_0/map
        sleep 5s
        echo "Killing all procs now!"
        for pname in measure_cpu dag_controller operator navigator mapper joy_node shimfreqnode rviz remote_joy stage get_map_client explore_client set_goal_client move_dynamic_obstacles_nav2d rosout rosmaster
        do
            echo "Killing_$pname"
            kill -15 $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') #| xargs kill -15
        done
        kill -9 $(ps -ef | grep "move_dynamic_obstacles_nav2d" | grep -v grep | awk '{print $2}')
        sleep 7s
    done
done
