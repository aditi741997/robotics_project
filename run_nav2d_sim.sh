source devel/setup.bash
# nohup roscore &
sleep 2s
for scanF in 10
do
    for r in 11 #6 7 8 #3 4 5
    do
        # launch stage simulator:
        taskset -a -c 3-7 roslaunch nav2d_tutorials tutorial4_stage.launch &
        sleep 2s
        # taskset -a -c 5-7 rosparam load ~/catkin_ws/src/navigation_2d/nav2d_tutorials/param/ros.yaml
        ename="Oct6_Default_1c_Speed10_scanF${scanF}_run$r"
        rosparam set /expt_name $ename
        rosparam set /pub_tid_topic "/exec_thread_id"
        sleep 2s

        # shim node to control frequency of laser scans:
        taskset -a -c 5-7 rosrun beginner_tutorials shimfreqnode $scanF "/robot_0/base_scan" "/robot_0/base_scan1" "scan" &
        
        # launch mapper for obstacles : useless :p
        # taskset -a -c 5-7 roslaunch nav2d_tutorials tutorial4_obstacles.launch &

        # taskset -a -c 5-7 rosrun stage_ros stageros ~/catkin_ws/src/navigation_2d/nav2d_tutorials/world/tutorial.world __name:=Stage _base_watchdog_timeout:=0 > "robot_nav2d_stage_logs_${ename}.out" 2> "robot_nav2d_stage_logs_${ename}.err" & #base_watchdog_timeout ??
        
        # launch the robot itself: [merged 3 nodes into one launch for now.]
        robofname="nav2d_robot_logs_${ename}.err"
        taskset -a -c 1 roslaunch nav2d_tutorials tutorial4_robot.launch > "nav2d_robot_logs_${ename}.out" 2> $robofname &
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
        # taskset -a -c 5-7 rosrun nav2d_navigator set_goal_client __name:=SetGoal &

        # taskset -a -c 5-7 rosrun joy joy_node __name:=Joystick &
        # taskset -a -c 5-7 rosrun nav2d_remote remote_joy __name:=Remote &

        # taskset -a -c 5-7 rosrun rviz rviz __name:=RVIZ -d ~/catkin_ws/src/navigation_2d/nav2d_tutorials/param/tutorial3.rviz > "robot_nav2d_rviz_logs_${ename}.out" 2> "robot_nav2d_rviz_logs_${ename}.err" &
        taskset -a -c 5-7 roslaunch nav2d_tutorials tutorial4_rviz.launch &

        sleep 4s

        # call mapping, then explore.
        rosservice call /robot_0/StartMapping
        sleep 5s

        # start moving obstacles:
        # params: freq to publish vel, vel, distance_to_be_travelled/speedup
        # WORKS!50, 0.5, 3 works FINE for speedup=1. For speedup=10 :
        # r1 50, 0.5, 0.3 : robots move in a small distance
        # r2 50, 0.5, 1.0 : robots more in weird direction?
        # r3 100, 0.5, 1.0 : too fast and too much dist []
        # r4 100, 0.2, 1.0 : speed okay but dist too much
        # r5,6,7 100, 0.2, 0.5,r8:0.4 : speed okay, distance okay but skewed?
        # WORKS!r1,2 speed5 : 100, 0.1, 0.6, roboSpeed: 1
        # r8,9 speed10 : 200, 0.05, 0.3, roboSpeed: 0.5
        # r10 speed10 : 200, 0.06, 0.3, roboSpeed: 0.5
        taskset -a -c 5-7 python src/rbx/src/move_dynamic_obstacles_nav2d.py 200 0.06 0.3  > "nav2d_moveObst_logs_${ename}.txt" &
        sleep 20s
        
        rosservice call /robot_0/StartExploration
        sleep 2s

        # measuring cpu:
        taskset -a -c 5-7 python measure_cpu.py 0 0 40 $ename $robofname &
        
        # script to measure robot, obstacle positions :
        # NO NEED, Incorporated into stageros.cpp itself.

        echo "STARTED everything. NOW waiting for Exploration to FINISH/FAIL."
        j="0"
        t=0
        while [ $j -lt 2 ]
        do
            j=`grep "Exploration has finished." $robofname | wc -l`
            a="${j}_$t"
            # check for 'Is the robot out of map', 'exploration has failed', 'navigator is busy' :
            if [ $j -lt 2 ]; then
                j=`grep "Exploration has failed." $robofname | wc -l`
            fi
            if [ $j -lt 2 ]; then
                j=`grep "Is the robot out of the map" $robofname | wc -l`
                j=$((j+1))
            fi
            if [ $j -lt 2 ]; then
                j=`grep "Navigator is busy!" $robofname | wc -l`
                j=$((j+1))
            fi
            echo $a
            t=$((t+1))
            sleep 1s
            #Loop upto 400 times. Then just exit.
            if [ $t = 400 ]; then
                j=2
                echo "EXPLORATION DIDNT FINISH EVEN IN 400sec!!!"
            fi
        done
        sleep 10s
        echo "SAVING map to file!!"
        rosrun map_server map_saver -f $ename map:=/robot_0/map
        sleep 10s
        echo "Killing all procs now!"
        for pname in operator navigator mapper joy_node shimfreqnode rviz remote_joy stage get_map_client explore_client set_goal_client move_dynamic_obstacles_nav2d
        do
            echo "Killing_$pname"
            kill -15 $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') #| xargs kill -15
        done
        kill -9 $(ps -ef | grep "move_dynamic_obstacles_nav2d" | grep -v grep | awk '{print $2}')
        sleep 10s
    done
done