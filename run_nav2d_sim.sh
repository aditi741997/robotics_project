source devel/setup.bash
# nohup roscore &
sleep 5s
for nc in 1 #2 3 4 5 6
do
    for r in 1 #2 3 4 5
    do
        # roslaunch nav2d_tutorials stage.launch
        sleep 1s
        rosparam load ~/catkin_ws/src/navigation_2d/nav2d_tutorials/param/ros.yaml
        ename="1c_run$r"
        rosparam set /expt_name ename
        taskset -a -c 5-7 rosrun stage_ros stageros ~/catkin_ws/src/navigation_2d/nav2d_tutorials/world/tutorial.world __name:=Stage _base_watchdog_timeout:=0 & #base_watchdog_timeout ??
        
        rosparam load ~/catkin_ws/src/navigation_2d/nav2d_tutorials/param/operator.yaml Operator
        rosparam load ~/catkin_ws/src/navigation_2d/nav2d_tutorials/param/costmap.yaml Operator/local_map
        taskset -a -c 0 rosrun nav2d_operator operator __name:=Operator scan:=base_scan > "robot_nav2d_operator_logs_${ename}.out" 2> "robot_nav2d_operator_logs_${ename}.err" &
        
        rosparam load ~/catkin_ws/src/navigation_2d/nav2d_tutorials/param/mapper.yaml Mapper
        taskset -a -c 0 rosrun nav2d_karto mapper __name:=Mapper scan:=base_scan > "robot_nav2d_mapper_logs_${ename}.out" 2> "robot_nav2d_mapper_logs_${ename}.err" &

        rosparam load ~/catkin_ws/src/navigation_2d/nav2d_tutorials/param/navigator.yaml Navigator
        nfname="robot_nav2d_navigator_logs_${ename}.err"
        taskset -a -c 0 rosrun nav2d_navigator navigator __name:=Navigator > "robot_nav2d_navigator_logs_${ename}.out" 2> $nfname &

        taskset -a -c 5-7 rosrun nav2d_navigator get_map_client __name:=GetMap &
        taskset -a -c 5-7 rosrun nav2d_navigator explore_client __name:=Explore &
        taskset -a -c 5-7 rosrun nav2d_navigator set_goal_client __name:=SetGoal &

        taskset -a -c 5-7 rosrun joy joy_node __name:=Joystick &
        taskset -a -c 5-7 rosrun nav2d_remote remote_joy __name:=Remote &

        taskset -a -c 5-7 rosrun rviz rviz __name:=RVIZ -d ~/catkin_ws/src/navigation_2d/nav2d_tutorials/param/tutorial3.rviz &

        sleep 4s
        rosservice call /StartMapping
        sleep 10s
        rosservice call /StartExploration
        sleep 2s
        j="0"
        t=0
        while [ $j -lt 2 ]
        do
            j=`grep "Exploration has finished." $nfname | wc -l`
            echo $j
            t=$((t+1))
            sleep 1s
            #Loop upto 500 times. Then just exit.
            if [ $t = 500 ]; then
                j=2
            fi
        done
        echo "Killing all procs now!"
        for pname in operator navigator mapper joy_node rviz remote_joy stage get_map_client explore_client set_goal_client
        do
            echo "Killing_$pname"
            kill -15 $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') #| xargs kill -15
        done
    done
done
