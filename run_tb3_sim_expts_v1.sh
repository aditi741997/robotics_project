bot=burger
export TURTLEBOT3_MODEL=$bot
echo $TURTLEBOT3_MODEL
# echo 0 | sudo tee /sys/devices/system/cpu/cpu7/online
# echo 0 | sudo tee /sys/devices/system/cpu/cpu6/online
# echo 0 | sudo tee /sys/devices/system/cpu/cpu5/online
# echo 0 | sudo tee /sys/devices/system/cpu/cpu4/online
# echo 0 | sudo tee /sys/devices/system/cpu/cpu3/online
# echo 0 | sudo tee /sys/devices/system/cpu/cpu2/online

for scan_freq in 50 #5 10 30 70 30 40 50 70 90 #100 250 500 750 1000 #5  
do
    for num_samples in 1800 #3600 360 1800 7200 #360 1800 3600 7200 #1440 1800 3600 #360 1440 1080 1800 720 3600
    do
        for run in 9 #8 10 1 2 3 4 5 6 7
        do
            nfname="tb3_navgn_2c_R1_logs_$scan_freq$num_samples$run.out"
            gfname="tb3_gz_2c_R1_logs_$scan_freq$num_samples$run.out"
            gferr="tb3_gz_2c_R1_errlogs_$scan_freq$num_samples$run.out"
            nferr="tb3_navgn_2c_R1_errlogs_$scan_freq$num_samples$run.out"
            python vary_pub_freq.py $scan_freq $num_samples ~/catkin_ws/src/turtlebot3/turtlebot3_description/urdf turtlebot3_burger.gazebo.xacro ~/catkin_ws/src/turtlebot3/turtlebot3_navigation/param/move_base_params.yaml ~/catkin_ws/src/turtlebot3/turtlebot3_navigation/param/costmap_common_params_$bot.yaml ~/catkin_ws/src/turtlebot3/turtlebot3_navigation/param/ #src/turtlebot3/turtlebot3_navigation/param/move_base_params.yaml 
            nohup roslaunch turtlebot3_world_no_gui.launch > $gfname 2> $gferr &
            sleep 2s
            nohup roslaunch turtlebot3_navigation_without_rviz.launch map_file:=$HOME/mapFull.yaml > $nfname 2> $nferr &
            sleep 9s
			echo "Publishing initial position"
			nohup rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "header:
  seq: 2
  stamp:
    secs: 197
    nsecs: 663000000
  frame_id: 'map'
pose:
  pose:
    position:
      x: -1.93499934673
      y: -0.499999761581
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.0111091174028
      w: 0.999938291851
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]" &
			sleep 9s
            echo "Publishing first goal"
            nohup rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: -1.29
    y: -1.4
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" &
            sleep 5s
            python measure.py 20 firstgoal $scan_freq $num_samples $nfname 1 149 'yes' '_2c_R1'
            sleep 2s
            ps -ef | grep move_base_simple | grep -v grep | awk '{print $2}' | xargs kill
            i="0"
            while [ $i -lt 1 ]
            do
                i=`grep "REACHED" $nfname | wc -l`
                echo $i
                sleep 1s
            done
            #Second goal now!
            sleep 1s
            echo "Publishing second goal now"
            nohup rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 1.49
    y: 1.55
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" &
            python measure.py 25 secondgoal $scan_freq $num_samples $nfname 2 149 'yes' '_2c_R1'  #last should be empty if no suffix in cpu time series fname, 2nd last should be yes if want time series.
            sleep 9s
            j="0"
            t=0
            while [ $j -lt 2 ]
            do
                j=`grep "REACHED" $nfname | wc -l`
                echo $j
                t=$((t+1))
                sleep 1s
                #Loop upto 300 times. Then just exit.
                if [ $t = 300 ]; then
                    j=2
                fi
            done
            echo "Killing all procs now!"
            for pname in move_base_simple move_base rosout rosmaster gzserver amcl map_server robot_state_publisher gazebo navigation
            do
                echo "Killing_$pname"
                kill -15 $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') #| xargs kill -15
            done
            echo "\n" >> $HOME/.ros/local_costmap_stats.txt
            echo "\n" >> $HOME/.ros/global_costmap_stats.txt
        done
    done
done
