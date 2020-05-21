# BEFORE RUNNING :
# CHANGE x
# CHANGE fname in gz_ros_cam_utils
# CHECK sleep line 31
# CHECK sleep lines 60
# CHECK param of preprocess
# DynAlgo : rm rostopic, measure_actual_freq, gz_ros_cam_util TD, objtracker 0 1, rbx_perf 1
# RTC : rm rostopic & measure_actual_freq & controller, gz_ros_cam_util ED, objtracker 1 0, rbx_perf 0 
bot=waffle_pi
export TURTLEBOT3_MODEL=$bot
echo $TURTLEBOT3_MODEL
export DISPLAY=:0
x=Expt1_1c_Dyn
# SBx : setting to x*10^5
source ~/.bashrc
#source /home/aditi/local/share/gazebo-9/setup.sh
source devel/setup.bash
catkin_make
echo 0 | sudo tee /sys/devices/system/cpu/cpu15/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu14/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu13/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu12/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu11/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu10/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu9/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu8/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu7/online
echo 1 | sudo tee /sys/devices/system/cpu/cpu6/online
echo 1 | sudo tee /sys/devices/system/cpu/cpu5/online
echo 1 | sudo tee /sys/devices/system/cpu/cpu4/online
echo 1 | sudo tee /sys/devices/system/cpu/cpu3/online
echo 1 | sudo tee /sys/devices/system/cpu/cpu2/online
echo 1 | sudo tee /sys/devices/system/cpu/cpu1/online
echo 1 | sudo tee /sys/devices/system/cpu/cpu0/online
# First experiment with gazebo src and verify behavior at high freq
# Done Then change queue length to 1 and check what happens.
for r in 1 #2 3 4 5 6 7 8 9 10 
do
    for freq in 18 #15 17 20 23 26 28 30 32 33 35 40 60 80 100 #12 28 44 60 80 100 140 
    do
	for t in 4
	do
	    python change_obj_speed.py $t 1 #1 denotes walking, 2 denotes jumping [very fast walk]
            python change_cam_Freq.py $freq $bot
            source devel/setup.bash
            taskset -a -c 0-4 roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch &
            sleep 15s
            # NEED to measure ACTUAL frequency!!
            cam_freq="${x}_actual_freq_$r.$freq.$t.out"
            echo "Starting rostopic"
            echo $cam_freq
            #taskset -a -c 5 rostopic hz /camera/rgb/image_raw > $cam_freq &
            #sleep 10s
            sleep 2s
            prep_out="${x}_preprocess_node_$r.$freq.$t.out"
            prep_err="${x}_preprocess_node_$r.$freq.$t.err"
            lat="${x}_preprocess_lat_$r.$freq.$t.txt"
            echo $prep_out
            # taskset -c 7 python src/rbx1/rbx1_vision/src/rbx1_vision/preprocess_img.py 2300 $lat > $prep_out 2> $prep_err & # 1200 -> 9ms, 1500 -> 13ms.
            taskset -a -c 6 rosrun beginner_tutorials subscriber 960000 1 $freq 8500 1 0 1 1 7700 listener chatter meh 1 > $prep_out 2> $prep_err &
            sleep 2s
            vis_out="${x}_vision_node_$r.$freq.$t.out"
            vis_err="${x}_vision_node_$r.$freq.$t.err"
            echo $vis_out
            # taskset -c 6 python src/rbx1/rbx1_vision/src/rbx1_vision/face_detector.py > $vis_out 2> $vis_err &
            # taskset -a -c 6 python src/rbx1/rbx1_vision/src/rbx1_vision/obj_detector.py 2100 > $vis_out 2> $vis_err &
            taskset -a -c 6 rosrun beginner_tutorials objdetector 1 1 8000 obj_detector 1 1 12700 > $vis_out 2> $vis_err &
            sleep 2s
            track_out="${x}_tracker_node_$r.$freq.$t.out"
            track_err="${x}_tracker_node_$r.$freq.$t.err"
            echo $track_out
            # taskset -a -c 6 python src/rbx1/rbx1_apps/nodes/object_tracker2.py /camera/rgb 1 > $track_out 2> $track_err &
            taskset -a -c 6 rosrun beginner_tutorials objtracker 3.2 0 1 > $track_out 2> $track_err &
	    taskset -a -c 5 rosrun beginner_tutorials controller 1 3 0.1 1.0 /camera/rgb/image_raw_cb_time /camera1/rgb/image_raw_cb_time /roi_cb_time & 
            sleep 5s
	    perf_file="${x}_perf_$r.$freq.$t.out"
            echo $perf_file
            #eval "$(conda shell.bash hook)"
            #conda activate py2_opencv
            echo "Killing rostopic"
            kill -INT $(ps -ef | grep rostopic | grep -v grep | awk '{print $2}')
	    taskset -c 5 python src/rbx/src/measure_rbx_perf.py $perf_file &
            tim=$((9500/$freq))
            opt=23 #CHANGE THIS BASED ON ci !!!!!
            if [ $freq -lt $opt ]
            then
                echo "Sleeping for 10500/freq"
                taskset -c 5 python measure_cpu_objtrack.py $t $freq $tim $x $r & 
	        sleep $tim
            else
                ti=500 #600
	        taskset -c 5 python measure_cpu_objtrack.py $t $freq $ti $x $r &
                sleep $ti
            fi
        
	    #conda deactivate
            sleep 5s
            for pname in measure_rbx_perf obj_detector subscriber objtracker controller object_tracker2 face_detector preprocess_img turtlebot3 gzclient rostopic rosmaster rosout gzserver gazebo
            do
                echo "Killing_$pname"
                kill -INT $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') #| xargs kill -15
                sleep 1s
            done
            #taskset -a -c 5 python measure_actual_freq.py $cam_freq $freq $t $x
	    sed -i '/Found /d' $prep_out
	    sed -i '/Found /d' $vis_out
            sleep 5s
        done
    done
done
#70 Freq pe 60Hz, 100 pe 80Hz, 200 pe 136Hz, 90 pe 70Hz, 250 pe 188Hz
# for t in 7
# do
#     for freq in 10 100 200 #10 20 50 80 100 120 200
#     do
#         python change_obj_speed.py $t 1 #1 denotes walking, 2 denotes jumping [very fast walk]
#         python change_cam_Freq.py $freq $bot
#         source devel/setup.bash
#         taskset -c 0-3 roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch &
#         sleep 5s
#         eval "$(conda shell.bash hook)"
#         conda activate py2_opencv
#         vis_out="jump_vision_node_$freq$t.out"
#         vis_err="jump_vision_node_$freq$t.err"
#         taskset -c 5 python src/rbx1/rbx1_vision/src/rbx1_vision/obj_detector.py > $vis_out 2> $vis_err &
#         sleep 5s
#         track_out="jump_tracker_node_$freq$t.out"
#         track_err="jump_tracker_node_$freq$t.err"
#         taskset -c 6 python src/rbx1/rbx1_apps/nodes/object_tracker2.py /camera/rgb > $track_out 2> $track_err &
#         sleep 240s
#         conda deactivate
#         for pname in obj_detector object_tracker2 turtlebot3 gzclient rosmaster rosout gzserver gazebo
#         do
#             echo "Killing_$pname"
#             kill -INT $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') #| xargs kill -15
#             sleep 1s
#         done
#         sleep 10s
#     done
# done
