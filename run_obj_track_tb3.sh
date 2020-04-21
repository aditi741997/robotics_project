bot=waffle_pi
export TURTLEBOT3_MODEL=$bot
echo $TURTLEBOT3_MODEL
x=T
# SBx : setting to x*10^5
source ~/.bashrc
source /home/aditi/local/share/gazebo-9/setup.sh
source devel/setup.bash
catkin_make
# First experiment with gazebo src and verify behavior at high freq
# Done Then change queue length to 1 and check what happens.
# for t in 8 #2 #24 20 16 12 8 4 2
for t in 8
do
    for freq in 12 #30 40 50 60 70 100
    do
        # python change_obj_speed.py $t 1 #1 denotes walking, 2 denotes jumping [very fast walk]
        python change_cam_Freq.py $freq $bot
        source devel/setup.bash
        taskset -c 0-3 roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch &
        sleep 15s
        # NEED to measure ACTUAL frequency!!
        cam_freq="${x}_actual_freq_$freq$t.out"
        echo "Starting rostopic"
        echo $cam_freq
        taskset -c 4 rostopic hz /camera/rgb/image_raw > $cam_freq &
        sleep 60s
        echo "Killing rostopic"
        kill -INT $(ps -ef | grep rostopic | grep -v grep | awk '{print $2}')
        sleep 2s
        taskset -c 4 python measure_actual_freq.py $cam_freq $freq $t $x
        prep_out="${x}_preprocess_node_$freq$t.out"
        prep_err="${x}_preprocess_node_$freq$t.err"
        lat="${x}_preprocess_lat_$freq$t.txt"
        echo $prep_out
        eval "$(conda shell.bash hook)"
        conda activate py2_opencv
        taskset -c 7 python src/rbx1/rbx1_vision/src/rbx1_vision/preprocess_img.py 1300 $lat > $prep_out 2> $prep_err & # 1200 -> 9ms, 1500 -> 13ms.
        # taskset -c 7 rosrun beginner_tutorials subscriber 960000 1 $freq 1000 1 0 1 1 13000 listener chatter meh 0 chatter1 > $prep_out 2> $prep_err &
        sleep 2s
        vis_out="${x}_vision_node_$freq$t.out"
        vis_err="${x}_vision_node_$freq$t.err"
        echo $vis_out
        taskset -c 6 python src/rbx1/rbx1_vision/src/rbx1_vision/face_detector.py > $vis_out 2> $vis_err &
        # taskset -c 6 python src/rbx1/rbx1_vision/src/rbx1_vision/obj_detector.py 1 > $vis_out 2> $vis_err &
        sleep 3s
        track_out="${x}_tracker_node_$freq$t.out"
        track_err="${x}_tracker_node_$freq$t.err"
        echo $track_out
        taskset -c 5 python src/rbx1/rbx1_apps/nodes/object_tracker2.py /camera/rgb > $track_out 2> $track_err &
        # sleep 5s
        # perf_file="${x}_new_perf_$freq$t.out"
        # echo $perf_file
        # taskset -c 4 python src/rbx1/rbx1_apps/nodes/measure_rbx_perf.py $perf_file &
        sleep 300s
        # if [ $freq -lt 31 ]
        # then
        #     echo "Sleeping a bit more"
        #     sleep 100s
        # fi
        # conda deactivate
        sleep 5s
        for pname in obj_detector object_tracker2 face_detector preprocess_img turtlebot3 gzclient rostopic rosmaster rosout gzserver gazebo measure_rbx_perf
        do
            echo "Killing_$pname"
            kill -INT $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') #| xargs kill -15
            sleep 1s
        done
        sleep 10s
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