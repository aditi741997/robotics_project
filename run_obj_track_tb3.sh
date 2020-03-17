bot=waffle_pi
export TURTLEBOT3_MODEL=$bot
echo $TURTLEBOT3_MODEL
x=3c_max5

# change freq at turtlebot3/tb3_description/urdf/tb3_wafflepi.gazebo.xacro, line 165
# the empty world file will need to be modified to change object speed
for t in 3 #2 #24 20 16 12 8 4 2
do
    for freq in 10 #70 #80 200 #70 80 90 100 200
    do
        python change_obj_speed.py $t 1 #1 denotes walking, 2 denotes jumping [very fast walk]
        python change_cam_Freq.py $freq $bot
        source devel/setup.bash
        taskset -c 0-2 roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch &
        sleep 7s
        # NEED to measure ACTUAL frequency!!
        cam_freq="${x}_actual_freq_$freq$t.out"
        echo "Starting rostopic"
        echo $cam_freq
        rostopic hz /camera/rgb/image_raw > $cam_freq &
        sleep 60s
        echo "Killing rostopic"
        kill -INT $(ps -ef | grep rostopic | grep -v grep | awk '{print $2}')
        sleep 5s
        python measure_actual_freq.py $cam_freq $freq $t $x
        eval "$(conda shell.bash hook)"
        conda activate py2_opencv
        prep_out="${x}_new_preprocess_node_$freq$t.out"
        prep_err="${x}_new_preprocess_node_$freq$t.err"
        echo $prep_out
        taskset -c 5 python src/rbx1/rbx1_vision/src/rbx1_vision/preprocess_img.py 1200 > $prep_out 2> $prep_err &
        sleep 2s
        vis_out="${x}_new_vision_node_$freq$t.out"
        vis_err="${x}_new_vision_node_$freq$t.err"
        echo $vis_out
        taskset -c 6 python src/rbx1/rbx1_vision/src/rbx1_vision/obj_detector.py > $vis_out 2> $vis_err &
        sleep 3s
        track_out="${x}_new_tracker_node_$freq$t.out"
        track_err="${x}_new_tracker_node_$freq$t.err"
        echo $track_out
        taskset -c 7 python src/rbx1/rbx1_apps/nodes/object_tracker2.py /camera/rgb > $track_out 2> $track_err &
        sleep 5s
        perf_file="${x}_new_perf_$freq$t.out"
        echo $perf_file
        taskset -c 4 python src/rbx1/rbx1_apps/nodes/measure_rbx_perf.py $perf_file &
        sleep 250s
        if [ $freq -lt 100 ]
        then
            echo "Sleeping a bit more"
            sleep 100s
        fi
        conda deactivate
        for pname in obj_detector object_tracker2 preprocess_img turtlebot3 gzclient rosmaster rosout gzserver gazebo measure_rbx_perf
        do
            echo "Killing_$pname"
            kill -INT $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') #| xargs kill -15
            sleep 1s
        done
        sleep 10s
    done
done
# 70 Freq pe 60Hz, 100 pe 80Hz, 200 pe 136Hz, 90 pe 70Hz, 250 pe 188Hz
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