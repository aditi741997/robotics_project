bot=waffle_pi
export TURTLEBOT3_MODEL=$bot
echo $TURTLEBOT3_MODEL

# change freq at turtlebot3/tb3_description/urdf/tb3_wafflepi.gazebo.xacro, line 165
# the empty world file will need to be modified to change object speed
for t in 8
do
    for freq in 40 #60 150  #10 90 200
    do
        python change_obj_speed.py $t 1 #1 denotes walking, 2 denotes jumping [very fast walk]
        python change_cam_Freq.py $freq $bot
        source devel/setup.bash
        taskset -c 0-3 roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch &
        sleep 5s
        eval "$(conda shell.bash hook)"
        conda activate py2_opencv
        vis_out="max5_new_vision_node_$freq$t.out"
        vis_err="max5_new_vision_node_$freq$t.err"
        taskset -c 5 python src/rbx1/rbx1_vision/src/rbx1_vision/obj_detector.py > $vis_out 2> $vis_err &
        sleep 5s
        track_out="max5_new_tracker_node_$freq$t.out"
        track_err="max5_new_tracker_node_$freq$t.err"
        taskset -c 6 python src/rbx1/rbx1_apps/nodes/object_tracker2.py /camera/rgb > $track_out 2> $track_err &
        sleep 240s
        conda deactivate
        for pname in obj_detector object_tracker2 turtlebot3 gzclient rosmaster rosout gzserver gazebo
        do
            echo "Killing_$pname"
            kill -INT $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') #| xargs kill -15
            sleep 1s
        done
        sleep 10s
    done
done
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