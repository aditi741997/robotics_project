#!/bin/bash
#source /opt/ros/melodic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

echo 0 | sudo tee /sys/devices/system/cpu/cpu7/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu6/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu5/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu4/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu3/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu2/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu1/online

t=$1 # fixed time of 200 sec.
p=$2
# 
for msg_sz in 2097152 #32768 524288 #5242880 #256 16384 262144 524288 1048576 2097152 # bytes 256,16KB,256KB,512KB,1MB, 2MB
do
    for pub_q in 1 #1 
    do
        for sub_q in 1
        do
            for sub_heavy in 1 #0
            do
                for rate in 50 #8 11 13 15 18 24 30 40 50 #23 27 30 40 50 #80 90 110 130 150 170 190 200 #1 10 50 100 250 500 750 1000 #1 10 1250 1500
                do
                    #1P1S UDS SAME M/C:
                    #gnome-terminal -x sh -c "rosrun test_rosuds listener_uds $msg_sz $pub_q $rate 100 $sub_q 1"
                    #rosrun test_rosuds talker_uds $msg_sz $pub_q $rate 100

                    #NODELET SAME M/C:
                    # num_msg=$(($t*$rate))
                    # nohup rosrun nodelet nodelet manager __name:=nodelet_manager &
                    # nohup rosrun nodelet nodelet load nodelet_tutorial_math/PlusSub nodelet_manager __name:=subnode subnode/chatter_nodelet_in:=pubnode/chatter_nodelet_out _num_msgs:=$(($num_msg-1)) _pub_queue_len:=$pub_q _sub_queue_len:=$sub_q _ros_rate:=$rate _msg_size:=$msg_sz _do_heavy:=$sub_heavy &
                    # nohup python measure.py $t $msg_sz $pub_q $sub_q $sub_heavy $rate 3 1 &
                    # #You need to publish a single message on /pubnode/in manually after starting this script.
                    # rosrun nodelet nodelet load nodelet_tutorial_math/PlusPub nodelet_manager __name:=pubnode _num_msgs:=$num_msg _pub_queue_len:=$pub_q _sub_queue_len:=$sub_q _ros_rate:=$rate _msg_size:=$msg_sz
                    # sleep 1s

                    #1P1S SAME M/C:
                    for trans_type in 1 #2 Not doing UDP for now
                    do
                        num_msg=$(($t*$rate))
                        echo "SUBSCRIBER : $msg_sz $pub_q $rate $num_msg $sub_q $trans_type $sub_heavy"
                    #     #gnome-terminal -x sh -c "rosrun nw_experiments listener $msg_sz $pub_q $rate 100 $sub_q $trans_type 1"
                        # update the limit for sieve
                        fname="listener_$p$rate$msg_sz.out"
                        fname_err="listener_$p$rate$msg_sz.err"
                        echo $fname
			fname1="listener1_$p$rate$msg_sz.out"
                        fname1_err="listener1_$p$rate$msg_sz.err"

                        #rosrun beginner_tutorials subscriber $msg_sz $pub_q $rate $(($num_msg-1)) $sub_q $trans_type 1 $sub_heavy 17000 listener1 chatter1 $p 0 > $fname1 2> $fname1_err &
			rosrun beginner_tutorials subscriber $msg_sz $pub_q $rate $(($num_msg-1)) $sub_q $trans_type 1 $sub_heavy 11000 listener chatter $p 0 chatter1 > $fname 2> $fname_err & #taskset 2 #nice --1 for priority more than pub. 
                        #nohup python measure.py $t $msg_sz $pub_q $sub_q $sub_heavy $rate $trans_type 0 &
			pname="PUB_$p.txt"
			#pname1="PUB1_$p.txt"
			#rosrun beginner_tutorials publisher $msg_sz $pub_q $rate $num_msg 8000 $pname1 1 talker1 > 2> & 
                        rosrun beginner_tutorials publisher $msg_sz $pub_q $rate $num_msg 8100 $pname 1 talker chatter #taskset 0x00000001 proc_time_param, file name for stats, #subs 
                        sleep 5s
                        #rm $fname
			#rm $fname1
			python clean_logs.py $fname 1 "noloop"
			python clean_logs.py $fname1 1 "noloop"
		    done

                    #1P1S SAME M/C UDP: Tried this because UDP isn't working when listener node name is changed
                    # echo "SUBSCRIBER : $msg_sz $pub_q $rate 1000 $sub_q $sub_heavy"
                    # nohup rosrun beginner_tutorials listener $msg_sz $pub_q $rate 999 $sub_q 2 1 $sub_heavy &
                    # rosrun beginner_tutorials mytalker $msg_sz $pub_q $rate 1000
                    # rosnode kill -a
                    # sleep 1s
                done
            done
        done
    done
done
