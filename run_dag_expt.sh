catkin_make
source devel/setup.bash
x="DAG_Fra_D"
echo 0 | sudo tee /sys/devices/system/cpu/cpu7/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu6/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu5/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu4/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu3/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu2/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu1/online
echo 1 | sudo tee /sys/devices/system/cpu/cpu0/online
# different f's to be used for testing on some fixed schedule
for f in 7.1 #20 30
do
	for r in 1 #2
	do
		nummsg=1000
		scan_out="${x}_scan_$f.$r.out"	
		chrt -d -T 2000000 -D 2000000 -P 140000000  rosrun beginner_tutorials publisher 25000 1 $f $nummsg 1 2 scanpub scan > $scan_out & # lcmp, gcmp subscribe
		odom_out="${x}_odom_$f.$r.out"
		chrt -d -T 2000000 -D 4000000 -P 140000000 rosrun beginner_tutorials publisher 6000 1 $f $nummsg 1 1 odompub odom > $odom_out & # lplan subscribes
		lcmp_out="${x}_lcmp_$f.$r.out"
		chrt -d -T 12000000 -D 16000000 -P 140000000 taskset -a -c 0 rosrun beginner_tutorials dsubscriber 1000 1 $f $nummsg 1 0 1 8500 localcmp scan Test 1 lcmp > $lcmp_out & # local costmap subs to scan and pubs local costmap
		gcmp_out="${x}_gcmp_$f.$r.out"
		chrt -d -T 17000000 -D 140000000 -P 280000000 rosrun beginner_tutorials dsubscriber 1000 1 $f $nummsg 1 0 1 10300 globalcmp scan Test 1 gcmp > $gcmp_out & # global cmp subs to scan and pubs global costmap
		gplan_out="${x}_gplan_$f.$r.out"
		chrt -d -T 202000000 -D 280000000 -P 280000000 rosrun beginner_tutorials dsubscriber 1000 1 $f $nummsg 1 0 1 39300 globalplanner gcmp Test 1 gplan > $gplan_out & # global planner subs to gcostmap and pubs global plan
		chrt -d -T 19000000 -D 35000000 -P 140000000 rosrun beginner_tutorials localplanner 11000 0 lcmp odom gplan &
		sleep 90s
		for pname in dsubscriber publisher localplanner
		do
			echo "Killing_$pname"
			kill -KILL $(ps -ef | grep $pname | grep -v grep | awk '{print $2}')
			sleep 1s
		done
		sed -i '/FOUND /d' $scan_out
		sed -i '/FOUND /d' $odom_out
		sed -i '/Found /d' $lcmp_out
		sed -i '/Found /d' $gcmp_out
		sed -i '/Found /d' $gplan_out
	done
done
