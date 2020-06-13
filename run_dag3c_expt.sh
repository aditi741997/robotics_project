catkin_make
source devel/setup.bash
x="DAG3c_S2"
echo 0 | sudo tee /sys/devices/system/cpu/cpu7/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu6/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu5/online
echo 0 | sudo tee /sys/devices/system/cpu/cpu4/online
echo 1 | sudo tee /sys/devices/system/cpu/cpu3/online
echo 1 | sudo tee /sys/devices/system/cpu/cpu2/online
echo 1 | sudo tee /sys/devices/system/cpu/cpu1/online
echo 1 | sudo tee /sys/devices/system/cpu/cpu0/online
for r in 2 3 4 5
do
	nummsg=20000
	f=50
	fg=4.2
	echo "${x} $f.$fg.$r"
        scan_out="${x}_scan_f$f.fg$fg.$r.out"
	taskset -a -c 0-1 rosrun beginner_tutorials publisher 25000 1 $f $nummsg 1 2 scanpub scan > $scan_out &
	odom_out="${x}_odom_f$f.fg$fg.$r.out"
        taskset -a -c 3 rosrun beginner_tutorials publisher 6000 1 $f $nummsg 1 1 odompub odom > $odom_out &
	lcmp_out="${x}_lcmp_f$f.fg$fg.$r.out"
        taskset -a -c 0-1 rosrun beginner_tutorials dsubscriber 1000 1 $f $nummsg 1 0 1 8500 localcmp scan Test 1 lcmp 0 0 > $lcmp_out &
	gcmp_out="${x}_gcmp_f$f.fg$fg.$r.out"
	taskset -a -c 2 rosrun beginner_tutorials globalcmptd 2000 $fg 10300 globalcmptd scan gcmp > $gcmp_out &
	gplan_out="${x}_gplan_f$f.fg$fg.$r.out"
        taskset -a -c 2 rosrun beginner_tutorials dsubscriber 1000 1 $f $nummsg 1 0 1 27300 globalplanner gcmp Test 1 gplan 0 1 > $gplan_out & 
	lplan_out="${x}_lplan_LOGS_f$f.fg$fg.$r.out"
        taskset -a -c 0-1 rosrun beginner_tutorials localplanner 11000 0 lcmp odom gplan > $lplan_out &
	sleep 240s
	for pname in dsubscriber publisher localplanner globalcmptd
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
	sed -i '/Found /d' $lplan_out
done
