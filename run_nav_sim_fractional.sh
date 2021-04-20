# Run this in sudo to allow dag_scheduler to set the right prio, FIFO.
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
source devel/setup.bash
sleep 2s
ct=7
ccid=2
td="yesyess"
for navpF in 100.0 #1.056 #5.0 #1.0 0.2
do
	for mcbF in 100.0 #1.107 #10.0 7.0 4.0 1.0 0.4 0.16 
	do
		for muF in 100.0 #1.928 #1.056 #10.0 #5.0 1.0 0.4
		do
			for navcF in 100.0 #9.0 #5.8125 #20.0 #10.0 5.0 1.0
			do
				for ccF in 100.0 #100.0 50.0 
				do
					div=0
					mcid=0
					if [ $div -eq $mcid ]; then
						#echo "WILL run this expt cuz div=mcID!!!!"
						for run in {18..30} #54 57 58 67 77 {98..101} 
						do
							rosclean purge -y
							rm ../robot_nav2d_obstacleDist_logs_.txt
							taskset -a -c 7-12 roslaunch nav2d_tutorials tutorial4_stage_noobst.launch &
							sleep 27s
							ename="AllHighQNO_1c_run$run"
							
							echo "DELETING OLD LOGFILES For this expt:"
							rm "../robot_nav2d_obstacleDist_logs_${ename}.txt"
							rm "../robot_nav2d_${ename}_rt_stats.txt"
							mapcb_procscans_name="../MCB_ProcScans_${ename}.bag"
							rm $mapcb_procscans_name 
							for thing in local_map navigator_plan navigator_cmd mapper_mapUpdate mapper_scanCB operator_loop
							do
								rm "../robot_nav2d_${thing}_stats_${ename}.txt"
							done
							echo "DELETEC OLD LOGFILES for this expt."
							
							echo "ALL Params: ", $ename, $td, $ccF, $mcbF, $muF, $navcF, $navpF
							python change_nav2d_freqs.py $ccF $mcbF $muF $navcF $navpF
							rosparam set /expt_name $ename
							rosparam set /use_td $td	
							sleep 4s

							opeMapFname="nav2d_robot_logs_OpeMap_${ename}.err"
							robofname="nav2d_robot_logs_${ename}.err"
							dagcontEname="dag_contBE_${ename}.err"
							dagcontOname="dag_contBE_${ename}.out"
							#For slowing down navp: 
							#taskset -a -c 0 chrt -f 4 rosrun beginner_tutorials dag_controller "nav2d_95p_smallest" $td "no" 74 98 1 74 1 1 1 0 > $dagcontOname 2> $dagcontEname &
							taskset -a -c 0 chrt -f 4 rosrun beginner_tutorials dag_controller "nav2d_95p_small" $td "no" 2 76 3 48 1 1 1 0 > $dagcontOname 2> $dagcontEname & 
							sleep 4s
							taskset -a -c 1 rosrun beginner_tutorials shimfreqnode $ccF "/robot_0/base_scan1" "/robot_0/base_scan" "scan" $td > "nav2d_shim_logs_${ename}.out" 2> "nav2d_shim_logs_${ename}.err" &
							
							# Start evt with prio=1, in any core.
							sleep 2s
							taskset -a -c 0 roslaunch nav2d_tutorials tutorial4_robot.launch 2> $opeMapFname &
							sleep 3s
							navRecvMapName="${ename}_navRecv"
							taskset -a -c 15 rosrun map_server map_saver -f $navRecvMapName __name:=navRecvMapNode map:=/robot_0/nav_recv_map &
							navRecvMappingName="${ename}_navRecvMapping"
							taskset -a -c 15 rosrun map_server map_saver -f $navRecvMappingName __name:=navRecvMappingNode map:=/robot_0/nav_recv_mapping &
							
							taskset -a -c 0 roslaunch nav2d_tutorials tutorial4_robot2.launch 2> $robofname &
							sleep 12s
							rosservice call /robot_0/StartMapping
							
							#taskset -a -c 5-6 python src/rbx/src/move_dynamic_obst_special.py 200 0.1 0.9 > "nav2d_moveObst_logs_${ename}.txt" 2> "nav2d_moveObst_logs_${ename}.err" &
							# Before startingExpl, Look for MAPPING Failed / Successful.
							failct="0"
							success="0"
							iter=0
							while [ $success -lt 1 ]
							do
								success=`grep "MAPPING Successful." $robofname | wc -l`
								if [ $success -lt 1 ]; then
									fails=`grep "MAPPING Failed." $robofname | wc -l`
									if [ $failct -lt $fails ]; then
										echo "CALLING Startmapping again cuz last one failed", $failct, $fails
										rosservice call /robot_0/StartMapping
									fi
									failct=`grep "MAPPING Failed." $robofname | wc -l`
								fi
								# give up after 3 Startmapping tries.
								if [ $failct -gt 2 ]; then
									echo "MAPPING DIDNT WORK EVEN AFTER 3 TRIES!!! For ", $ename, $td, $ccF, $mcbF, $muF, $navcF, $navpF
									success="1"
								fi
								if [ $iter -gt 15 ]; then
									success="1"
									echo "MAPPING DIDNT WORK EVEN AFTER iter=", $iter, $ename, $td, $ccF, $mcbF, $muF, $navcF, $navpF
								fi
								sleep 2s
								echo "iter: ", $iter
								iter=$((iter+1))
								echo $iter, $failct, $success
							done

							sleep 10s
							echo "StartMapping done, ABOUT TO CALL StartExploration!"
							rosservice call /robot_0/StartExploration
							sleep 2s
							
							taskset -a -c 5-6 python measure_cpu.py 0 0 40 $ename $robofname &
							echo "STARTED everything. NOW waiting for Exploration to FINISH/FAIL."
							j="0"
							t=0
							while [ $j -lt 2 ]
							do
								mapi="${ename}_i${t}"
								taskset -a -c 14 rosrun map_server map_saver -f $mapi map:=/robot_0/map &
								sleep 20s
								j=`grep "Exploration has finished." $robofname | wc -l`
								if [ $j -lt 2 ]; then
									j=`grep "Exploration has failed." $robofname | wc -l`
								fi
								if [ $j -lt 2 ]; then
									# Exploration failed
									j=`grep "Exploration failed." $robofname | wc -l`
									j=$((2*j))
									echo "j is still<2, Checked for Exploration failed. j: ", $j
								fi
								t=$((t+1))
								# NOTHING should die during the expt!!
								xnav=`grep "process has died" $robofname | wc -l`
								xmap=`grep "process has died" $opeMapFname | wc -l`
								xdag=`grep "process has died" $dagcontEname | wc -l`
								if [ $xnav -gt 0 ] || [ $xmap -gt 0 ] || [ $xdag -gt 0 ];then
									j=2
									echo "Something DIED in SOME node!!", $xnav, $xmap, $xdag
								fi
								echo "j&t: ", $j, $t
								timelimit=16 # divide timelimit=800s by sleeptime=5s.
								if [ $t = $timelimit ]; then
									j=2
									echo "!!!!!~~~%%% EXPLORATION DIDNT FINISH EVEN IN ", $timelimit, "For: ", $ename, $td, $ccF, $mcbF, $muF, $navcF, $navpF
								fi
							done
							sleep 1s
							navRecvMapOName="${ename}_navRecvO"
							taskset -a -c 15 rosrun map_server map_saver -f $navRecvMapOName __name:=navRecvMapONode map:=/robot_0/nav_recv_mapO &
							echo "SAVING map to file!!"
							rosrun map_server map_saver -f $ename map:=/robot_0/map &
							sleep 5s
							echo "Killing all procs now!"
							for pname in measure_cpu dag_controller operator map_saver navigator mapper joy_node shimfreqnode rviz remote_joy stage get_map_client explore_client set_goal_client move_dynamic_obst
							do
							    echo "Killing_$pname"
							    kill -15 $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') #| xargs kill -15
							done
							kill -9 $(ps -ef | grep "move_dynamic_obst" | grep -v grep | awk '{print $2}')
							sleep 7s
							
							# POST PROCESS MAPPER:
							roscore &
							rosparam set /use_td "nono"
							rosrun map_server map_saver -f "${ename}_PP" map:=/robot_0/ppmap __name:=ppnode &
							rosrun nav2d_karto postprocessmapper $opeMapFname $mapcb_procscans_name "../robot_nav2d_obstacleDist_logs_${ename}.txt" 4 0 -2 > "nav2d_PPMap_${ename}.out" 2> "nav2d_PPMap_${ename}.err"
							sleep 2s
							for pname in map_saver roscore mapper
							do
								echo "Killing_$pname"
								kill -15 $(ps -ef | grep $pname | grep -v grep | awk '{print $2}')
							done
							sleep 2s
							rm $mapcb_procscans_name 
						done					
						ccid=$((ccid+1))
					fi
					# increment global counter in the end : Remains same for runs' loop.
					ct=$((ct+1))
				done
			done
		done
	done
done
