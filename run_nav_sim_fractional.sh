# Run this in sudo to allow dag_scheduler to set the right prio, FIFO.
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
source devel/setup.bash
sleep 2s
ct=7
td="nono"
for navpF in 1.259 #1.056 #5.0 #1.0 0.2
do
	for mcbF in 10.07 #1.107 #10.0 7.0 4.0 1.0 0.4 0.16 
	do
		for muF in 1.259 #1.928 #1.056 #10.0 #5.0 1.0 0.4
		do
			for navcF in 30.23 #9.0 #5.8125 #20.0 #10.0 5.0 1.0
			do
				for ccF in 30.23 #54.0 #23.25 #20.0 #10.0 5.0 2.5 1.0 
				do
					div=0
					mcid=0
					if [ $div -eq $mcid ]; then
						#echo "WILL run this expt cuz div=mcID!!!!"
						for run in 90 #66 67 68 69 #44 45 43 44 #19 20 21 22 23 24 25 26 27 28 29 30 #8 9 10 11 12 13 14 15 16 17 18 #1 2 3 4 5 6 7 #50 #45 46 47 48 49 # # 31 # 1  
						do
							rosclean purge -y
							rm ../robot_nav2d_obstacleDist_logs_.txt
							taskset -a -c 7-12 roslaunch nav2d_tutorials tutorial4_stage_largemap.launch &
							sleep 27s
							ename="DFrac_1c_run$run"
							
							echo "DELETING OLD LOGFILES For this expt:"
							rm "../robot_nav2d_obstacleDist_logs_${ename}.txt"
							rm "../robot_nav2d_${ename}_rt_stats.txt"
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
							taskset -a -c 0 chrt -f 4 rosrun beginner_tutorials dag_controller "dummy_nav2d" $td "no" 1 1 1 1 1 1 1 1 > $dagcontOname 2> $dagcontEname &
							sleep 4s
							taskset -a -c 1 chrt -f 1 rosrun beginner_tutorials shimfreqnode $ccF "/robot_0/base_scan1" "/robot_0/base_scan" "scan" > "nav2d_shim_logs_${ename}.out" 2> "nav2d_shim_logs_${ename}.err" &
							
							# Start evt with prio=1, in any core.
							sleep 2s
							taskset -a -c 0 chrt -f 1 roslaunch nav2d_tutorials tutorial4_robot.launch 2> $opeMapFname &
							sleep 3s
							taskset -a -c 0 chrt -f 1 roslaunch nav2d_tutorials tutorial4_robot2.launch 2> $robofname &
							sleep 12s
							rosservice call /robot_0/StartMapping
							
							taskset -a -c 6-7 python src/rbx/src/move_dynamic_obstacles_nav2d.py 200 0.15 0.9 > "nav2d_moveObst_logs_${ename}.txt" &
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
								sleep 2s
								echo "iter: ", $iter
								iter=$((iter+1))
								echo $iter, $failct, $success
							done

							sleep 10s
							echo "StartMapping done, ABOUT TO CALL StartExploration!"
							rosservice call /robot_0/StartExploration
							sleep 2s
							
							taskset -a -c 6-7 python measure_cpu.py 0 0 40 $ename $robofname &
							echo "STARTED everything. NOW waiting for Exploration to FINISH/FAIL."
							j="0"
							t=0
							while [ $j -lt 2 ]
							do
								sleep 5s
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
								timelimit=160 # divide timelimit=800s by sleeptime=5s.
								if [ $t = $timelimit ]; then
									j=2
									echo "!!!!!~~~%%% EXPLORATION DIDNT FINISH EVEN IN ", $timelimit, "For: ", $ename, $td, $ccF, $mcbF, $muF, $navcF, $navpF
								fi
							done
							sleep 4s
							echo "SAVING map to file!!"
							rosrun map_server map_saver -f $ename map:=/robot_0/map &
							sleep 5s
							echo "Killing all procs now!"
							for pname in measure_cpu dag_controller operator map_saver navigator mapper joy_node shimfreqnode rviz remote_joy stage get_map_client explore_client set_goal_client move_dynamic_obstacles_nav2d
							do
							    echo "Killing_$pname"
							    kill -15 $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') #| xargs kill -15
							done
							kill -9 $(ps -ef | grep "move_dynamic_obstacles_nav2d" | grep -v grep | awk '{print $2}')
							sleep 7s
						done					
					fi
					# increment global counter in the end : Remains same for runs' loop.
					ct=$((ct+1))
				done
			done
		done
	done
done
