# Run this in sudo to allow dag_scheduler to set the right prio, FIFO.
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
source devel/setup.bash
sleep 2s
ct=7
td="yessyessy"
fifo="yes_1c"
for navpF in 1.18343 #5.0 #1.0 0.2
do
	for mcbF in 1.1607 #10.0 7.0 4.0 1.0 0.4 0.16 
	do
		for muF in 1.0394 #10.0 #5.0 1.0 0.4
		do
			for navcF in 6.5876 #20.0 #10.0 5.0 1.0
			do
				for ccF in 19.312476 #20.0 #20.0 #10.0 5.0 2.5 1.0 
				do
					#check machine id, run this only if ct%mccount == mcid
					#div=$((ct%mccount))
					#echo "CHECKING division, ct: ", $ct, " mcid: ", $mcid, " ct%mccount: ", $div 
					div=0
					mcid=0
					# FIFO:Davare et al - Need to set LC, LP nodes' periods as well.
					# Davare V1 - 1c:
					lcF=5.77
					lpF=12.73561
					if [ $div -eq $mcid ]; then
						#echo "WILL run this expt cuz div=mcID!!!!"
						for run in 8 9 10 #1 2 3 4 #4 #5 6 7 
						do
							rm ../robot_nav2d_obstacleDist_logs_.txt
							taskset -a -c 7-12 roslaunch nav2d_tutorials tutorial4_stage.launch &
							sleep 7s
							ename="DF_V1_1c_run$run"
							
							echo "DELETING OLD LOGFILES For this expt:"
							rm "../robot_nav2d_obstacleDist_logs_${ename}.txt"
							rm "../robot_nav2d_${ename}_rt_stats.txt"
							for thing in local_map navigator_plan navigator_cmd mapper_mapUpdate mapper_scanCB operator_loop
							do
								rm "../robot_nav2d_${thing}_stats_${ename}.txt"
							done
							echo "DELETEC OLD LOGFILES for this expt."
							
							echo "ALL Params: ", $ename, $td, $ccF, $mcbF, $muF, $navcF, $navpF, $lcF, $lpF
							python change_nav2d_freqs.py $ccF $mcbF $muF $navcF $navpF $lcF $lpF
							rosparam set /expt_name $ename
							rosparam set /use_td $td	
							sleep 4s

							taskset -a -c 0 chrt -f 8 rosrun beginner_tutorials shimfreqnode $ccF "/robot_0/base_scan1" "/robot_0/base_scan" "scan" > "nav2d_shim_logs_${ename}.out" 2> "nav2d_shim_logs_${ename}.err" &
							sleep 2s
							taskset -a -c 6 chrt -f 8 rosrun beginner_tutorials dag_controller "nav2d" $td $fifo 1 2 5 3 4 6 7  & # if td&fifo, inputs are priorities in order mc,mu,nc,np, s,lc,lp.
							
							# Start evt with prio=1, in any core.
							sleep 2s
							opeMapFname="nav2d_robot_logs_OpeMap_${ename}.err"
							robofname="nav2d_robot_logs_${ename}.err"
							taskset -a -c 0 chrt -f 8 roslaunch nav2d_tutorials tutorial4_robot.launch 2> $opeMapFname &
							sleep 4s
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

							sleep 20s
							echo "StartMapping done, ABOUT TO CALL StartExploration!"
							rosservice call /robot_0/StartExploration
							sleep 2s
							
							taskset -a -c 6-7 python measure_cpu.py 0 0 40 $ename $robofname &
							echo "STARTED everything. NOW waiting for Exploration to FINISH/FAIL."
							j="0"
							t=0
							while [ $j -lt 2 ]
							do
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
								sleep 5s
								echo "j&t: ", $j, $t
								timelimit=240 # divide timelimit=800s by sleeptime=5s.
								if [ $t = $timelimit ]; then
									j=2
									echo "!!!!!~~~%%% EXPLORATION DIDNT FINISH EVEN IN ", $timelimit, "For: ", $ename, $td, $ccF, $mcbF, $muF, $navcF, $navpF
								fi
							done
							sleep 10s
							echo "SAVING map to file!!"
							rosrun map_server map_saver -f $ename map:=/robot_0/map &
							sleep 10s
							echo "Killing all procs now!"
							for pname in measure_cpu dag_controller operator map_saver navigator mapper joy_node shimfreqnode rviz remote_joy stage get_map_client explore_client set_goal_client move_dynamic_obstacles_nav2d
							do
							    echo "Killing_$pname"
							    kill -15 $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') #| xargs kill -15
							done
							kill -9 $(ps -ef | grep "move_dynamic_obstacles_nav2d" | grep -v grep | awk '{print $2}')
							sleep 10s
							rosclean purge -y
						done					
					fi
					# increment global counter in the end : Remains same for runs' loop.
					ct=$((ct+1))
				done
			done
		done
	done
done
