import sys
import matplotlib.pyplot as plt
import numpy as np
import math

# For each metric, we read 3 files : F/5, F, 5F
num_e = 1
num_r = int(sys.argv[5])
efs = [sys.argv[1] ] #, sys.argv[2]] #, sys.argv[3]] #e.g. tb3_gz_5c_scanF2, ...

param_varying = [sys.argv[3], sys.argv[4] ] #, sys.argv[6]]


# Plot Median RT
medrt_S_LC_LP = [0.0 for x in range(num_e)]
medrt_S_MapCB_NavC_LP = [0.0 for x in range(num_e)]
medrt_S_MapCB_NavP_NavC_LP = [0.0 for x in range(num_e)]
medrt_S_MapCB_MapU_NavP_NavC_LP = [0.0 for x in range(num_e)]

"""
for ei in range(num_e):
    ename = efs[ei]
    
    mean_medrt_S_LC_LP = 0.0
    mean_medrt_S_MapCB_NavC_LP = 0.0
    mean_medrt_S_MapCB_NavP_NavC_LP = 0.0
    mean_medrt_S_MapCB_MapU_NavP_NavC_LP = 0.0

    mean_75perc_rt_S_LC_LP = 0.0
    mean_75perc_rt_S_MapCB_NavC_LP = 0.0
    mean_75perc_rt_S_MapCB_NavP_NavC_LP = 0.0
    mean_75perc_rt_S_MapCB_MapU_NavP_NavC_LP = 0.0
    
    for run in range(1,num_r+1,1):
        with open('nav2d_robot_logs_' + ename + '_run' + str(run) + '.err', 'r') as of:
            a=0.0
            b=0.0
            c=0.0
            d=0.0
            a75p=0.0
            b75p=0.0
            c75p=0.0
            d75p=0.0
            for l in of.readlines():
                if "RT wrt S-LC-LP" in l:
                    a= float(l.split(' ')[13])
                    a75p = float(l.split(' ')[22][:-1])
                elif "RT wrt S-MapCB-NavC-LP" in l:
                    b= float(l.split(' ')[13])
                    b75p = float(l.split(' ')[22][:-1])
                elif "RT wrt S-MapCB-NavP-NavC-LP" in l:
                    c= float(l.split(' ')[13])
                    c75p = float(l.split(' ')[22][:-1])
                elif "RT wrt S-MapCB-MapU-NavP-NavC-LP" in l:
                    # print l, l.split(' ')[13]
                    d= float(l.split(' ')[13])
                    d75p = float(l.split(' ')[22][:-1])
        mean_medrt_S_LC_LP += a
        mean_medrt_S_MapCB_NavC_LP += b
        mean_medrt_S_MapCB_NavP_NavC_LP += c
        mean_medrt_S_MapCB_MapU_NavP_NavC_LP += d

    medrt_S_LC_LP[ei] = mean_medrt_S_LC_LP/num_r
    medrt_S_MapCB_NavC_LP[ei] = mean_medrt_S_MapCB_NavC_LP/num_r
    medrt_S_MapCB_NavP_NavC_LP[ei] = mean_medrt_S_MapCB_NavP_NavC_LP/num_r
    medrt_S_MapCB_MapU_NavP_NavC_LP[ei] = mean_medrt_S_MapCB_MapU_NavP_NavC_LP/num_r
"""

def plot_smt(arr, yl, titl):
    ngroups = num_e
    fig, ax = plt.subplots()
    index = np.arange(ngroups)
    bar_width = 0.2
    opacity = 0.8
    plt.bar(index, arr, alpha=opacity, color='b', label=titl)
    plt.ylabel(yl)
    plt.title(titl)
    plt.xticks(index, param_varying)
    plt.show()
  
"""
print 'Avg MedianRT across ' + str(num_r) + ' runs.', medrt_S_LC_LP
plot_smt(medrt_S_LC_LP, 'RT (Sec)', 'Median RT wrt S-LC-LP')
plot_smt(medrt_S_MapCB_NavC_LP, 'RT (sec)', 'Median RT wrt S-MapCB-NavC-LP')
plot_smt(medrt_S_MapCB_NavP_NavC_LP, 'RT (sec)', 'Median RT wrt S-MapCB-NavP-NavC-LP')
plot_smt(medrt_S_MapCB_MapU_NavP_NavC_LP, 'RT (sec)', 'Median RT wrt S-MapCB-MapU-NavP-NavC-LP')
# plt.bar(index, medrt_S_LC_LP, alpha=opacity, color='b', label='Med RT S-LC-LP')
# # plt.xlabel()
# plt.ylabel('RT (sec)')
# plt.title('Median RT wrt S-LC-LP')
# plt.xticks(index, param_varying)
# plt.show()
"""

def get_room_no(x,y):
    # given x,y of robot, find room#
    if (x >= -3.5) and (x <= 3.5) and (y >= -2.5) and (y <= 0.5):
        return 1
    elif (x >= 0) and (x <= 2) and (y >=1) and (y <= 5):
        return 2
    elif (x >= -7.5) and (x <= -5.5) and (y >= 1.5) and (y <= 4.5):
        return 3
    elif (x >= 2.5) and (x <= 7.5) and (y >= 0) and (y <= 4.5) :
        return 4 #room5
    elif (x >= 5) and (x <= 7.5) and (y >= -5) and (y <= -0.5) :
        return 5 #room6
    else:
        return -1 

def get_obstacle_no_stage(x,y):
    if (x >= -7) and (x <= -1) and (y >= -1) and (y <= 5):
        return 1 # robot1 is line_no+1
    elif (x >= -15) and (x <= -9) and (y >= -2) and (y <= +4):
        return 2 # robot2 is line_no+2
    elif (x >= 1) and (x <= 7) and (y >= 6) and (y <= 12) :
        return 3 # robot3 is line_no+3
    elif (x >= 0.9) and (x <= 7) and (y >= 1) and (y <= 5.5):
	return 4 # robot4 is line_no+4
    elif (x >= 9) and (x <= 15) and (y <= -2.5) and (y >= -7):
	return 5
    elif (x <= 0) and (x >= -8) and (y >= 9) and (y <= 13):
	return 6
    elif (x <= -8) and (x >= -16) and (y >= 9) and (y <= 13):
	return 7
    elif (x >= 0) and (x <= 4.5) and (y >= -13) and (y <= -8):
	return 8
    else:
        return -1

def get_dist(x1,y1,x2,y2):
    return math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))

# Plot avg dist to obstacle : 5rooms - 5cases
# room1 : x [-3.5,3.5] y [-2.5,0.5]
# room2 : x [0,2] y [1,5]
# room3 : x [-7.5, -5.5] y [1.5, 4.5] 
# room5 :
# room6 : 

def aggregate_over_time(m_arr, ts_arr, start_t, slot, end_t):
	new_m_arr = []
	new_ts_arr = []
	start_arr = []
	# start from startt, any reading 
	i = 0
	curr_ts = start_t
	print "#### Func aggregate_over_time called with params: ", slot, start_t, end_t
	print "Len of array to be aggregated: ", len(m_arr), " 0th TS:", ts_arr[0]
	# aggregate until TS > curr_ts + slot. 
	while (curr_ts + slot) < end_t:
		# collect all those in this slot.
		this_slot = []
		this_slot_ts = []
		while i < len(m_arr):
			if ts_arr[i] < curr_ts:
				i += 1
			elif ts_arr[i] <= (curr_ts + slot):
				this_slot.append(m_arr[i])
				this_slot_ts.append(ts_arr[i])
				# print "Adding ind ", i, " TS: ", ts_arr[i], " to slot from ", curr_ts
				i += 1
			else:
				break
		# print "Done with this slot, moving to next, i=", i
		curr_ts += slot
		start_arr.append(curr_ts)
		new_m_arr.append(this_slot)
		new_ts_arr.append(this_slot_ts)
	# returns aggregated arr of arr, arr of end-TS of slots, arr of arr of TS
	return new_m_arr, start_arr, new_ts_arr

avg_obst_dist_arr = [0.0 for x in range(num_e)]
obst_dist_arr = [[] for x in range(num_e)]

num_obst = int(sys.argv[6])

def plot_obstDist_agg(dist, ts, start_t, end_t, slot):
	agg1_obst_dist_arr, agg1_start_arr, agg1_obst_ts_arr = aggregate_over_time(dist, ts, start_t, slot, end_t)
        # plot with start_arr on x, avg obst dist on y.
        agg1_obst_AvgDist_arr = []
        agg1_obst_25pDist_arr = []
        agg1_obst_aggStart_arr = [] # remove the slots where no obstacle near robo.
        for i in range(len(agg1_obst_dist_arr)):
                a = []
                for j in range(len(agg1_obst_dist_arr[i])):
                        if agg1_obst_dist_arr[i][j] > 0.001:
                                a.append(agg1_obst_dist_arr[i][j])
                if len(a) > 0:
                        agg1_obst_aggStart_arr.append(agg1_start_arr[i])
                        agg1_obst_AvgDist_arr.append( sum(a)/len(a) )
                        a.sort()
                        agg1_obst_25pDist_arr.append( a[( 25*len(a) )/100] )
	print "Done with all slots, Len arr:", len(agg1_obst_aggStart_arr)	

        plt.plot(agg1_obst_aggStart_arr, agg1_obst_25pDist_arr, 'g*:', markersize=7, label="25ile Dist")
        plt.plot(agg1_obst_aggStart_arr, agg1_obst_AvgDist_arr, 'b^-.', markersize=7, label="Avg Dist")
        plt.xlabel('Time')
        plt.title("Avg/25ile Dist from closest obstacle in " + str(slot) + "s period")
	plt.ylabel('Distance')
	plt.legend()
	plt.show()

def plot_odom_agg(odom, ts, start, end, slot):
	agg_odom, agg_end, agg_ts = aggregate_over_time(odom, ts, start, slot, end)
	# fraction of time vel=0, i.e. robot stopped.
	agg_stopped_frac_arr = []
	agg_AvgOdom_arr = []
	agg_end_arr = []
	for i in range(len(agg_odom)):
		ct = 0
		for j in range(len(agg_odom[i])):
			if agg_odom[i][j] < 0.025:
				ct += 1	
		if len(agg_odom[i]) >0:
			agg_stopped_frac_arr.append(float(ct)/len(agg_odom[i]))
			agg_AvgOdom_arr.append( sum(agg_odom[i])/len(agg_odom[i]) )
			agg_end_arr.append( agg_end[i] )
	plt.plot(agg_end_arr, agg_stopped_frac_arr, 'g*:', markersize=7, label="Speed=0 Fraction")
	plt.plot(agg_end_arr, agg_AvgOdom_arr, 'b^-.', markersize=7, label="Avg Speed")
	plt.xlabel('Time')
	plt.title('Fraction of time with Speed=0 & AvgSpeed in ' + str(slot) + "s periods")
	plt.ylabel('Speed')
	plt.legend()
	plt.show()


# TS for starting, ending aggregation:
start_t = float(sys.argv[7])
end_t = float(sys.argv[8])
print "Num obstacles : ", num_obst
for ei in range(num_e):
    ename = efs[ei]
    avg_obst_d = 0.0

    for run in [3]:
        r_avg_obst_dist = 0.0
        num_read = 0
	# for plotting TimeSeries of ObstDist:
	obst_dist_ts_arr = []
	obst_room_arr = []
	obst_ts_arr = []
	robo_odom_arr = []
        with open('../robot_nav2d_obstacleDist_logs_' + ename + '_run' + str(run) + '.txt', 'r') as obf:
            obfl = obf.readlines()
            # split into groups of 6*[]lines. for measure_obstacleDist script.
            # numl = 6*4
            # split into groups of 4lines for stageros.
            numl = (num_obst+3)
            print "Reading ", ename, ", run:", run
            for i in range(len(obfl)/numl):
                # for tb3 - gz:
                # rob_pos = i*numl
                # rob_x = float( obfl[tb3_pos].split(' ')[-1][:-1] )
                # rob_y = float( obfl[tb3_pos+1].split(' ')[-1][:-1] )
                # obst_ind = get_room_no(rob_x, rob_y)

                # for stage:
                rob_pos = i*numl + 1
                rob_pos_l = obfl[rob_pos].split(' ')
                rob_x = float(rob_pos_l[1])
                rob_y = float(rob_pos_l[2][:-1])
                obst_ind = get_obstacle_no_stage(rob_x, rob_y)
		
		# read TS:
		pos_st_ts = float(obfl[i*numl].split(' ')[1])
		pos_rt_ts = float(obfl[i*numl].split(' ')[3]) # need to divide by 1000 if using boost sys_clock.msec count.                

		# read odom info:
		vx = float( obfl[i*numl + num_obst + 2].split(' ')[1] )
		vy = float( obfl[i*numl + num_obst + 2].split(' ')[2] )
		robo_odom_arr.append( math.sqrt( vx*vx + vy*vy ) ) 

                if (obst_ind > -1):
                    # 4lines for each robot in tb3-gazebo:
                    # obst_x = float( obfl[tb3_pos + 4*obst_ind].split(' ')[-1][:-1] )
                    # obst_y = float( obfl[tb3_pos + 4*obst_ind + 1].split(' ')[-1][:-1] )

                    # 1line per obstacle for stageros:
                    obst_line = obfl[rob_pos + obst_ind].split(' ')
                    obst_x = float(obst_line[1])
                    obst_y = float(obst_line[2][:-1]) # remove \n.

                    dist = get_dist(rob_x, rob_y, obst_x, obst_y)

                    r_avg_obst_dist += dist
                    num_read += 1
                    obst_dist_arr[ei].append(dist)

		    obst_dist_ts_arr.append(dist)
	
                    if i % 200 == 5:
                        print "i:", i, ", tb3_pos:", rob_pos, rob_x, rob_y, obst_x, obst_y, obst_ind
                    if dist > 15.0:
                        print "i:", i, ", tb3_pos:", rob_pos, rob_x, rob_y, obst_x, obst_y, obst_ind, " DIST : ", dist
		else:
		    obst_dist_ts_arr.append(0.0)
		obst_ts_arr.append(pos_rt_ts)
		obst_room_arr.append(obst_ind)
        print "Num reads: ", num_read  
	print "PLOTTING TimeSeries of ObstDist: "
	plt.plot(obst_ts_arr, obst_dist_ts_arr, 'bo:', label="ObstDist")
	plt.plot(obst_ts_arr, obst_room_arr, 'r^-.', label="Obst Id")
	plt.title('Dist from closest obst')
	plt.xlabel('Time')
	plt.ylabel('Dist.')
	plt.legend()
	plt.show()

	agg1_slot = 2.0
	agg2_slot = 10.0
	plot_obstDist_agg(obst_dist_ts_arr, obst_ts_arr, start_t, end_t, 2.0)
	plot_obstDist_agg(obst_dist_ts_arr, obst_ts_arr, start_t, end_t, 10.0)

	
	plt.plot(obst_ts_arr, robo_odom_arr, 'g*:', label="Odom")
	plt.xlabel('Time')
	plt.ylabel('Absolute robot speed')
	plt.title('Robot Velocity')
	plt.show()

	# agg1_robo_odom_arr, agg1_start_arr, agg1_robo_ts_arr = aggregate_over_time(robo_odom_arr, obst_ts_arr, start_t, 2.0, end_t)
	plot_odom_agg(robo_odom_arr, obst_ts_arr, start_t, end_t, 2.0)
	plot_odom_agg(robo_odom_arr, obst_ts_arr, start_t, end_t, 10.0)

        if (num_read) > 0:
            avg_obst_d += r_avg_obst_dist/num_read
            print "Avg dist for this run :", r_avg_obst_dist/num_read
        else:
            avg_obst_d += 100

    avg_obst_dist_arr[ei] = avg_obst_d/num_r

print "Avg obst dist :", avg_obst_dist_arr
#plot_smt(avg_obst_dist_arr, "Avg dist (m)", "Avg Distance of Robot to Closest Obstacle [In room]")

sty = ['g-', 'b*', 'r--']
# Plotting cdf of distance:
for ei in range(num_e):
    arr = obst_dist_arr[ei]
    arr = np.sort(arr)
    pr = 1. * np.arange(len(arr))/(len(arr) - 1)
    #plt.plot(arr, pr, sty[ei], label=efs[ei])

plt.ylabel('CDF')
plt.legend()
plt.show()


# Plot Map Area/Size
# Read mapper output
avg_map_sz = [0 for x in range(num_e)]
avg_unknown_ratio = [00 for x in range(num_e)]

def plot_coveredArea_agg(new_area_arr, area_ts, start, end, slot):
	agg_area, agg_end, agg_ts = aggregate_over_time(new_area_arr, area_ts, start, slot, end)
	agg_area_covered = [ sum(x) for x in agg_area ]
	plt.plot(agg_end, agg_area_covered, 'b*:', label="New Area Explored")
	plt.xlabel('Time')
	plt.ylabel('MapArea in #cells')
	plt.title('New area Explored per ' + str(slot) + 's period')
	plt.legend()
	plt.show()

def get_area_stats(fn):
	print "In get area stats for file", fn
	known_area_arr = []
	known_area_ts = []
	new_area_arr = []
	last_known_area = 0.0
	with open(fn, 'r') as mf:
            for l in mf.readlines():
                if 'ratio of unknown/total area' in l:
                    a = l.split(' ')[-2]
                    # Later maybe change to just int(a).
                    mpsz = int(a)
                    unk_ratio = float(l.split(' ')[-3])/mpsz
                    known_area = mpsz*(1.0 - unk_ratio)
		    known_area_arr.append( known_area )
                    # add TS.
		    new_area_arr.append(known_area - last_known_area)
                    # if using ros walltime:
		    # known_area_ts.append( float(l.split(' ')[1][1:-1] ) )
		    # using mono time from run1056 onw:
		    known_area_ts.append( float(l.split(' ')[4]) )
		    last_known_area = known_area
	print "Len of known_area_arr : ", len(known_area_arr)
	plt.plot(known_area_ts, known_area_arr, 'b*:', label="Total Explored Area")
	plt.title('Area Explored by Robot')
	plt.xlabel('Time')
	plt.ylabel('Area, in #cells')
	plt.legend()
	plt.show()
	plt.plot(known_area_ts, new_area_arr, 'g^-.', label="New explored Area")
	plt.title('New Area Explored by Robot')
        plt.xlabel('Time')
        plt.ylabel('Area, in #cells')
        plt.legend()
        plt.show()
	plot_coveredArea_agg(new_area_arr, known_area_ts, start_t, end_t, 2.0)
	plot_coveredArea_agg(new_area_arr, known_area_ts, start_t, end_t, 10.0)
	return known_area_arr, new_area_arr, known_area_ts

logs_file = sys.argv[9]
known_ar_arr, new_ar_arr, ar_ts = get_area_stats(logs_file)

# Aggregate wrt time now.

"""
for ei in range(num_e):
    ename = efs[ei]
    sum_mpsz = 0
    sum_unk_ratio = 0.0
    for run in range(1,num_r+1,1):
        mpsz = 0
        unk_ratio = 0.0

	known_area_arr = []
	known_area_ts = []

	get_area_stats('nav2d_robot_logs_' + ename + '_run' + str(run) + '.err')
        
	with open('nav2d_robot_logs_' + ename + '_run' + str(run) + '.err', 'r') as mf:
            for l in mf.readlines():
                if 'ratio of unknown/total area' in l:
                    a = l.split(' ')[-2]
                    # Later maybe change to just int(a).
                    mpsz = int(a)
                    unk_ratio = float(l.split(' ')[-3])/mpsz
		    known_area_arr.append( mpsz*(1.0 - unk_ratio) )
		    # add TS.
		    known_area_ts.append( float(l.split(' ')[1][1:-1] ) )
	sum_mpsz += mpsz
        sum_unk_ratio += unk_ratio
        print "For expt", ename, ", run:", run, " Map Area: ", mpsz, " UNcovered: ", unk_ratio
	plt.plot(known_area_ts, known_area_arr, 'b*:', label="Explored Area")
	plt.title('Explored Area')
	plt.xlabel('Time')
	plt.ylabel('Explored Area in #cells')
	plt.legend()
	plt.show()

    avg_map_sz[ei] = sum_mpsz/num_r
    avg_unknown_ratio[ei] = sum_unk_ratio/num_r

plot_smt(avg_map_sz, 'Avg MapSize in #Cells', 'Avg MapSize at EoR')
plot_smt(avg_unknown_ratio, 'UnknownArea Ratio', 'Avg UnknownArea Ratio in Map')
"""

# TODO: For stage-p3at, ONLY count exploration runs with total explored area > 710000. 
# Anything less than that denotes incomplete exploration.
# Plot #Fails
num_fails = [0 for x in range(num_e)]
avg_time = [0.0 for x in range(num_e)]

for ei in range(num_e):
    ename = efs[ei]
    ct = 0
    avgt = 0.0
    for run in range(1,num_r+1,1):
        fail = False # 0 : unifnished, 1 : fail, -1 : success.
        with open('nav2d_robot_logs_' + ename + '_run' + str(run) + '.err', 'r') as nf:
            for l in nf.readlines():
                if 'Exploration has failed' in l and 'Total_Time_Taken:' in l:
                    fail = True
                    print l.split(' ')[-1][:4] + "#"
                    avgt += float( l.split(' ')[-1][:4] ) #remove \n
                elif 'Exploration has finished' in l and 'Total_Time_Taken:' in l:
                    fail = False
                    avgt += float( l.split(' ')[-1][:4] ) #remove \n
                elif 'Exploration failed,' in l:
                    fail = True
        ct += fail

    num_fails[ei] = ct
    avg_time[ei] = avgt/num_r
    print "For ", ename, num_fails[ei], avg_time[ei]

plot_smt(num_fails, '#Failed Runs', '#Failed Runs out of %i'%(num_r) )
# plot_smt(avg_time, 'Avg RunTime', 'Avg RunTime over all runs')
