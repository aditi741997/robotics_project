# Nov5:
# Collect ALL metrics for 16runs: fi of each subchain = 2/8.
# Plot each perf metric wrt 2or3 low-level metrics.
import math
import sys
import matplotlib.pyplot as plt
import numpy as np
#from sympy import *
#from sympy.geometry import *
import random

slot = 5 # in seconds.
#For plotting area covered in first 60sec: change slot=60, end_t=start_t+60+1.
#slot = 14.0

all_CHAINS = ["Scan_MapCB_MapU_NavP_NavC_LP", "Scan_MapCB_NavCmd_LP","Scan_LC_LP", "Scan_MapCB_NavPlan_NavCmd_LP"]

def get_num_collisions_run(ts_arr, ts_colln_arr):
        # make clusters of continuous true's
        colln_cluster_len = []
        colln_cluster_start = []
        colln_cluster_end = []
        i = 0
        while i < len(ts_arr):
            while ( ( i < len(ts_arr) ) and (not ts_colln_arr[i]) ):
                i += 1
            if ( ( i < len(ts_arr) ) and ts_colln_arr[i] ):
                st = i
                while ( ( i < len(ts_arr) ) and ts_colln_arr[i]):
                    i += 1
                end = i-1
                print("GOT a cluster!! s: %i [%f], e: %i [%f]"%(st, ts_arr[st], end, ts_arr[end]) )
                colln_cluster_len.append(end - st + 1)
                colln_cluster_start.append( st )
                colln_cluster_end.append( end )

        # merge clusters with |end_i - start_i| <= ?
        newcolid_colid = {} # new id -> arr of col ids.
        if len(colln_cluster_len) > 0:
            new_colln_len = []
            new_colln_start = []
            new_colln_end = []
            colid_newcolid = {}
            colid_newcolid[0] = 0 # col id -> new col id
            newcolid_colid[0] = [0]
            for cid in range( len(colln_cluster_len) - 1 ):
                ei = colln_cluster_end[cid]
                si1 = colln_cluster_start[cid+1]
                if (abs(si1-ei) < 3) or ( abs(ts_arr[ei] - ts_arr[si1]) < 0.105):
                    colid_newcolid[cid+1] = colid_newcolid[cid]
                    print("MERGED cluster %i [ end %i:%f] WITH cluster %i [start %i: %f]"%(cid, ei, ts_arr[ei], cid+1, si1, ts_arr[si1]) )
                else:
                    colid_newcolid[cid+1] = 1+colid_newcolid[cid]
                    newcolid_colid[ 1+colid_newcolid[cid] ] = []
                    print("Cluster %i is a new cluster!"%(cid+1) )
                newcolid_colid[ colid_newcolid[cid+1] ].append(cid+1)
        # return final #colln clusters with >= 5 true's
        final_cols = set()
        for newid in newcolid_colid.keys():
            numvals = sum( [colln_cluster_len[x] for x in newcolid_colid[newid] ] )
            if numvals > 4:
                final_cols.add( ts_arr [ colln_cluster_start[ newcolid_colid[newid][0] ] ] )
        print("FINAL #nEW COLS : ", len(newcolid_colid), " #COLLISIONS: ", final_cols)
        return final_cols
        

# return dict: slot# -> aggregate value.
def aggregate_over_time(m_arr, ts_arr, start_t, slot, end_t):
	m_dict = {}
	new_m_arr = []
	new_ts_arr = []
	start_arr = []
	# start from startt, any reading 
	i = 0
	curr_ts = start_t
	slot_count = 0
	#print("#### Func aggregate_over_time called with params: ", slot, start_t, end_t)
	#print("Len of array to be aggregated: ", len(m_arr), " 0th TS:", ts_arr[0])
	# aggregate until TS > curr_ts + slot. 
	'''
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
				#print "Adding ind ", i, " TS: ", ts_arr[i], " to slot from ", curr_ts
				i += 1
			else:
				break
		#print "Done with this slot, moving to next, i=", i
		curr_ts += slot
		slot_count += 1
		start_arr.append(curr_ts)
		if len(this_slot) > 0:
			m_dict[slot_count] = this_slot
		new_m_arr.append(this_slot)
		new_ts_arr.append(this_slot_ts)
        '''
        while i < (len(m_arr)-1):
            # the whole array has not been covered.
            if ts_arr[i] < start_t:
                i += 1
            elif ts_arr[i] > end_t:
                i += 1
            else:
                sl_ct = (ts_arr[i] - start_t)//slot # this is the slot# for this element.
                if sl_ct not in m_dict:
                    m_dict[sl_ct] = []
                m_dict[sl_ct].append(m_arr[i])
                i += 1
        return m_dict

# For intra run metrics:
# 2.5s windows....
tput_agg = {  } # SC name -> list of dicts.

rt_agg = {} # chain name -> list of dicts: {slot#->aggregateVal}, one dict for each i.

tput_cc_agg = [] # list of dicts for Tput cc.

odom0_frac_agg = [] # list of dicts, one for each Frac_expt. slot# -> frac of time v=0.

obst_colln_agg = [] # list of dicts, slot# -> Time (d <= 0.8) / Time (d <= 2)

obst_ccall_agg = [] # list of dicts, slot# -> Time (d <= 1.4) / Time (d <= 2)

new_area_agg = []  # list of dicts, slot# -> Total new area seen in this time slot.

collision = {} # frac9A_run1 -> # obstacle collision readings.

runlevel_total_area_expl = {} # exp_id -> area expl.
runlevel_meantputs = {} # exp id -> array of mean tputs [mU,mC,nC,nP,CC]
runlevel_meanRTs = {} # exp id -> array of mean RTs ["Scan_MapCB_MapU_NavP_NavC_LP", "Scan_LC_LP", "Scan_MapCB_NavCmd_LP", "Scan_MapCB_NavPlan_NavCmd_LP"]

def mean_aggregate(agg_dict):
	mean_dict = {}
	for k in agg_dict.keys():
		mean_dict[k] = sum ( agg_dict[k] ) / len ( agg_dict[k] )
	return mean_dict

def plot_perf_metric(perf_agg_arr, metr_agg_arr, p,m, slt,plot_first=False):
	data_arr = []
	print("######## Plotting ", p, " vs", m, len(perf_agg_arr), len(metr_agg_arr))
	data0_arr = []
	colors = ['red', 'blue', 'cyan', 'purple', 'green', 'orange']
	for a in range(len(perf_agg_arr)):
		di_arr = []
		if plot_first:
			data0_arr.append( (metr_agg_arr[a][0], perf_agg_arr[a][0]) )
		for b in perf_agg_arr[a].keys():
			if b in metr_agg_arr[a]:
				#print "For Frac", a, " slot#",b, " present in both", (metr_agg_arr[a][b], perf_agg_arr[a][b])
				di_arr.append( (metr_agg_arr[a][b], perf_agg_arr[a][b]) )
				data_arr.append( (metr_agg_arr[a][b], perf_agg_arr[a][b]) )
		s_di_arr = sorted(di_arr, key=lambda x: x[0])
		xi_arr = [x[0] for x in s_di_arr]
		yi_arr = [x[1] for x in s_di_arr]
		# divide by num expts for each Fraci : 
		plt.scatter(xi_arr, yi_arr, color=colors[a//10])
	s_data_arr = sorted(data_arr, key=lambda x: x[0])
	xarr = [x[0] for x in s_data_arr]
	yarr = [x[1] for x in s_data_arr]
	#plt.scatter(xarr, yarr)
	plt.title('Perf metric %s vs Low-Level metric %s, Agg over %f'%(p,m, slt) )
	#plt.show()
	plt.savefig("Dec4_nav2d_" + p + "_" + m + "_" + str(int(slt)) + "s.png")
	if plot_first:
		s_d0 = sorted(data0_arr,  key=lambda x: x[0])
		xarr = [x[0] for x in s_d0]
		yarr = [x[1] for x in s_d0]
		plt.scatter(xarr, yarr)
		plt.title('Perf metric %s vs Low-level metric %s, In first slot of %f s'%(p,m, slt) )
		plt.show()

def plot_runlevel_agg(xarr, yarr, titl, yl, xl, yli=0.0, mean=[], tail=[], counts=[]):
	#plt.scatter(xarr, yarr, color=col)
        plt.plot(xarr, yarr, 'b*--', markersize=9, linewidth=3, label=yl)
        if len(mean) > 0:
            plt.plot(xarr, mean, 'g.-.', markersize=9, linewidth=3, label="Mean "+yl)
        if len(tail) > 0:
            plt.plot(xarr, tail, 'r^:', markersize=9, linewidth=3, label="Tail "+yl)
        if len(counts) > 0:
            print("ABOUT to print texts, xarr: ", xarr, " At ht: ", 0.9*min(yarr), " Counts: ", counts)
            for i in range(len(xarr)):
                # random no b/w i - i+1 / len(xarr)
                plt.text(xarr[i], 0.5*min(yarr)+random.uniform( (i*min(yarr)*0.5)/(len(xarr)), ((i+1)*min(yarr)*0.5)/len(xarr) ), str(counts[i]) )
        plt.xlabel(xl)
        plt.ylabel(yl)
        plt.legend(loc='best')
        if yli>0.0:
            plt.ylim(0.0,yli)
        elif len(tail) > 0:
            plt.ylim(0.0, max(tail))
        else:
            plt.ylim(0.0, max(yarr))
        plt.title(titl)
        plt.show()

def plot_cdfs(arrs, labels, titl, xl):
    print("PLOTTING CDFs, Len(arrs): %i, Til: %s, Xlabel: %s"%( len(arrs), titl, xl ) )
    cols = ['olive', 'magenta', 'teal', 'chocolate', 'red', 'blue', 'cyan', 'purple', 'green', 'orange'] #['b.:', 'r.:', 'g.:', '']
    for i in range(len(arrs)):
        arr = arrs[i]
        sa = np.sort(arr)
        pr = 1. * np.arange(len(arr))/(len(arr) - 1)
        plt.plot(sa, pr, '.:', color=cols[i], linewidth=2, label=labels[i])
    plt.title(titl + " CDF")
    plt.xlabel(xl)
    plt.legend(loc='best')
    plt.show()

def plot_scatter(xarr, medyarr_arr_d, tailyarr_arr_d, x_name, y_name):
    colors = ['red', 'blue', 'cyan', 'purple', 'green', 'orange']
    for i in range(len(xarr)):
        med_pts_x = []
        tail_pts_x = []
        med_pts_y = []
        tail_pts_y = []
        for j in range(len(medyarr_arr_d[i])):
            med_pts_x.append( xarr[i] )
            med_pts_y.append( medyarr_arr_d[i][j] )
            tail_pts_x.append( xarr[i] )
            tail_pts_y.append( tailyarr_arr_d[i][j] )
        plt.scatter( med_pts_x, med_pts_y, marker='*', color=colors[i] )
        plt.scatter( tail_pts_x, tail_pts_y, marker='^', color=colors[i] )
    plt.xlabel('Set period for %s'%(x_name))
    plt.ylabel('Resulting %s (s)'%(y_name) )
    plt.title('[*: Med, ^: 75ile]Metric %s w.r.t Varying Period of %s'%(y_name, x_name))
    plt.show()


# return true of e11-e21 line segment intersets with e21-e22
# treat each wall as 2 lines, thickness apart
# each robot/obstacle as 4lines.
def edges_intersect(e1,e2):
	'''
	p11 = Point2D(e11)
	p12 = Point2D(e12)
	p21 = Point2D(e21)
	p22 = Point2D(e22)
	e1 = Segment2D(p11,p12)
	e2 = Segment2D(p21,p22)
	'''
	ans = intersection(e1,e2)
	return (len(ans) > 0)

# closest dist bw 2 edges.
def get_closest_edge_dist(e1,e2):
	# TODO
	x=1

# get 4edges of the robot, return Segment2D.
def get_robot_edges(px,py,oz,ow):
	ts = oz
	tc = ow
	tantheta = ts*float((2*tc)/((2*tc*tc) - 1.0)) # tan(theta) as a func of sin theta/2 and cos theta/2
	ang = math.atan(tantheta)
	print("Robot posn : px %f,py %f, ts: %f, tc: %f, Found angle: %f"% ( px,py,oz,ow,ang ) )
	rsq = RegularPolygon( Point2D(px,py) , 0.3*sqrt(2), 4, ang - pi/4 ) 
	# ox : sin theta/2, ow : cos theta/2
	verts = rsq.vertices
	return [ Segment2D(verts[0], verts[1]), Segment2D(verts[1], verts[2]), Segment2D(verts[2], verts[3]), Segment2D(verts[3], verts[0]) ]

def get_obstacle_no_stage(x,y):
        '''
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
        '''
        if (x >= -27) and (x <= -17) and (y >= -11) and (y <= -5):
                return 1
        elif (x >= 5.5) and (x <= 11.5) and (y >= -14) and (y <= -5):
                return 2
        elif (x >= 0) and (x <= 5.5) and (y >= 5) and (y <= 13):
                return 3
        elif (x >= 7) and (x <= 15) and (y >= 7) and (y <= 13):
                return 4
        elif (x >= 13) and (x <= 22) and (y >= -11) and (y <= -5):
                return 5
        elif (x >= -28) and (x <= -19) and (y >= 9.5) and (y <= 15.5):
                return 6
        elif (x >= -18) and (x <= -10) and (y >= -3) and (y <= 3):
                return 7
        elif (x >= -18.5) and (x <= -9) and (y >= 5.0) and (y <= 10.0):
                return 8
        else:
		return -1

def get_dist(x1,y1,x2,y2):
	return math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))

walls_p1 = [ [0,1], [0,6], [0,8], [2.25,1], [2.25,0.75], [-5.75,9], [-5.75,8.75], [-8.5,9], [-9.5,7], [-9.5,-4] ]
walls_p2 = [ [0,6], [8,6], [0,13], [8,1], [8,0.75], [0,9], [0,8.75], [-7.5,9], [-6.5,7], [-6.5,-4]  ]

walls_shortedges_p1 = [ [2.3,0.75], [-5.75,8.75]  ]
walls_shortedges_p2 = [ [2.3,1.0], [-5.75,9]  ]

# todo: Make each 2Sided wall with two Segments. 

#wall thickness: ~0.25
def get_closest_wall_dist(x,y):
	walls_p1 = [ np.array(z) for z in walls_p1 ]
	walls_p2 = [ np.array(z) for z in walls_p2 ]
	pt = np.array([x,y]) #p3
	closest_dist = 100.0
	closest_wall = -1
	for wi in range(len(walls_p1)):
		p2 = walls_p2[wi]
		p1 = walls_p1[wi]
		try:
			dist = np.cross(p2 - p1, pt - p1)/np.linalg.norm(p2 - p1)
			# need to add dist from a corner if 
			c1_dist = get_dist(x,y, p1[0], p1[1])
		except:
			print( p1, p2, pt, "Input:", x,y)
		if dist <= closest_dist:
			closest_dist = dist
			closest_wall = wi
	return (closest_dist, closest_wall) 

# Walls:Thichness is ~0.25
# w1: (0,1) - (0,6) : 2sided
# w2: (0,5.6) - (8,5.6) : 2sided
# w3: (0,8) - (0,13) : 2sided
# w4: (3,0.89) - (8,0.89) : 2sided 
# w5: (-5.7,8.89) - (0,8.89) : 2sided
# w6: (-8.5,9) - (-7.5,9)
# w7: (-9.5,7) - (-6.5,7)
# w8: (-9.5,-4) - (-6.5,-4)

letter = 'N'
opt_total_Area = 339142.0

# LMap: 818045, 817667, 818141, 817838
opt_total_Area = 818045.0

runlevel_agg_lowlevelmetrics = [] # list of lists. 
runlevel_agg_lowlevelmetrics_dict = {} # metric name -> value. [median in runs, then ?]

runlevel_agg_collision_count = [] # one no for each i.

runlevel_agg_path_plan_fail_count = []

run_level_total_times = [] # list of lists

runlevel_agg_lowestTTC = [] # lowest time to collision. Inf. if no collision.

runlevel_mean_totalarea = []
runlevel_med_totalarea = [] # one no for each i.
runlevel_tail_totalarea = [] #9th area.

runlevel_med_0veltime = []
runlevel_mean_0veltime = []
runlevel_tail_0veltime = []

runlevel_agg_fullExpl_count = []
runlevel_mean_fullExplTime = []
runlevel_med_fullExplTime = [] #For each expt i, Median of expl time over all runs that finished & explored >=0.9*totalArea.
runlevel_tail_fullExplTime = []

runlevel_means = {"TotalExplTime": []}
runlevel_meds = {"TotalExplTime": []}
runlevel_tails = {"TotalExplTime": []}

runlevel_med_time80area = [] # Median Time taken to cover 80% area.
runlevel_tail_time80area = [] # Tail Time taken to cover 80% area.
runlevel_mean_time80area = []
counts_80area = [] # no. of runs per expt which cover atleast 80p area.

runlevel_med_time60area = [] # Median Time taken to cover 60% area.
runlevel_tail_time60area = [] # Tail Time taken to cover 60% area.
runlevel_mean_time60area = []
counts_60area = []

runlevel_med_pathlen = []
runlevel_mean_pathlen = []
runlevel_tail_pathlen = []

runlevel_med_areabypath = []
runlevel_mean_areabypath = []
runlevel_tail_areabypath = []

runs_tput_arrs = {} # i_subchainName -> array of [tputs] for each run.
runs_med_tputs = {} # subchain name -> array[over is] of arrays[over runs].
runs_mean_tputs = {} # subchain name -> array[over is] of arrays[over runs].
runs_75p_tputs = {} # subchain name -> array[over is] of arrays[over runs].

exptn = "OfflineMCB_H"
expts = ["DFrac1SO2SB_2c" ] #"DefaultTD_2c"] 

runs = range(61,86)
runs = range(1,51)
#runs.remove(3)
print(runs, len(runs))

#for i in [1,2,3,4,5,6]: #1,3,6,7,8,9]:
run_rts_percentile = 75 #50
run_lats_percentile = 75
for i in expts:
        run_totalareas = []
	run_tputs = {} # name -> list
	run_rts = {} # chain name -> list
        run_lats = {} # chain name -> list
        run_ttc = []

	if i == 2:
		runs = [1,3,4,5]
	colln_count = 0 # #runs with collision.
	new_colln_count = 0 # total #collns across all runs.
        path_plan_fail_count = 0
        
        run_total_times = []
        irun_75p_tput = {} # subchain name -> array.
        irun_mean_tput = {}  # subchain name -> array. for a given expt i.

        vel0_frac_arr = [] # vel0 fraction for each run.
        fullExplTimes = [] # include expl time only if finished & area > 0.9*opt.

        time_80area = [] # for each run, time to cover 80% of area. [in terms of known_area]
        time_60area = [] # for each run, time to cover 60% of area. [in terms of known_area]
        time_areas = { 20: [], 30: [], 40: [], 50: [], 60: [], 70: [], 80: [], 90: []} 
	time_st_areas = { 20: [], 30: [], 40: [], 50: [], 60: [], 70: [], 80: [], 90: []}
        area_time_zip_arr = []
        area_time_agg_dict = { 20: [], 100: [] }
        colln_count_arr = []
        time_to_areas = [] # arr of dicts
        stime_to_areas = [] # arrof dicts

        run_pathlens = [] # Distance travelled by robot
        run_areabypaths = [] # ratio of area covered to path length.

        for run in runs: #1,2]:
		run_collision_hua = False
                run_collision_count = 0

		stall_ct = 0
                sto_ct = 0
                run_expl_finished = False
		run_path_plan_fail = False
                
                #exp_id = str(i) + letter +'run_' + str(run)
		exp_id = i + '_run' + str(run)
		collision[exp_id] = {}
		# get start, end times
		start_i = 0.0
                start_rt_i = 0.0
		start_st_i = 0.0
		end_i = 0.0
		end_st_i = 0.0
                #run = 2 if (i > 1) else 1
		#run = 2
		print("Starting Frac",i, "run:",run)
		with open("nav2d_robot_logs_" + exp_id + ".err", 'r') as f:
			for fl in f.readlines():
				if "received StartExploration service action" in fl:
					start_i = float( fl.split(' ')[11][:-1] )
				        start_rt_i = float( fl.split(' ')[1][1:-1] )
                                	start_st_i = float( fl.split(' ')[2][:-2] )
				if ( ("Exploration has failed" in fl) or ("Exploration has finished" in fl) ) and "Time of finish" in fl:
					end_i = float( fl.split(' ')[-2] )
		                        end_st_i = float( fl.split(' ')[2][:-2] )
                                if ("Exploration has finished" in fl):
                                    run_expl_finished = True
                                if ("No way between robot and goal!" in fl):
                                    run_path_plan_fail = True
                		if ("Exploration failed." in fl):
                                        end_rt_i = float( fl.split(' ')[1][1:-1] )
				    	end_i = start_i + (end_rt_i - start_rt_i)
                                        end_st_i = float( fl.split(' ')[2][:-2] )
		end_i += 0.1 # expl should fail after the collision for the collision to count.
		# JUST to plot area covered in 1st 60sec: 
		# end_i = start_i + slot + 1.0
		print("For i ", i, " Start, end times: ", start_i, end_i)
		run_total_times.append(end_i - start_i - 0.1)

		# store each run's metrics as dict: window# -> value
		# so its easier to take intersection of set of keys of all metrics.
	# Get tput for each subchain
		runlevel_meantputs[exp_id] = []
		for fname in ["mapper_mapUpdate", "mapper_scanCB", "navigator_cmd", "navigator_plan"]:
			print("Starting ", fname)
                        if fname not in runlevel_agg_lowlevelmetrics_dict:
                            runlevel_agg_lowlevelmetrics_dict[fname] = []
                        tput_agg_i = {}
			tput_arr = []
			ts_arr = []
                        try:
                            with open("../robot_nav2d_" + fname + "_stats_" + exp_id + ".txt", 'r') as f:
                                    for fl in f.readlines():
                                            if "tput:" in fl:
                                                    tput_arr += [ float(x) for x in fl.split(" ")[2:-1] ]		
                                            elif "ts:" in fl:
                                                    ts_arr += [ float(x) for x in fl.split(" ")[2:-1] ]
                            
			    if fname not in run_tputs:
                                    run_tputs[fname] = []
                                    irun_75p_tput[fname] = []
                                    irun_mean_tput[fname] = []

                            if (len(ts_arr) < 1):
                                run_tputs[fname].append( np.median(tput_arr) ) #sorted(tput_arr)[ len(tput_arr)/2 ] )
                                irun_75p_tput[fname].append( sorted(tput_arr)[ (75*len(tput_arr))/100 ] )
                                irun_mean_tput[fname].append( np.mean(tput_arr) )

			    tput_agg_i = aggregate_over_time(tput_arr, ts_arr[1:], start_i, slot, end_i) 
                            #if "mapU" in fname:
                                #print(tput_agg_i, tput_arr, ts_arr, ":::: HERE'S the tput_agg_i")
                            meantput_agg_i = {}
                            
                            new_tput_arr = [] # only keep the ones within start_i, end_i.
                            new_ts_arr = [] # APPROXIMATE TS of the tput values.

                            # ignore the first tput reading, might be delay due to gap bw StartMap, StartExpl.
                            mink = min(tput_agg_i.keys() )
                            for k in tput_agg_i.keys():
                                    if  (k == mink):
                                            # Delete first tput elem.
                                            if ( len( tput_agg_i[k][1:] ) > 0 ):
                                                    meantput_agg_i[k] = sum( tput_agg_i[k][1:] ) / len( tput_agg_i[k][1:] )
                                                    new_tput_arr += tput_agg_i[k][1:]
                                                    new_ts_arr += [ (k*slot + start_i)  for x in tput_agg_i[k][1:] ]
                                    else:
                                            meantput_agg_i[k] = sum( tput_agg_i[k] ) / len ( tput_agg_i[k] )
                                            new_tput_arr += tput_agg_i[k]
                                            new_ts_arr += [ (k*slot + start_i)  for x in tput_agg_i[k] ]
                                            #print("For Frac%i%s_run%i : tput of SC%s for bin %i has just 1 elem!"%(i,letter, run,fname,k))
                            if fname not in tput_agg:
                                tput_agg[fname] = []
                            tput_agg[fname].append(meantput_agg_i)
                            runlevel_meantputs[exp_id].append( np.mean(new_tput_arr) ) #sum(tput_arr)/len(tput_arr) ) #Saving tput means
                            #run_tputs[fname].append( sum(tput_arr)/len(tput_arr) ) : Mean over each run.
                            run_tputs[fname].append( np.median(new_tput_arr) ) #sorted(tput_arr)[ len(tput_arr)/2 ] )
                            irun_75p_tput[fname].append( sorted(new_tput_arr)[ (75*len(new_tput_arr))/100 ] )
                            irun_mean_tput[fname].append( np.mean(new_tput_arr) )

                            if ("H4" in exp_id and "CB" in fname):
                                tput_ts = zip(new_tput_arr, new_ts_arr)
                                if (i + "_" + fname) not in runs_tput_arrs:
                                    runs_tput_arrs[i + "_" + fname] = []
                                runs_tput_arrs[i + "_" + fname].append( new_tput_arr )
                            
                                print("Here's the sorted tput array: ", sorted(tput_ts, key=lambda x: x[0]), len(new_tput_arr))
                                print("NP Median %f, sorted[1/2]: %f"%( np.median(new_tput_arr), sorted(new_tput_arr)[len(new_tput_arr)/2] ) )

                        except:
                            print("EXCEPTION In exp %s, for getting tput of node %s, Exception: %s"% (exp_id, fname, sys.exc_info()[0]) )
                            '''
                            if "_cmd" in fname and "H5_5c_run6" in exp_id:
                                if fname not in run_tputs:
                                    run_tputs[fname] = []
                                    irun_75p_tput[fname] = []
                                run_tputs[fname].append(1.25)
                                irun_75p_tput[fname].append(1.25)
                            '''
                            if "_plan" in fname and "NP" not in exp_id and "AllL" in exp_id:
                                if fname not in run_tputs:
                                    run_tputs[fname] = []
                                    irun_75p_tput[fname] = []
                                    irun_mean_tput[fname] = []
                                run_tputs[fname].append(10.0)
                                irun_75p_tput[fname].append(10.0)
                                irun_mean_tput[fname].append(10.0)

	# Get RT, Lat, Tput for each chain
		runlevel_meanRTs[exp_id] = []
		with open('../robot_nav2d_' + exp_id + "_rt_stats.txt", 'r') as f:
			fl = f.readlines()
                        for chain in all_CHAINS: #["Scan_MapCB_MapU_NavP_NavC_LP", "Scan_LC_LP", "Scan_MapCB_NavPlan_NavCmd_LP"]: #"Scan_MapCB_NavCmd_LP", 
				print("Starting chain ", chain)
				rts = []
				ts = []
				tputs = []
                                lats = []
				if chain not in rt_agg:
					rt_agg[chain] = []
                                if chain not in runlevel_agg_lowlevelmetrics_dict:
                                    runlevel_agg_lowlevelmetrics_dict[chain] = []
				for l in fl:
					if chain in l:
						if "RT_" in l:
							rts += [ float(x) for x in l.split(' ')[1:-1] ]
						elif "TS_" in l:
							ts += [ float(x) for x in l.split(' ')[1:-1] ]
						elif "Tput" in l:
							tputs += [ float(x) for x in l.split(' ')[1:-1] ] 
				                elif "Latency" in l:
                                                        lats += [ float(x) for x in l.split(' ')[1:-1] ]
                                # aggregate the RTs array, then add slot_mean_dict to rt_agg.
				runlevel_meanRTs[exp_id].append( 5.0 )
				chainrt_agg_i = aggregate_over_time(rts, ts[1:], start_i, slot, end_i)
				rt_agg[chain].append(mean_aggregate(chainrt_agg_i) )

				if chain not in run_rts:
					run_rts[chain] = []
                                        run_lats[chain] = []
                                
                                try:
                                    run_rts[chain].append( np.percentile(rts, run_rts_percentile ) )
                                    run_lats[chain].append( np.percentile(lats, run_lats_percentile ) )
                                except:
                                    print("EXCEPTION in getting rts/lats for chain %s, exp: %s"%(chain, exp_id) )

				if "LC_LP" in chain:
					# print "Here's CC RT: ", rt_agg[chain][-1]
					tput_cc_agg_i = aggregate_over_time(tputs, ts[1:], start_i, slot, end_i)
					tput_cc_agg.append(mean_aggregate(tput_cc_agg_i) )
					# print "GOT CC tput", tput_cc_agg[-1]
                                        if len(tputs) > 0:
                                            runlevel_meantputs[exp_id].append( np.mean(tputs) ) #sum(tputs)/len(tputs) )
					if chain+"_tput" not in run_tputs:
						run_tputs[chain+"_tput"] = []
                                                irun_75p_tput[chain+"_tput"] = []
                                                irun_mean_tput[chain+"_tput"] = []
                                        if chain + "_tput" not in runlevel_agg_lowlevelmetrics_dict:
                                                runlevel_agg_lowlevelmetrics_dict[chain + "_tput"] = []
					#run_tputs[chain].append( sum(tputs)/len(tputs) )
                                        try:
                                            run_tputs[chain+"_tput"].append( np.median(tputs) ) # Median over each run
                                            irun_75p_tput[chain+"_tput"].append( sorted(tputs)[(75*len(tputs))/100] ) # 75%ile over each run.
                                            irun_mean_tput[chain+"_tput"].append( np.mean(tputs) )
                                            if ("H5" in exp_id):
                                                # need to plot cdf for cc, H7.
                                                if (i + "_" + chain) not in runs_tput_arrs:
                                                    runs_tput_arrs[i + "_" + chain] = []
                                                runs_tput_arrs[i + "_" + chain].append( tputs )
                                        except:
                                            print("Exception in CC_Tput reading!!! len of tputs: ", len(tputs))
                                            if "CC" not in exp_id and "AllLow" in exp_id:
                                                run_tputs[chain+"_tput"].append(1.0)
                                                irun_75p_tput[chain+"_tput"].append(1.0)
                                                irun_mean_tput[chain+"_tput"].append(1.0)
                                            raise

	# Get intra run perf metrics
	# 1. Odometry v=0 fraction per 2s
                run_path_len_sum = 0.0
		with open('../robot_nav2d_obstacleDist_logs_' + exp_id + '.txt', 'r') as f:
			num_obst = 8
			obfl = f.readlines()
			numl = (num_obst+3)
			ts_arr = []
                        ts_colln_bool_arr = []
			robo_odom_arr = []
			robo_ang_odom_arr = []
			obst_dist_arr = []
			obst_ts_arr = []
			wall_dist_arr = []

                        old_pos_x = 0.0
                        old_pos_y = 0.0
                        path_started = False
			for o in range(len(obfl)//numl):
				rob_pos = o*numl + 1
				rob_pos_l = obfl[rob_pos].split(' ')
				rob_x = float(rob_pos_l[1])
				rob_y = float(rob_pos_l[2][:-1])
				obst_ind = get_obstacle_no_stage(rob_x, rob_y)
		
				#closest_wall_dist, wall_id = get_closest_wall_dist(rob_x,rob_y)
				closest_wall_dist, wall_id = 100,-1
				wall_dist_arr.append( closest_wall_dist )

				# read TS:
                                try:
				    pos_st_ts = float(obfl[o*numl].split(' ')[1])
				    pos_rt_ts = float(obfl[o*numl].split(' ')[3])
                                except:
                                    print("ERROR in numl = %i, o: %i, line split: "%(numl,o), obfl[o*numl].split(' ') )
                                    raise

				if ( (pos_rt_ts > start_i) and (pos_rt_ts < end_i) ):
				        # Calculating dist travelled for pathlen:
                                        if not path_started:
                                            path_started = True
                                        else:
                                            run_path_len_sum += get_dist(rob_x,rob_y,old_pos_x,old_pos_y)

                                        old_pos_x = rob_x
                                        old_pos_y = rob_y

                                        vx = float( obfl[o*numl + num_obst + 2].split(' ')[1] )
					vy = float( obfl[o*numl + num_obst + 2].split(' ')[2] )
					robo_odom_arr.append( math.sqrt( vx*vx + vy*vy ) )
					try:
						vang = float( obfl[o*numl + num_obst + 2].split(' ')[4] )
						#if o%2000 == 77:
							#print("VELS for o=", o,vx,vy,vang)
							#print "Dist from closest wall:", rob_x, rob_y, wall_id, closest_wall_dist 			
						av = math.sqrt( vx*vx + vy*vy + vang*vang)
						robo_ang_odom_arr.append( av )
					except:
						print("PROBLEM in reading vang for o: ", o)
                                                raise

					# get orientation
					oz = float( obfl[o*numl + num_obst + 2].split(' ')[6] )
					ow = float( obfl[o*numl + num_obst + 2].split(' ')[7] )
					
					
					ts_arr.append(pos_rt_ts)
			                ts_colln_hua = False	
                                        rob_stalled = obfl[o*numl + num_obst + 2].split(' ')[9]
                                        if "\n" in rob_stalled:
                                            rob_stalled = rob_stalled[:-1]
					if ( (pos_rt_ts > start_i) and (pos_rt_ts < end_i) and (int( rob_stalled ) == 1) ):
						run_collision_hua = True
                                                ts_colln_hua = True
						stall_ct += 1                                       
 
                                        #if "St_O" in obfl[o*numl + num_obst + 2].split(' '):
                                            #print(obst_ind, obfl[o*numl + num_obst + 2], "St_O!!!!")
					
					if obst_ind != -1:
						obst_line = obfl[rob_pos + obst_ind].split(' ')
						obst_x = float(obst_line[1])
						obst_y = float(obst_line[2][:-1]) # remove \n.
						dist = get_dist(rob_x, rob_y, obst_x, obst_y)
						obst_ts_arr.append(pos_rt_ts)
	                                        obst_dist_arr.append(dist)
                                                if "St_O" in obfl[o*numl + num_obst + 2]:
                                                    #print("St_O!!!!!! obst ind %i, line %s, dist %f, rt TS: %f"%(obst_ind, obfl[o*numl + num_obst + 2], dist, pos_rt_ts) )
                                                    if (dist < 1.5) and ((pos_rt_ts > start_i) and (pos_rt_ts < end_i)) :
						    # collision! check pos of robot and this obst
						        run_collision_hua = True
							ts_colln_hua = True
                                                        sto_ct += 1
                                        ts_colln_bool_arr.append(ts_colln_hua)
                        run_collision_count = get_num_collisions_run(ts_arr, ts_colln_bool_arr)
			'''
					if ( pos_rt_ts > (end_i - (end_i - start_i)/10 ) ) and (pos_rt_ts < (end_i + 1.0) ):
						robot_edges = get_robot_edges(rob_x, rob_y, oz, ow) # gives an arr of 4segments.
						colln = False
						#if (o%100 == 7):
						print("GOING to try for CollisionDETECTion at o=%i,robx=%f,roby=%f, edges: %i"%(o,rob_x,rob_y, len(robot_edges) ) )
						for redg in robot_edges:
							print("STARTing to check edge ", redg)
							for i in range( len(walls_p1) ):
								if (not colln):
									wi = Segment2D( Point2D(walls_p1[i]), Point2D(walls_p2[i]) )
									colln = colln or (edges_intersect(redg, wi) )
							print("CHECKed walls1")
							for i in range( len(walls_shortedges_p1) ):
								if (not colln):
									wi = Segment2D( Point2D(walls_shortedges_p1[i]), Point2D(walls_shortedges_p2[i]) )
									colln = colln or (edges_intersect(redg, wi))
							print("CHECKed walls short")
							if colln:
								print("For Frac%i%s_run%i : Collision with wall!! at time %f, robx: %f, roby: %f"%(i,letter,run,pos_rt_ts,rob_x,rob_y) )
							if ( (obst_ind != 1) and (not colln) ):
								obst_edges = get_robot_edges(obst_x, obst_y, 0, 1) # obstacles always ||al to axes.
								for oedg in obst_edges:
									if (not colln):
										colln = colln or ( edges_intersect(oedg, redg) )
								if colln:
									print("For Frac%i%s_run%i : Collision with obstacle!! at time %f, robx: %f, roby: %f"%(i,letter,run,pos_rt_ts,rob_x,rob_y) )
									collision[exp_id][pos_rt_ts] = True
	    		'''

			odom_agg_i = aggregate_over_time(robo_odom_arr, ts_arr, start_i, slot, end_i)
			odom_ang_agg_i = aggregate_over_time(robo_ang_odom_arr, ts_arr, start_i, slot, end_i)
			odom0_agg_i = {}
                        total_vel0_count = 0
			for k in odom_ang_agg_i.keys():
				ct = 0
				for x in odom_ang_agg_i[k]:
					if x < 0.01:
						ct += 1
                                                total_vel0_count += 1
				odom0_agg_i[k] = float(ct) / len(odom_ang_agg_i[k])
			odom0_frac_agg.append(odom0_agg_i)
                        vel0_frac_arr.append(float(total_vel0_count)/len(robo_ang_odom_arr))
			#print("For Frac%iA, run%i, #collision points: %i"%( i,run, len(odom0_agg_i) ) )	

	# 2. Obstacle Info: 
			# for each 2s period, if its within ?mtrs of obstacle, usme se amt of time its in close call.
			obst_agg_i = aggregate_over_time(obst_dist_arr, obst_ts_arr, start_i, slot, end_i)
			colln_agg_i = {}
			ccall_agg_i = {}
			for k in obst_agg_i.keys():
				ct2 = 0 # d<2m count
				coct = 0 # collision ct
				cc_ct = 0 # close call ct
				for x in obst_agg_i[k]:
					if x <= 2.0:
						ct2 += 1
					if x <= 1.4:
						cc_ct += 1
					if x <= 0.82:
						coct += 1
				#if ct2 > 5:
				colln_agg_i[k] = coct/len(obst_agg_i[k]) #/float(ct2)
				ccall_agg_i[k] = cc_ct/len(obst_agg_i[k]) #/float(ct2)
			obst_colln_agg.append(colln_agg_i)
			obst_ccall_agg.append(ccall_agg_i)
			#print("For %s : obstacle colln count: %i"%(exp_id, len(colln_agg_i) )) 
	# 3. Area covered per 10s:
		new_area_covered = []
		new_area_covered_ts = []
		last_known_area = 0.0
		added_to_area_times = {}
                zip_at = []
                zip_aa = []
                zip_as = { 20: {}, 100: {}} # 20s RT ~100s ST.
		run_time_to_area = {}
                run_stime_to_area = {}
                with open("nav2d_robot_logs_OpeMap_" + exp_id + ".err", 'r') as f:
			for l in f.readlines():
				if 'ratio of unknown/total area' in l:
					try:
						mpsz = int(l.split(' ')[-2])
						unk = float(l.split(' ')[-3])
						known = mpsz - unk
						new_area_covered.append(known - last_known_area)
						new_area_covered_ts.append( float(l.split(' ')[4] ) )
                                                zip_aa.append(known)
                                                # For RT: 
                                                zip_at.append( float(l.split(' ')[4] ) )
                                                # For ST: 
                                                #zip_at.append( float(l.split(' ')[2][:-2] ) )
                                        except:
						print("ERROR in line %s in getting area stuff!!"%(l) )
                                                raise
                                        if ( (known >= 0.8*opt_total_Area) and (last_known_area < 0.8*opt_total_Area) ):
                                            time_80area.append( float(l.split(' ')[4]) - start_i )
                                        if ( (known >= 0.6*opt_total_Area) and (last_known_area < 0.6*opt_total_Area) ):
                                            time_60area.append( float(l.split(' ')[4]) - start_i )
		                        if ( (known >= 0.9*opt_total_Area) and (last_known_area < 0.9*opt_total_Area) ):
                                            fullExplTimes.append( float(l.split(' ')[4]) - start_i )
                                        for k in time_areas.keys():
                                            ratio = float(k)/100.0
                                            if ( ( known >= ratio*opt_total_Area) and (last_known_area < ratio*opt_total_Area) ):
                                                time_areas[k].append( round(float(l.split(' ')[4]) - start_i, 3) )
						time_st_areas[k].append( round(float(l.split(' ')[2][:-2]) - start_st_i, 3) )
						added_to_area_times[ratio] = True
						run_time_to_area[k] = round(float(l.split(' ')[4]) - start_i, 3)
                                                run_stime_to_area[k] = round( float(l.split(' ')[2][:-2]) - start_st_i, 3 )
                                                if ratio > 0.6:
							print("Adding line %s to ratio %f"%(l, ratio) )
					last_known_area = known
                time_to_areas.append( run_time_to_area )
                stime_to_areas.append( run_stime_to_area )
                for zk in zip_as.keys():
                    agg_timearea = aggregate_over_time(zip_aa, zip_at, start_i, zk, end_i)
                    #take last entry for each time slot
                    for si in agg_timearea.keys():
                        zip_as[zk][si] = agg_timearea[si][-1] # last area val covered in each timeslot.
                    area_time_agg_dict[zk].append( zip_as[zk] )
                for rx in added_to_area_times.keys():
			if (last_known_area < (rx*opt_total_Area)):
				print("WEIRDDD!!! Final area %f < %f * opt!!!"%(last_known_area, rx) )
                runlevel_total_area_expl[exp_id] = last_known_area
		run_totalareas.append(last_known_area)
		new_area_cov_Agg = aggregate_over_time(new_area_covered, new_area_covered_ts, start_i, slot, end_i)
		sum_new_area_cov_agg = {}
		for k in new_area_cov_Agg.keys():
			sum_new_area_cov_agg[k] = sum( new_area_cov_Agg[k] )
		#print("Sum nEW Area covered Agg: ", sum_new_area_cov_agg) # Do this only for first slot.
		new_area_agg.append(sum_new_area_cov_agg)
		if (stall_ct > 5) or (sto_ct > 5):
			colln_count += run_collision_hua
     
                #new_colln_count += run_collision_count
		colln_count_arr.append( run_collision_count )
                path_plan_fail_count += (run_path_plan_fail and (not run_collision_hua))
                if run_collision_hua:
                    run_ttc.append(end_i - start_i - 0.1)
                print("For expt %s, collision hua? %i !! STALL COUNT: %i, ST_O CT: %i"%(exp_id, run_collision_hua, stall_ct, sto_ct) )
                
                #if ( (last_known_area > (0.9*opt_total_Area)) ): #run_expl_finished and : for now, only checking 90%area time.
                    #fullExplTimes.append(end_i - start_i - 0.1)
	        
                run_pathlens.append(run_path_len_sum)
                run_areabypaths.append( last_known_area / run_path_len_sum )
        
	# Run-level perf metrics : #Datapoints = #runs LOL.
	# aggregate runlevel metrics over all runs. [to remove randommess]
        
        # Collision count: [based on stage Stalled]
        runlevel_agg_collision_count.append( colln_count )
	numrun = len(runs)//2

        runlevel_agg_path_plan_fail_count.append( path_plan_fail_count )


        # Total Area explored
	#print("NEW Area Agg array across runs: ", new_area_agg)
	run_level_total_times.append(run_total_times)
        print("For i= ", i, ", run-TotalArea Explored:", run_totalareas)
	runlevel_med_totalarea.append( (sorted(run_totalareas)[ numrun ])/opt_total_Area ) #median over all runs.
	runlevel_mean_totalarea.append( (sum(run_totalareas)/len(runs))/opt_total_Area ) # mean totalArea
        runlevel_tail_totalarea.append( (sorted(run_totalareas)[ (8*len(runs))/10 ])/opt_total_Area ) # tail totalArea

        # Throughputs and RTs:
        ith_lowlevel_arr = []
        print("FOR EXPT: %s, run_meds_tputs for MapCB: %s"%(i, str(run_tputs["mapper_scanCB"] ) ) )
	for sc in ["mapper_mapUpdate", "mapper_scanCB", "navigator_cmd", "navigator_plan", "Scan_LC_LP_tput"]:
		ith_lowlevel_arr.append( np.median(run_tputs[sc]) ) #sorted(run_tputs[sc])[len(runs)/2] ) # Median across runs
                runlevel_agg_lowlevelmetrics_dict[sc].append( np.median(run_tputs[sc]) ) #sorted(run_tputs[sc])[len(runs)/2] )
                if sc not in runs_med_tputs:
                    runs_med_tputs[sc] = []
                    runs_75p_tputs[sc] = []
                    runs_mean_tputs[sc] = []
                runs_mean_tputs[sc].append( irun_mean_tput[sc] )
                print("FOR EXPT %s, 75P tput for  %s is %s, median[over runs] 75P tput: %f"%( i, sc, str( irun_75p_tput[sc] ), np.median(irun_75p_tput[sc]) ) )
		#print("FOR EXPT %s, MEAN tput for  %s is %s, median[over runs] Mean tput: %f"%( i, sc, str( irun_mean_tput[sc] ), np.median(irun_mean_tput[sc]) ) )
                runs_med_tputs[sc].append(run_tputs[sc]) #median of run tput
                runs_75p_tputs[sc].append(irun_75p_tput[sc]) # 75%ile of run tput
        for ch in all_CHAINS: #["Scan_MapCB_MapU_NavP_NavC_LP", "Scan_MapCB_NavCmd_LP","Scan_LC_LP", "Scan_MapCB_NavPlan_NavCmd_LP"]: #
		ith_lowlevel_arr.append( np.median(run_rts[ch]) ) #sorted(run_rts[ch])[len(runs)/2] )
	        runlevel_agg_lowlevelmetrics_dict[ch].append( np.median(run_rts[ch]) ) #sorted(run_rts[ch])[len(runs)/2] )
                print("FOR EXPT %s, chain %s, RT: %iile :  %s, median RT ile: %f"%(i, ch, run_rts_percentile, str(run_rts[ch]), np.median(run_rts[ch]) ) )
                print("FOR EXPT %s, chain %s, Latency  %iile :  %s, median Lat ile : %f"%(i, ch, run_lats_percentile, str(run_lats[ch]), np.median(run_lats[ch]) ) )
        runlevel_agg_lowlevelmetrics.append( [-1.0*x for x in ith_lowlevel_arr] )
       
        # Fraction of time v=0.
        #print("For expt %s, vel0_frac_arr: %s"%(exp_id, str(vel0_frac_arr) ) )
        runlevel_med_0veltime.append( sorted(vel0_frac_arr)[len(runs)/2] )
        runlevel_mean_0veltime.append( sum(vel0_frac_arr)/len(runs) )
        runlevel_tail_0veltime.append( sorted(vel0_frac_arr)[(8*len(runs))/10] )

        # Lowest TimeToCollision:
        if len(run_ttc) > 0:
            runlevel_agg_lowestTTC.append( min(run_ttc) )
        else:
            runlevel_agg_lowestTTC.append( 1000.0 ) # INF.

        # Time taken for full explorations:
        print("For expt %s, FullExplTimes Array: %s"%( i, str(fullExplTimes) ) )
        if len(fullExplTimes) > 0:
            runlevel_med_fullExplTime.append( sorted(fullExplTimes)[len(fullExplTimes)/2] )
            runlevel_tail_fullExplTime.append( sorted(fullExplTimes)[(8*len(fullExplTimes))/10] )
            runlevel_mean_fullExplTime.append( sum(fullExplTimes)/len(fullExplTimes) )
        else:
            runlevel_med_fullExplTime.append(2000.0)
            runlevel_tail_fullExplTime.append(2000.0)
            runlevel_mean_fullExplTime.append(2000.0)
        runlevel_agg_fullExpl_count.append( len(fullExplTimes) )

        # Time to cover 80/60% of totalArea.
        print("FOR expt %s, Time taken to cover 80p area arr: %s"%(i, str(time_80area)) )
        print("FOR expt %s, Time taken to cover 60p area arr: %s"%(i, str(time_60area)) )
        print("FOR expt %s, Time taken to cover Area arr: %s"%( i, str(time_areas) ))
        print("FOR expt %s, ST Time taken to cover Area arr: %s"%(i, str(time_st_areas) ) )
	runlevel_med_time80area.append( np.median(time_80area) ) #sorted(time_80area)[ len(time_80area)/2 ] )
        runlevel_tail_time80area.append( np.percentile(time_80area, 80, interpolation='nearest') ) #[ (8*len(time_80area))/10 ] )
        runlevel_mean_time80area.append( np.mean(time_80area) ) #sum(time_80area)/len(time_80area) )

        print("FOR expt %s, area-time zip arr : %s"%( i, str(area_time_zip_arr) ))
        print("FOR expt %s, area-time slot-wise agg : %s" %(i, str(area_time_agg_dict) ) )
        print("FOR expt %s, colln array : %s"%(i, str(colln_count_arr) ) )
        print("Time to cover Xp area : ", time_to_areas)
        print("SimTime to cover Xp area : ", stime_to_areas)

        counts_80area.append( len(time_80area) )

        counts_60area.append( len(time_60area) )
        if len(time_60area) > 0:
            runlevel_med_time60area.append( np.median(time_60area) ) #sorted(time_60area)[ len(time_60area)/2 ] )
            runlevel_tail_time60area.append( np.percentile(time_60area, 80, interpolation='nearest') ) 
            runlevel_mean_time60area.append( np.mean(time_60area) ) #sum(time_60area)/len(time_60area) )

        # Path Length covered by robot:
        print("For expt %s, pathlength covered by robot: %s"%(i, str(run_pathlens)) )
        print("For i= ", i, "#COLLISIONS IN RUNS: ", new_colln_count)
        runlevel_med_pathlen.append( np.median(run_pathlens) ) #sorted(run_pathlens)[len(run_pathlens)/2] )
        runlevel_tail_pathlen.append( np.percentile(run_pathlens, 80, interpolation='nearest') ) #sorted(run_pathlens)[(8*len(run_pathlens))/10] )
        runlevel_mean_pathlen.append( np.mean(run_pathlens) ) #sum(run_pathlens)/len(run_pathlens) )

        # Area covered / Path Length:
        #print("For expt %s, Area/pathLen: %s"%(i, str(run_areabypaths) ) )
        runlevel_med_areabypath.append( np.median(run_areabypaths) ) #sorted(run_areabypaths)[len(run_areabypaths)/2] )
        runlevel_tail_areabypath.append( np.percentile(run_areabypaths, 80,interpolation='nearest') ) #sorted(run_areabypaths)[(8*len(run_areabypaths))/10] )
        runlevel_mean_areabypath.append( np.mean(run_areabypaths) ) #sum(run_areabypaths)/len(run_areabypaths) )

        runlevel_means["TotalExplTime"].append( np.mean(run_total_times) )
        runlevel_meds["TotalExplTime"].append( np.median(run_total_times) )
        runlevel_tails["TotalExplTime"].append( np.percentile(run_total_times, 80,interpolation='nearest') )

print("#Explorations which finished fully per-expt: ", runlevel_agg_fullExpl_count)
print("Collision coUNT ARR: ", runlevel_agg_collision_count)
print("RunLevel total times: ", run_level_total_times)


print("Arr for #PathPlanFailures: ", runlevel_agg_path_plan_fail_count)

# CC: expected_tputs = [0.05, 0.1, 0.2, 0.4, 1.0] # the tputs set for the experiments 1-5.
# MCB:
expected_tputs = [0.1, 0.143, 0.25, 1.0, 2.5]
for sc in ["mapper_mapUpdate", "mapper_scanCB", "navigator_cmd", "navigator_plan", "Scan_LC_LP_tput"]:
    scn = (sc + "_tput") if "tput" not in sc else sc
    print("Plotting ScatterPlot for %s, runs_75p_tputs: %s, run_med_tputs: %s"%(scn, str(runs_75p_tputs[sc]), str(runs_med_tputs[sc]) ) )
    #plot_scatter(expected_tputs, runs_med_tputs[sc], runs_75p_tputs[sc], 'MCb', scn) # plt.scatter(expected_tputs[i], all vals of med[i] array.)

for k in runs_tput_arrs:
    plot_cdfs(runs_tput_arrs[k], ["run"+str(r) for r in runs], k, k + " Inter-arrival (s)")

# title, yl, xl
pxl = ["MapCB period", "RT S_Mcb_NC_LP", "CC Tput", "mapUpd Tput", "CC RT", "NavCmd Tput"] #"CC RT", "MapScanCB Tput", "CC Tput"]
pxnm = ["mapper_scanCB", "Scan_MapCB_NavCmd_LP", "Scan_LC_LP_tput", "mapper_mapUpdate", "Scan_LC_LP", "navigator_cmd"] #, "Scan_LC_LP", "mapper_scanCB", "Scan_LC_LP_tput"]
for i in range(len(pxl)):
        for k in runlevel_meds.keys(): 
            plot_runlevel_agg(runlevel_agg_lowlevelmetrics_dict[pxnm[i]], runlevel_meds[k], pxl[i]+" vs "+k, k, pxl[i], mean=runlevel_means[k], tail=runlevel_tails[k])
        plot_runlevel_agg(runlevel_agg_lowlevelmetrics_dict[pxnm[i]], runlevel_med_pathlen, pxl[i]+" vs RobotPathLength", "RobotpathLength (m)", pxl[i], mean=runlevel_mean_pathlen, tail=runlevel_tail_pathlen)
        #plot_runlevel_agg(runlevel_agg_lowlevelmetrics_dict[pxnm[i]], runlevel_med_areabypath, pxl[i]+" vs Area/PathLength", "Area/PathLength", pxl[i], mean=runlevel_mean_areabypath, tail=runlevel_tail_areabypath)
        if len(runlevel_med_time60area) == len(expts):
            plot_runlevel_agg(runlevel_agg_lowlevelmetrics_dict[pxnm[i]], runlevel_med_time60area, pxl[i]+" vs TimeToCover 60p Area", "Time to cover 0.6*area (s)", pxl[i], mean=runlevel_mean_time60area, tail=runlevel_tail_time60area, counts=counts_60area)
        plot_runlevel_agg(runlevel_agg_lowlevelmetrics_dict[pxnm[i]], runlevel_med_time80area, pxl[i]+" vs TimeToCover 80p Area", "Time to cover 0.4*area (s)", pxl[i], mean=runlevel_mean_time80area, tail=runlevel_tail_time80area, counts=counts_80area)
        plot_runlevel_agg(runlevel_agg_lowlevelmetrics_dict[pxnm[i]], runlevel_agg_fullExpl_count, pxl[i]+" vs #FullExplorations", "#FullExplorations/10", pxl[i], yli=10.5)
        plot_runlevel_agg(runlevel_agg_lowlevelmetrics_dict[pxnm[i]], runlevel_med_fullExplTime, pxl[i]+" vs Full Exploration RunTime", "Full Exploration Time(s)", pxl[i], yli=600.0, mean=runlevel_mean_fullExplTime, tail=runlevel_tail_fullExplTime, counts=runlevel_agg_fullExpl_count)
        #plot_runlevel_agg(runlevel_agg_lowlevelmetrics_dict[pxnm[i]], runlevel_med_0veltime, pxl[i]+" vs MedianVel=0 Fraction", "Median Vel=0 Fraction of Run", pxl[i], yli=1.0, mean=runlevel_mean_0veltime, tail=runlevel_tail_0veltime)
        #plot_runlevel_agg(runlevel_agg_lowlevelmetrics_dict[pxnm[i]], runlevel_agg_collision_count, pxl[i]+" vs #Collisions/10", "#Collisions /10", pxl[i], yli=10.5)
        #plot_runlevel_agg(runlevel_agg_lowlevelmetrics_dict[pxnm[i]], runlevel_med_totalarea, pxl[i]+ " vs TotalArea", "TotalArea [Mapper]", pxl[i], yli=1.2, mean=runlevel_mean_totalarea, tail=runlevel_tail_totalarea)
        #plot_runlevel_agg(runlevel_agg_lowlevelmetrics_dict[pxnm[i]], runlevel_agg_lowestTTC, pxl[i]+ " vs LowestTimeToColln", "TTC(sec)", pxl[i], yli=400.0)

'''
from sklearn.linear_model import LinearRegression

x = np.array( runlevel_agg_lowlevelmetrics )

y = np.array( [runlevel_agg_totalarea] )
model = LinearRegression().fit(x, y)

print("runlevel_total_area_expl: ", runlevel_total_area_expl)
print("runlevel Agg total area : ", runlevel_agg_totalarea)
print("runlevel LowLevel Agg: ", runlevel_agg_lowlevelmetrics)
print("runlevel tputs dict: ", runlevel_meantputs)
print("Order is tput mapU, mapCB, navC, navP, CC then RT Longest, CC, S_MC_NC_LP, s_mC_Np_Nc_lp")
print("model intercept %f, coeffs : %s"%( model.intercept_, str(model.coef_) ) )
'''

plot_perf_metric(obst_colln_agg, tput_cc_agg, "Collision D<0.8", 'CC Tput', slot)
plt.clf()
plot_perf_metric(odom0_frac_agg, tput_cc_agg, 'Vel=0 Fraction', 'CC Tput', slot)
plt.clf()
plot_perf_metric(odom0_frac_agg, tput_agg["navigator_cmd"], 'Vel=0 Fraction', 'NavCmd Tput', slot)
plt.clf()
#plot_perf_metric(obst_colln_agg, rt_agg["Scan_LC_LP"], "Collision D<0.8", 'CC RT', slot)
plot_perf_metric(obst_ccall_agg, tput_agg["navigator_cmd"], "Close Call", 'NavCmd Tput', slot)
plt.clf()
plot_perf_metric(obst_ccall_agg, rt_agg["Scan_LC_LP"], "Close Call", 'CC RT', slot)
plt.clf()
'''
plot_perf_metric(obst_ccall_agg, tput_agg["navigator_cmd"], "Close Call", 'NavCmd Tput', slot)
plt.clf()
plot_perf_metric(obst_colln_agg, tput_agg["navigator_plan"], "Collision D<0.8", 'NavPlan Tput', slot)
plt.clf()
plot_perf_metric(obst_colln_agg, tput_agg["navigator_cmd"], "Collision D<0.8", 'NavCmd Tput', slot)
plt.clf()
plot_perf_metric(obst_colln_agg, rt_agg["Scan_MapCB_NavCmd_LP"], "Collision D<0.8", 'RT Scan-mapCB-NavC-LP', slot)
plt.clf()
'''

#plot_perf_metric(new_area_agg, tput_cc_agg, 'New area Covered', 'CC Tput', slot)
#plot_perf_metric(new_area_agg, tput_agg["mapper_mapUpdate"], 'New area Covered', 'MapUpdate Tput', slot)
#plot_perf_metric(new_area_agg, tput_agg["mapper_scanCB"], 'New area Covered', 'MapScanCB Tput', slot)
#plot_perf_metric(new_area_agg, tput_agg["navigator_plan"], 'New area Covered', 'NavPlan Tput', slot)

