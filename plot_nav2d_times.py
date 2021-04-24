# THIS SCRIPT PLOTS THE CI FOR ALL NODES, TPUT FOR NC Nodes.
import sys
import matplotlib.pyplot as plt
import numpy as np
import random

fname_pre_str = sys.argv[1]
fname_post_str = sys.argv[2]

start_t = float(sys.argv[3])
end_t = float(sys.argv[4])

start_run_ind = int(sys.argv[5])
end_run_ind = int(sys.argv[6])

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
                                #print "Adding ind ", i, " TS: ", ts_arr[i], " to slot from ", curr_ts
                                i += 1
                        else:
                                break
                #print "Done with this slot, moving to next, i=", i
                curr_ts += slot
                start_arr.append(curr_ts)
                new_m_arr.append(this_slot)
                new_ts_arr.append(this_slot_ts)
        return new_m_arr, start_arr, new_ts_arr

def plot_agg(x,y,start,slot,end, nm, node):
        agg1_y, agg_end, agg1_ts = aggregate_over_time(y,x,start, slot, end)

        agg_avg = []
        # need 25%ile
        # need 75%ile
        agg_25p = []
        agg_75p = []
	agg_end1 = []
        ind = 0
        for i in agg1_y:
                if len(i) > 0:
			arr = sorted(i)
                	agg_25p.append( arr[(25*len(i))/100] )
                	agg_75p.append( arr[(75*len(i))/100] )
			agg_avg.append ( sum(i)/len(i) )
			agg_end1.append(agg_end[ind])
		ind += 1
        plt.plot(agg_end1, agg_avg, 'b*:', markersize=8, label="Avg " + nm)
        plt.plot(agg_end1, agg_25p, 'g^-.', markersize=8, label="25p " + nm)
        plt.plot(agg_end1, agg_75p, 'ro:', markersize=8, label="75p " + nm)
        plt.xlabel('Time')
	plt.ylabel(nm + " in s")
        plt.title('Aggregate %s for subchain %s over %fs period'%( nm, node, slot) )
	plt.legend()
	plt.show()

def print_arr(a,s):
    sa = sorted(a)
    la = len(a)

def get_rel_ts_arr(ts):
	rel_ts = []
	start_ts = ts[0]
        for i in ts:
        	rel_ts.append(round(i - ts[0], 2))
	return rel_ts

per_run_scan_ct = []
per_run_mapupd_ts = []
per_run_mapu_time = []
per_run_navp_time = []
per_run_navp_ts = []
per_run_drops_ratio = []
per_run_scan_lat = []
aggregate_navc_tput = []

per_run_navc_ts = []
node_tputs = {} # node name -> array

for fname in ["navigator_cmd", "mapper_scanCB", "mapper_mapUpdate", "navigator_plan"]:
    node_tputs[fname] = []
    for run in range(start_run_ind,end_run_ind+1):#[52,53,54,56,57]: 
    	print("################## DOING RUN %i"%(run) )
        times = []
        ts = []
        scan_count = []
        tputs = []
        drops_ts = []
        drops_times = []
        scan_pose_ts = []
	lats = [] # lat wrt sensor inputs
        wallts = []
        try:
		with open(fname_pre_str + fname + fname_post_str + "_run" + str(run) + ".txt", 'r') as f:
		    for fl in f.readlines():
			if ("times:" in fl) or ("local_map Times" in fl):
			    times += [ round(float(x),3) for x in fl.split(" ")[2:-1] ]
			elif "ts:" in fl:
			    ts += [ float(x) for x in fl.split(" ")[2:-1] ]
			elif "ScanCOunt" in fl:
			    scan_count += [ int(x) for x in fl.split(" ")[2:-1] ]
			elif "tput:" in fl:
			    tputs += [ float(x) for x in fl.split(" ")[2:-1] ]
			    #print("Added to tputs len: %i"%(len(tputs)))
			elif "scanDrop" in fl:
			    drops_ts += [ float(x) for x in fl.split(" ")[2:-1] ]
			elif "scanPoseTS" in fl:
			    scan_pose_ts += [ int(x) for x in fl.split(" ")[2:-1] ]
                        elif "scanDropExecTimes" in fl:
                            drops_times += [ float(x) for x in fl.split(" ")[2:-1] ]
                        elif "lat:" in fl:
                            lats += [ float(x) for x in fl.split(" ")[2:-1] ]
                        elif "wallTimeScanCB" in fl:
                            wallts += [ float(x) for x in fl.split(" ")[2:-1] ]
        except:
		print("ERROR READING FOR ", fname, run)
	# plot times,ts and scan_count.
        print("Starting node ", fname, "Lengths of all arrs: times: %i, ts: %i, tputs: %i"%(len(times), len(ts), len(tputs) ) )
        if len(drops_ts) > 0:
                per_run_drops_ratio.append( len(tputs)/float(len(drops_ts)) )
                drops_times_large = filter(lambda x: x[0] > 0.0015, zip(drops_times, drops_ts))
                print("DROPS ci >1.5ms: ", drops_times_large)
        #print("NODE %s TPUT: %s \n \n "%(fname, str(tputs) ) )
        #print("NODE %s CI: %s \n \n"%(fname, str(times) ) )

        if len(scan_pose_ts) > 0:
            good_scan_pose = filter( lambda x: x > 0, scan_pose_ts )
            print("Good scan pose : %i, total : %i"%( len(good_scan_pose), len(scan_pose_ts) ) )

            '''
            plt.plot(ts, times, 'bo-', label=fname + " compute time")
            plt.title("Nav2d Node : %s"%(fname) )
            plt.legend()
            plt.show()
            '''

        #plot_agg(ts, times, start_t, 2.0, end_t, "ComputeTime", fname)
        #plot_agg(ts, times, start_t, 10.0, end_t, "ComputeTime", fname)

        sorted_times = sorted(times)
        ltimes = len(sorted_times)
	if ltimes>0:
        	print("For node %s, ci best case: %f, 10p: %f, 25p: %f, median: %f, mean %f, 75ile %f, 90ile %f, 95ile %f, worst case: %f" % ( fname, sorted_times[0], sorted_times[ltimes/10], sorted_times[(25*ltimes)/100], sorted_times[ltimes/2], sum(sorted_times)/ltimes, sorted_times[(75*ltimes)/100], sorted_times[(90*ltimes)/100], sorted_times[(95*ltimes)/100], sorted_times[-1] ) )

        if "oper" in fname:
            for i in range(len(times)):
                if (times[i] > 0.01) or (ts[i] > 114586.8 and ts[i] < 114588):
                    print i, times[i], ts[i]
        if len(lats) > 0:
            large_lats = filter(lambda x: x[0] > 1.0 , zip(lats, ts, wallts, times) ) 
            print("NODE ", fname, " VERY HIGH LATS: ", large_lats, " LARGEST Lat: ", sorted(zip(lats, ts, wallts, times), key=lambda x: x[0])[-1] )
            zip_ar = zip(lats, wallts, times)
	    random.shuffle(zip_ar)
	    per_run_scan_lat += zip_ar[:100] 

        if len(tputs) > 0:
            if "cmd" in fname:
                aggregate_navc_tput += tputs
                for i in range(len(tputs)):
                    if tputs[i] > 0.93:
                        print("FOR NAVC , LARGE tput: %f, ts: %f"%(tputs[i], ts[i]) )
            
            sorted_tput = sorted(tputs)
            ltimest = len(tputs)
            print("For node %s, Tput: 10p: %f, 25p %f, median %f, mean %f, 75ile %f, 90ile %f, 95ile %f"%( fname, sorted_tput[(10*ltimest)/100], sorted_tput[(25*ltimest)/100], sorted_tput[ltimest/2], sum(sorted_tput)/ltimest, sorted_tput[(75*ltimest)/100], sorted_tput[(90*ltimest)/100], sorted_tput[(95*ltimest)/100] ) )
            print("Len tputs: %i, Len TS: %i"%(len(tputs), len(ts)) )
            
            if "plan" in fname or ("scanCB" in fname):
                thresh = 2.0 if "plan" in fname else 1.5
                large_tput_navp = filter(lambda x: x[0] > thresh, zip(tputs, ts[1:]) )
                print("NNODE LARGE TPUT: ", fname, large_tput_navp)


            '''
                plt.plot(ts[19:], tputs[18:], 'r*-.', label=fname + " Inter-arrival Time")
                plt.xlabel("Time")
                if "cmd" in fname:
                    plt.ylim(0.0, 1.0)
                plt.ylabel(fname+" Inter-arrival Time (s)")
                plt.title("nav2d Node : %s Inter-arrival"%(fname) )
                plt.legend()
                #plt.show()
        '''
        if ("_plan" in fname and (len(ts) > 0) ):
		per_run_navp_ts.append(get_rel_ts_arr(ts))
		per_run_navp_time.append(times)

        if ("_cmd" in fname  and (len(ts) > 0) ):
                per_run_navc_ts.append(get_rel_ts_arr(ts))

        if len(scan_count) > 0:
            msc = max(scan_count)
            # scan_count = [ (x*0.1/msc) for x in scan_count]
            print("Scan ct 1st: %i, last: %i"%(scan_count[0], scan_count[-1]) )
            #plt.plot(ts, scan_count, 'g^:', label=fname + " Scan Count")
            #plt.title("Nav2d NOde : %s #Scans"%(fname))
            #plt.legend()
            #plt.show()
            print("Len tputs: %i, Len TS: %i"%(len(tputs), len(ts)) )
            #plt.plot(ts[1:], tputs, 'r*-.', label=fname + " Inter-arrival Time")
            #plt.title("nav2d Node : %s Inter-arrival"%(fname) )
            #plt.legend()
            #plt.show()
            per_run_scan_ct.append(scan_count)
            per_run_mapupd_ts.append(get_rel_ts_arr(ts))
            per_run_mapu_time.append(times)
	    #print("SCAN COUNTS : ", scan_count, ", TS: ", ts, "\n \n")

	random.shuffle(tputs)
	node_tputs[fname] += tputs[:100]
        '''
            plot_agg(ts[1:], tputs, start_t, 2.0, end_t, "Inter-arrival Time", fname)
            sorted_tput = sorted(tputs)
            ltimest = len(tputs)
            print("For node %s, Tput: median %f, mean %f, 75ile %f, 90ile %f"%( fname, sorted_tput[ltimest/2], sum(sorted_tput)/ltimest, sorted_tput[(75*ltimest)/100], sorted_tput[(90*ltimest)/100] ) )
            #plot_agg(ts[1:], tputs, start_t, 10.0, end_t, "Inter-arrival Time", fname)
        '''
print("Aggregate NC: median: %f, 75ile: %f, 95ile: %f"%(np.median(aggregate_navc_tput), np.percentile(aggregate_navc_tput, 75), np.percentile(aggregate_navc_tput, 95) ) )
for k in node_tputs:
	print(node_tputs[k], k)
	print("-")
	print("-")
'''
print("\n \n PER RUN SCAN CT : ", per_run_scan_ct)
print("-")
print("-")
print("\n \n per_run_mapupd_ts : ", per_run_mapupd_ts)
print("-")
print("-")
print("\n \n PER RUN MAPUPD CI : ", per_run_mapu_time)
print("-")
print("-")
print("\n \n PER RUN NAVP CI : ", per_run_navp_time, '\n')
print("-")
print("-")
print("-")
print("-")
print("PER RUN DROPS RATIO: , avg ratio : %f, median ratio : %f", per_run_drops_ratio, sum(per_run_drops_ratio)/len(per_run_drops_ratio), np.median(per_run_drops_ratio) )
'''
print("\n \n PER RUN NAVP TS: ", per_run_navp_ts)
print("\n \n PER RUN NAVC TS: ", per_run_navc_ts)
print("\n \n PER RUN SCAN LATENCY AT MAPCB : ", len(per_run_scan_lat), per_run_scan_lat)
