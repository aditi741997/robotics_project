# THIS SCRIPT PLOTS THE CI FOR ALL NODES, TPUT FOR NC Nodes.
import sys
import matplotlib.pyplot as plt

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

per_run_scan_ct = []
per_run_mapupd_ts = []
for run in range(start_run_ind,end_run_ind+1):
    for fname in ["navigator_cmd", "navigator_plan", "mapper_scanCB", "mapper_mapUpdate", "local_map", "operator_loop"]:
        times = []
        ts = []
        scan_count = []
        tputs = []
        drops_ts = []
        scan_pose_ts = []
        with open(fname_pre_str + fname + fname_post_str + "_run" + str(run) + ".txt", 'r') as f:
            for fl in f.readlines():
                if ("times:" in fl) or ("local_map Times" in fl):
                    times += [ float(x) for x in fl.split(" ")[2:-1] ]
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
        # plot times,ts and scan_count.
        print("Starting node ", fname, "Lengths of all arrs: times: %i, ts: %i, tputs: %i"%(len(times), len(ts), len(tputs) ) )
        #print("NODE %s TPUT: %s \n \n "%(fname, str(tputs) ) )
        #print("NODE %s CI: %s \n \n"%(fname, str(times) ) )

        if len(scan_pose_ts) > 0:
            good_scan_pose = filter( lambda x: x > 0, scan_pose_ts )
            print("Good scan pose : %i, total : %i"%( len(good_scan_pose), len(scan_pose_ts) ) )

        #plt.plot(ts, times, 'bo-', label=fname + " compute time")
        #plt.title("Nav2d Node : %s"%(fname) )
        #plt.legend()
        #plt.show()

        #plot_agg(ts, times, start_t, 2.0, end_t, "ComputeTime", fname)
        #plot_agg(ts, times, start_t, 10.0, end_t, "ComputeTime", fname)

        sorted_times = sorted(times)
        ltimes = len(sorted_times)
        print("For node %s, ci best case: %f, 10p: %f, 25p: %f, median: %f, mean %f, 75ile %f, 90ile %f, 95ile %f, worst case: %f" % ( fname, sorted_times[0], sorted_times[ltimes/10], sorted_times[(25*ltimes)/100], sorted_times[ltimes/2], sum(sorted_times)/ltimes, sorted_times[(75*ltimes)/100], sorted_times[(90*ltimes)/100], sorted_times[(95*ltimes)/100], sorted_times[-1] ) )

        if "oper" in fname:
            for i in range(len(times)):
                if (times[i] > 0.01) or (ts[i] > 114586.8 and ts[i] < 114588):
                    print i, times[i], ts[i]

        if len(tputs) > 0:
            if "cmd" in fname:
                for i in range(len(tputs)):
                    if tputs[i] > 0.7:
                        print("FOR navc, tput: %f, ts: %f"%(tputs[i], ts[i]) )
            
            sorted_tput = sorted(tputs)
            ltimest = len(tputs)
            print("For node %s, Tput: 10p: %f, 25p %f, median %f, mean %f, 75ile %f, 90ile %f, 95ile %f"%( fname, sorted_tput[(10*ltimest)/100], sorted_tput[(25*ltimest)/100], sorted_tput[ltimest/2], sum(sorted_tput)/ltimest, sorted_tput[(75*ltimest)/100], sorted_tput[(90*ltimest)/100], sorted_tput[(95*ltimest)/100] ) )
            print("Len tputs: %i, Len TS: %i"%(len(tputs), len(ts)) )
            if "plan" in fname:
                plt.plot(ts[19:], tputs[18:], 'r*-.', label=fname + " Inter-arrival Time")
                plt.xlabel("Time")
                if "cmd" in fname:
                    plt.ylim(0.0, 1.0)
                plt.ylabel(fname+" Inter-arrival Time (s)")
                plt.title("nav2d Node : %s Inter-arrival"%(fname) )
                plt.legend()
                #plt.show()
        '''
        '''
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
            rel_ts = []
            start_ts = ts[0]
            for i in ts:
                rel_ts.append(i - ts[0])
            per_run_mapupd_ts.append(rel_ts)
            #print("SCAN COUNTS : ", scan_count, ", TS: ", ts, "\n \n")

        '''
            plot_agg(ts[1:], tputs, start_t, 2.0, end_t, "Inter-arrival Time", fname)
            sorted_tput = sorted(tputs)
            ltimest = len(tputs)
            print("For node %s, Tput: median %f, mean %f, 75ile %f, 90ile %f"%( fname, sorted_tput[ltimest/2], sum(sorted_tput)/ltimest, sorted_tput[(75*ltimest)/100], sorted_tput[(90*ltimest)/100] ) )
            #plot_agg(ts[1:], tputs, start_t, 10.0, end_t, "Inter-arrival Time", fname)
        '''
print("per_run_mapupd_ts : ", per_run_mapupd_ts)
print("PER RUN SCAN CT : ", per_run_scan_ct)
