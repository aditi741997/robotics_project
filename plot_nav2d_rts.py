import sys
import matplotlib.pyplot as plt
import numpy as np
import random
import copy

rt_fname_pre = sys.argv[1]

start_t = float(sys.argv[2])
end_t = float(sys.argv[3])

exp_cc_tput = float(sys.argv[4])

def plot_smt(x,y,sty,n,chain,yl):
	plt.plot(x,y,sty, label=n)
	plt.title('Nav2D %s for chain %s'%(n, chain) )
	if yl > 0.0:
		plt.ylim(0.0, yl)
	plt.legend()
	plt.xlabel('Time')
	plt.ylabel(n + ' in s.')
	plt.show()

	#plot_agg(x,y,start_t,2.0,end_t,n, chain)
	#plot_agg(x,y,start_t,10.0,end_t,n, chain)

def plot_agg(x,y,start,slot,end, nm, chain):
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
			agg_avg.append ( sum(i)/len(i) )
			agg_end1.append(agg_end[ind])
			agg_75p.append( arr[(75*len(i))/100] )
		ind += 1
	plt.plot(agg_end1, agg_avg, 'b*:', markersize=8, label="Avg " + nm)
	plt.plot(agg_end1, agg_25p, 'g^-.', markersize=8, label="25p " + nm)
	plt.plot(agg_end1, agg_75p, 'ro:', markersize=8, label="75p " + nm)
	plt.xlabel('Time')
	plt.title('Aggregate %s for chain %s over %fs period'%( nm, chain, slot) )
	plt.ylabel(nm + " in s.")
	plt.legend()
	plt.show() 
	


yls = {"Scan_MapCB_MapU_NavP_NavC_LP" : 6.0, "Odom_LP": 0.4, "Scan_LC_LP" : 0.4, "Scan_MapCB_NavCmd_LP" : 4.0, "Scan_MapCB_NavPlan_NavCmd_LP" : 4.0}

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
                                print "Adding ind ", i, " TS: ", ts_arr[i], " to slot from ", curr_ts
                                i += 1
                        else:
                                break
		print "Done with this slot, moving to next, i=", i
                curr_ts += slot
                start_arr.append(curr_ts)
                new_m_arr.append(this_slot)
                new_ts_arr.append(this_slot_ts)
        return new_m_arr, start_arr, new_ts_arr

start_run_ind = int(sys.argv[5])
end_run_ind = int(sys.argv[6])

aggregate_chain_lats = {}
aggregate_chain_rts = {}

per_run_chain_lats = {}
per_run_chain_tputs = {}
per_run_chain_rts = {}
per_run_chain_ts = {}

nav_chains = ("nav" in sys.argv[7])
yolo_chains = ("yolo" in sys.argv[7])

arr_chains = ["Scan_LC_LP", "Scan_MapCB_MapU_NavP_NavC_LP", "Scan_MapCB_NavCmd_LP", "Scan_MapCB_NavPlan_NavCmd_LP"] if (not nav_chains and not yolo_chains) else ["Odom_NC", "Odom_NP"] if nav_chains else ["PPCam_Yolo"]

fname_append = sys.argv[7] #"_nav" if nav_chains else ""

high_lat_thresh = float(sys.argv[8])

runs = range(start_run_ind,end_run_ind+1)

for run in runs: #range(start_run_ind,end_run_ind+1):
        #for chain in ["Scan_LC_LP", "Scan_MapCB_MapU_NavP_NavC_LP", "Scan_MapCB_NavCmd_LP", "Scan_MapCB_NavPlan_NavCmd_LP"]:
	for chain in arr_chains:
                print "Starting chain", chain
                if chain not in aggregate_chain_lats:
                    aggregate_chain_lats[chain] = []
		    aggregate_chain_rts[chain] = []
                    per_run_chain_lats[chain] = []
                    per_run_chain_tputs[chain] = []
                    per_run_chain_rts[chain] = []
                    per_run_chain_ts[chain] = []
                rts = []
                lats = []
                tputs = []
                ts = []
                lat_txt = "Lat_" if (nav_chains or yolo_chains) else "Latency"
                with open(rt_fname_pre + "_run" + str(run) + fname_append + "_rt_stats.txt", 'r') as f:
                        fl = f.readlines()
                        for l in fl:
                                if chain in l:
                                        #if "Latency" in l:
                                        if lat_txt in l:
					        lats += [ float(x) for x in l.split(' ')[1:-1] ]
                                        elif "Tput" in l:
                                                tputs += [ float(x) for x in l.split(' ')[1:-1] ]
                                        elif "RT_" in l:
                                                rts += [ float(x) for x in l.split(' ')[1:-1] ]
                                        elif "TS_" in l:
                                                ts += [ float(x) for x in l.split(' ')[1:-1] ]
                                #else:
                                        #print "Chain not in Line", l
                # len(TS) should be = len(lat).
                # len(TS) should be len(tput)+1.
                #print("\n \n ARR Lat FOR chain %s : %s"%( chain, str(lats) ) )
                #if "Scan_LC" in chain:
                        #print("\n \n ARR RT FOR chain %s : %s"%( chain, str(rts) ) )
                print(chain, len(lats))
                per_run_chain_lats[chain].append(copy.deepcopy(lats))
                per_run_chain_tputs[chain].append(copy.deepcopy(tputs))
                per_run_chain_rts[chain].append(copy.deepcopy(rts))
                per_run_chain_ts[chain].append(copy.deepcopy(ts))
                
                high_lats = filter(lambda x: x[0] > high_lat_thresh, zip(lats, range(len(lats))) )
                if len(high_lats) > 0:
                    print(high_lats, " for ", run, fname_append)

                if len(ts) > 1:
                        '''
                        plot_smt(ts, lats, 'bo:', " Latency", chain, yls[chain])
                        '''
                        s_rt = sorted(rts)
                        #print(s_rt[(-1*len(rts)/10):])
                        print("max val of rt: %f, lat: %f, tput: %f"%(max(rts), max(lats), max(tputs)) )

                        print("Chain Tput 25p %f, 50p %f, 75p %f, 90p %f, 95p %f "% ( np.percentile(tputs, 25), np.percentile(tputs, 50), np.percentile(tputs, 75), np.percentile(tputs, 90), np.percentile(tputs, 95) ) )

                        #plot_smt(ts[1:], rts, 'r^:', " RT", chain, yls[chain])
                        #plot_smt(ts[1:], tputs, 'g*:', " Inter-arrival Times", chain, yls[chain])
                        if "Scan_LC" in chain:
                                bad_tputs = filter(lambda x: x[0] > 1.1*exp_cc_tput, zip(tputs, ts[1:]) )
                                vgood_tputs = filter(lambda x: x[0] < 0.9*exp_cc_tput, zip(tputs, ts[1:]))
                                print("FOR CRITICAL CHAIN, Total tputs: %i, bad_tputs: %i, vgood_tputs: %i"%(len(tputs), len(bad_tputs), len(vgood_tputs) ) )
                                print("FOR CRITICAL CHAIN, BAD Tputs: ", bad_tputs)
                        '''
                        '''
		newrts = copy.deepcopy(rts)
                random.shuffle(newrts)
		aggregate_chain_rts[chain] += newrts[:100]
                newlats = copy.deepcopy(lats)
                random.shuffle(newlats)
		aggregate_chain_lats[chain] += newlats[:200]


#per_chain_count = {"Scan_LC_LP": 1000, "Scan_MapCB_MapU_NavP_NavC_LP": 1000, "Scan_MapCB_NavCmd_LP": 10, "Scan_MapCB_NavPlan_NavCmd_LP": 10}
per_chain_count = {}
for ch in arr_chains:
    per_chain_count[ch] = 1

for k in per_run_chain_lats:
    print(k, "LAT ORDERED WRT TIME:", per_run_chain_lats[k])
    print('---')

if yolo_chains:
    for k in per_run_chain_lats:
        print(k, "RT WRT TIME:", per_run_chain_rts[k])
        print("----")
        print(k, "Tput WRT TIME:", per_run_chain_tputs[k])
        print("----")
        print(k, "TS:", per_run_chain_ts[k])
        print("----")

for ch in per_chain_count:
    random.shuffle(aggregate_chain_lats[ch])
    lch = len(aggregate_chain_lats[ch])
    print("For runs: ", runs, "RANDOM SUBSET: ", ch, aggregate_chain_lats[ch])
    #print(aggregate_chain_lats[ch][:lch/5], ch)
    #print("AGGREGATE LAT for chain ", ch, " median: %f, 75ile: %f, 95ile: %f"%(np.median(aggregate_chain_lats[ch]), np.percentile(aggregate_chain_lats[ch], 75), np.percentile(aggregate_chain_lats[ch], 95) ) )
    print('-')
    print('-')
