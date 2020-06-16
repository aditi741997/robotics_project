# This Python file uses the following encoding: utf-8
import sys
import os
import matplotlib.pyplot as plt
import math
import plot_cdf

farr = [12, 15, 17, 19, 21, 23, 30, 45, 67, 80, 100] #Largec1
farr = [10, 15, 21, 23, 25, 30, 33, 38, 45, 80, 100] # Smallc1 1c
farr = [10, 15, 17, 20, 23, 26, 28, 30, 32, 33, 35, 40, 60, 80, 100] # Smallc1 2c
farr = [10, 14, 15, 16, 20, 30, 60]
farr = [9, 16, 23, 24, 25, 30, 55, 80]
farr = [10, 20, 30, 32, 34, 36, 60] 
farr = [7,11,16,20,30,60]

pre = ''

def read_actual_metric_file(fname,c3):
    # m1 is human.y div by 3*2*t.oz*t.ow/(2*t.ow*t.ow - 1)
    print "File : ", fname
    arr_m1 = []
    arr_m2 = [] # angle difference in radians
    if c3:
	c = 3
    else:
	c = 5
    with open(fname, 'r') as fm:
        fml = fm.readlines()
        for i in range((len(fml)/c)):
            hpp = c*i
            try:
                #human y coordinate
                # hy = float(fml[hpp+2].split(' ')[-1][:-1])
                hy = float(fml[hpp].split(' ')[-2])
                
                # tb3 cos theta/2
                # tc = float(fml[hpp+17].split(' ')[-2])
                if c3:
		    tc = float(fml[hpp+2].split(' ')[-2])
		else:
		    tc = float(fml[hpp+4].split(' ')[-2])

                # tb3 sin theta/2
                # ts = float(fml[hpp+16].split(' ')[-1][:-1])
                if c3:
		    ts = float(fml[hpp+1].split(' ')[-2])
		else:
		    ts = float(fml[hpp+3].split(' ')[-1][:-1])
                
                ty = ts*float((3.0*2*tc)/((2*tc*tc) - 1.0))
		arr_m1.append(abs(ty-hy))
                h_theta = math.atan(hy/3.0)
                t_theta = 2*math.atan(ts/tc)
                t_theta1 = math.atan(ty/3.0)
                arr_m2.append(abs(t_theta1 - h_theta))
            #if i%500 == 3:
                #print i, hpp, hy, tc, ts, ty, arr_m2[-1]
            except:
                print "ERRORRR!! ", c3, hpp, fml[hpp].split(' '), fml[hpp+2].split(' '), fml[hpp+1].split(' ')
		return (1,1)
    arr_m1 = arr_m1[(len(arr_m1)/50):]
    arr_m2 = arr_m2[(len(arr_m2)/50):]
    sarr = sorted(arr_m1)
    sarr2 = sorted(arr_m2)
    ans = (sarr[(95*len(sarr))/100], sarr[len(sarr)/2], sum(sarr)/len(sarr), sarr[(99*len(sarr))/100])
    ans2 = (sarr2[(95*len(sarr2))/100], sarr2[len(sarr2)/2], sum(sarr2)/len(sarr2), sarr2[(99*len(sarr2))/100])
    print "Ignored the first 2% of data." , (ans, ans2)
    return (ans, ans2)

# read_actual_metric_file("2c_max5_new_perf_1008.out")

def cmps(x):
    return x[0]

def subtract_min(x):
    # find the smallest elem in array and subtract it from all elements
    if (len(x) > 0):
        m = x[0]
        for i in range(len(x)):
            if x[i] < m:
                m = x[i]
        # subtract from all elems :
        new_arr = [(y-m) for y in x]
        return new_arr
    else:
        return x

if __name__ == '__main__':
    pre = sys.argv[2]
    cpp = int(sys.argv[5])
    need_actual_freq = (int(sys.argv[7]) == 1) # we dont need new_freq for RTC and DynamicAlgo.
    #farr = farr if need_actual_freq else [15]
    fname = sys.argv[8]

    ind = {}
    for i in range(len(farr)):
        ind[farr[i]] = i

    opt_freq = int(sys.argv[3])
    rel_perf_tail_improv_wrt_low_freq = []
    abs_perf_tail_improv_wrt_low_freq = []
    rel_perf_med_improv_wrt_low_freq = []
    abs_perf_med_improv_wrt_low_freq = []

    # human_speed_arr = [dist/24, dist/20, dist/16, dist/12, dist/8, dist/4, dist/3, dist/2]
    t_arr = [int(sys.argv[9])]
    t_ind = {t_arr[0] : 0}

    abs_deg_arr = []

    for t in t_arr:
        perc_m = [0.0 for x in farr]
        med_m = [0.0 for x in farr]
        mean_m = [0.0 for x in farr]
	p9_m = [0.0 for x in farr]

        perc_rxn = [0.0 for x in farr]
        med_rxn = [0.0 for x in farr]
        mean_rxn = [0.0 for x in farr]
	p9_rxn = [0.0 for x in farr]        

        new_farr = [0.0 for x in farr]

        m_rxn_ratio = [1.0 for x in farr]

        if need_actual_freq:
            with open(sys.argv[1], 'r') as af:
                afl = af.readlines()
                for l in afl:
                    ls = l.split(' ')
                    freq = int(ls[0])
                    if int(ls[1]) == t and freq in farr:
                        new_farr[ind[freq]] = float(ls[-1][:-1])
        else:
            new_farr = farr

        # Measuring from gz model states :
        # m2 : if tb3 pointing towards (x,y1) and human at (x,y2) then y1/y2
        # m1 : tb3_theta - human_theta absolute (diff in radians of orientation)
        perc_m1 = [0.0 for x in farr]
        med_m1 = [0.0 for x in farr]
        mean_m1 = [0.0 for x in farr]
	p9_m1 = [0.0 for x in farr]

        perc_newm1 = [0.0 for x in farr]
        med_newm1 = [0.0 for x in farr]
        mean_newm1 = [0.0 for x in farr]    
	p9_newm1 = [0.0 for x in farr]

        perc_m2 = [0.0 for x in farr]
        med_m2 = [0.0 for x in farr]
        mean_m2 = [0.0 for x in farr]
	p9_m2 = [0.0 for x in farr]

        perc_lat =  [0.0 for x in farr]
        med_lat = [0.0 for x in farr]
        mean_lat = [0.0 for x in farr]
	p9_lat = [0.0 for x in farr]

        perc_tput =  [0.0 for x in farr]
        med_tput = [0.0 for x in farr]
        mean_tput = [0.0 for x in farr]
	p9_tput = [0.0 for x in farr]

        td_perc_lat =  [0.0 for x in farr]
        td_med_lat = [0.0 for x in farr]
        td_mean_lat = [0.0 for x in farr]
	td_p9_lat = [0.0 for x in farr]

        td_perc_rxn = [0.0 for x in farr]
        td_med_rxn = [0.0 for x in farr]
        td_mean_rxn = [0.0 for x in farr]
	td_p9_rxn = [0.0 for x in farr]

        runs = [1,2,3,4,5] #,6, 7, 8] #, , 9, 10,11,12,13,14,15,16,17,18]        
        for f in farr:
	    actual_run_count = 0
            for r in runs:
                # read tracker log to find metric vals.
		# first check the hit rate in vision file.
		hr = 0.0
		with open('%s_vision_node_%i.%i.%s.out'%(pre, r, f, str(t)), 'r') as fil:
		    for l in fil.readlines():
			if cpp == 1 and "Hit rate :" in l:
				hr = float(l.split(' ')[7][:-1])
		#if (f == 6):
			#print "Fixing hr for f 6"
			#hr = 0.7 # since at 6Hz for exp2_2c : always out of frame
                with open('%s_tracker_node_%i.%i.%s.out'%(pre, r, f, str(t)), 'r') as fil:
                    print "Reading for : ", f, t, r
                    for l in fil.readlines():
                        larr = l.split(' ')
                        if cpp == 0:
                            if "perf_metric" in l:
                                mean_m[ind[f]] = float(larr[7][:-1])
                                med_m[ind[f]] = float(larr[8][:-1])
                                perc_m[ind[f]] = float(larr[9][:-1])
                            elif "rxn_time" in l:
                                mean_rxn[ind[f]] = float(larr[6][:-1])
                                med_rxn[ind[f]] = float(larr[7][:-1])
                                perc_rxn[ind[f]] = float(larr[8][:-1])
                            elif "of latency" in l:
                                mean_lat[ind[f]] = float(larr[6][:-1])
                                med_lat[ind[f]] = float(larr[7][:-1])
                                perc_lat[ind[f]] = float(larr[8][:-1])
                            elif "of tput" in l:
                                mean_tput[ind[f]] = float(larr[6][:-1])
                                med_tput[ind[f]] = float(larr[7][:-1])
                                perc_tput[ind[f]] = float(larr[8][:-1])
                        else:
                            if "Metric" in l and "Metric1" not in l:
                                m_mean = float(larr[10])
                                m_med = float(larr[11])
                                m_perc = float(larr[12])
				m_p9 = float(larr[26][:-1])
                            elif "RxnTime" in l:
                                meanrxn = float(larr[10])
                                medrxn = float(larr[11])
                                percrxn = float(larr[12])
				p9rxn = float(larr[26][:-1])
                            elif "N3 latency" in l:
                                meanlat = float(larr[11])
                                medlat = float(larr[12])
                                perclat = float(larr[13])
				p9lat = float(larr[27][:-1])
                            elif "Tput" in l:
                                meantput = float(larr[10])
                                medtput = float(larr[11])
                                perctput = float(larr[12])
				p9tput = float(larr[26][:-1])
                            elif "Metric1" in l:
                                meannewm1 = float(larr[10])
                                mednewm1 = float(larr[11])
                                percnewm1 = float(larr[12])
				p9newm1 = float(larr[26][:-1])
                            elif "N3 Lat w.r.t. TDNode" in l:
                                td_meanlat = float(larr[13])
                                td_medlat = float(larr[14])
                                td_perclat = float(larr[15])
				td_p9lat = float(larr[29][:-1])
                            elif "RxnTm w.r.t." in l:
                                td_meanrxn = float(larr[12])
                                td_medrxn = float(larr[13])
                                td_percrxn = float(larr[14])
				td_p9rxn = float(larr[28][:-1])
		rrr = True
                #(a,b) = read_actual_metric_file('%s_perf_%i.%i.%s.out'%(pre, r, f, str(t)), rrr)
                m_arr = plot_cdf.read_new_abs_deg_metric('%s_perf_%i.%i.%s.out'%(pre, r, f, str(t)))
                sm_arr = sorted(m_arr)
                lsm_arr = len(sm_arr)
                b = (sm_arr[(95*lsm_arr)/100], sm_arr[lsm_arr/2], sum(sm_arr)/lsm_arr, sm_arr[(99*lsm_arr)/100])
		# add the value of this run to the metric arr of len(farr)
                if hr > 0.0:
			actual_run_count += 1
			mean_lat[ind[f]] += meanlat
			med_lat[ind[f]] += medlat
			perc_lat[ind[f]] += perclat
			p9_lat[ind[f]] += p9lat

			td_mean_lat[ind[f]] += td_meanlat
			td_med_lat[ind[f]] += td_medlat
			td_perc_lat[ind[f]] += td_perclat
			td_p9_lat[ind[f]] += td_p9lat

			mean_rxn[ind[f]] += meanrxn
			med_rxn[ind[f]] += medrxn
			perc_rxn[ind[f]] += percrxn
			p9_rxn[ind[f]] += p9rxn

			td_mean_rxn[ind[f]] += td_meanrxn
			td_med_rxn[ind[f]] += td_medrxn
			td_perc_rxn[ind[f]] += td_percrxn
			td_p9_rxn[ind[f]] += td_p9rxn		

			mean_tput[ind[f]] += meantput
			med_tput[ind[f]] += medtput
			perc_tput[ind[f]] += perctput
			p9_tput[ind[f]] += p9tput

			mean_m[ind[f]] += m_mean
			med_m[ind[f]] += m_med
			perc_m[ind[f]] += m_perc
			p9_m[ind[f]] += m_p9

			mean_newm1[ind[f]] += meannewm1
			med_newm1[ind[f]] += mednewm1
			perc_newm1[ind[f]] += percnewm1
			p9_newm1[ind[f]] += p9newm1

			perc_m1[ind[f]] += b[0]
			med_m1[ind[f]] += b[1]
			mean_m1[ind[f]] += b[2]
			p9_m1[ind[f]] += b[3]

			'''
			perc_m2[ind[f]] += a[0]
			med_m2[ind[f]] += a[1]
			mean_m2[ind[f]] += a[2]
			p9_m2[ind[f]] += a[3]
			'''
		else:
			print "Rejecting run ", r, " for freq, t : ", f, t, pre
            # Divide all metric by len(runs)
            mean_m[ind[f]] /= actual_run_count
            med_m[ind[f]] /= actual_run_count
            perc_m[ind[f]] /= actual_run_count
	    p9_m[ind[f]] /= actual_run_count

            mean_rxn[ind[f]] /= actual_run_count
            med_rxn[ind[f]] /= actual_run_count
            perc_rxn[ind[f]] /= actual_run_count
	    p9_rxn[ind[f]] /= actual_run_count

            mean_tput[ind[f]] /= actual_run_count
            med_tput[ind[f]] /= actual_run_count
            perc_tput[ind[f]] /= actual_run_count
	    p9_tput[ind[f]] /= actual_run_count

            mean_newm1[ind[f]] /= actual_run_count
            med_newm1[ind[f]] /= actual_run_count
            perc_newm1[ind[f]] /= actual_run_count
	    p9_newm1[ind[f]] /= actual_run_count

            mean_lat[ind[f]] /= actual_run_count
            med_lat[ind[f]] /= actual_run_count
            perc_lat[ind[f]] /= actual_run_count
	    p9_lat[ind[f]] /= actual_run_count

            td_mean_lat[ind[f]] /= actual_run_count
            td_med_lat[ind[f]] /= actual_run_count
            td_perc_lat[ind[f]] /= actual_run_count
	    td_p9_lat[ind[f]] /= actual_run_count

            td_mean_rxn[ind[f]] /= actual_run_count
            td_med_rxn[ind[f]] /= actual_run_count
            td_perc_rxn[ind[f]] /= actual_run_count
	    td_p9_rxn[ind[f]] /= actual_run_count

            perc_m1[ind[f]] /= actual_run_count
            med_m1[ind[f]] /= actual_run_count
            mean_m1[ind[f]] /= actual_run_count
	    p9_m1[ind[f]] /= actual_run_count

	    perc_m2[ind[f]] /= actual_run_count
            med_m2[ind[f]] /= actual_run_count
            mean_m2[ind[f]] /= actual_run_count
	    p9_m2[ind[f]] /= actual_run_count

            with open(fname, 'a') as f1:
                f1.write('%i %i Averaging over %i runs out of 5 #\n'%(f, t, actual_run_count))
		f1.write('%i %i 10RunAvg N3Latency 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, p9_lat[ind[f]], perc_lat[ind[f]], med_lat[ind[f]], mean_lat[ind[f]]))
                f1.write('%i %i 10RunAvg N3Latency w.r.t. TDNode 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, td_p9_lat[ind[f]], td_perc_lat[ind[f]], td_med_lat[ind[f]], td_mean_lat[ind[f]]))
                f1.write('%i %i 10RunAvg Tput 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, p9_tput[ind[f]], perc_tput[ind[f]], med_tput[ind[f]], mean_tput[ind[f]]))
                f1.write('%i %i 10RunAvg RxnTime 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, p9_rxn[ind[f]], perc_rxn[ind[f]], med_rxn[ind[f]], mean_rxn[ind[f]]))
                f1.write('%i %i 10RunAvg RxnTime w.r.t. TDNode 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, td_p9_rxn[ind[f]], td_perc_rxn[ind[f]], td_med_rxn[ind[f]], td_mean_rxn[ind[f]]))
                f1.write('%i %i 10RunAvg Perf Rel. Metric 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, p9_m[ind[f]], perc_m[ind[f]], med_m[ind[f]], mean_m[ind[f]]))
                f1.write('%i %i 10RunAvg Perf Rel. Metric1 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, p9_newm1[ind[f]], perc_newm1[ind[f]], med_newm1[ind[f]], mean_newm1[ind[f]]))
                f1.write('%i %i 10RunAvg Perf Abs Deg Metric 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, p9_m1[ind[f]], perc_m1[ind[f]], med_m1[ind[f]], mean_m1[ind[f]]))
                f1.write('%i %i 10RunAvg Perf Abs Distance Metric 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, p9_m2[ind[f]], perc_m2[ind[f]], med_m2[ind[f]], mean_m2[ind[f]]))

        abs_deg_arr.append(perc_m1)

        print new_farr
	lw = 4.5
	fs = 27
	mas=10
        tailpc='m^-.'
	xaxis='Frequency'
	legloc='lower right'
	legsz=25
	legcolsp=0.4
	legtextpad=0.2
	#plt.figure(figsize=(3,1))
	'''
	p1 = plt.plot(new_farr, perc_m, 'ro-', markersize=mas, linewidth=lw, label='95ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, p9_m, tailpc, markersize=mas, linewidth=lw, label='99ile')
	plt.plot(new_farr, med_m, 'g.:', markersize=mas, linewidth=lw, label='Median')
        plt.plot(new_farr, mean_m, 'b*--', markersize=mas, linewidth=lw, label='Mean')
        #plt.title('Metric at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel(xaxis, fontsize=fs)
        plt.ylabel(r'$\Delta$ Rel Metric (offset)', fontsize=fs)
        plt.ylim(0.0, 1.0)
	plt.xticks(fontsize=fs)
	plt.yticks(fontsize=fs)
        plt.legend(loc=legloc, prop={"size":legsz}, ncol=2, columnspacing=legcolsp, handletextpad=legtextpad)
	plt.tight_layout()
	fig = plt.gcf()
        print plt.rcParams["figure.figsize"]
	plt.rcParams["figure.figsize"][0] = 3.
	plt.rcParams["figure.figsize"][1] = 1.
	#fig.set_size_inches(5.,1.)
	print fig.get_size_inches(), plt.rcParams["figure.figsize"]
	plt.show()
	fig.savefig('Final_RelMetric_Default_O4.pdf')

        p5 = plt.plot(new_farr, perc_newm1, 'ro-', markersize=mas, linewidth=lw, label='95ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, p9_newm1, tailpc, markersize=mas, linewidth=lw, label='99ile')
	plt.plot(new_farr, med_newm1, 'g.:', markersize=mas, linewidth=lw, label='Median')
        plt.plot(new_farr, mean_newm1, 'b*--', markersize=mas, linewidth=lw, label='Mean')
        #plt.title('Metric1 at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel(xaxis, fontsize=fs)
        plt.ylabel('Rel Metric1 (offset)', fontsize=fs)
        plt.ylim(0.0, 1.0)
        plt.xticks(fontsize=fs)
	plt.yticks(fontsize=fs)
        plt.legend(loc=legloc, prop={"size":legsz}, ncol=2, columnspacing=legcolsp, handletextpad=legtextpad)
        plt.tight_layout()
	plt.show()
	'''

	'''
	fig = plt.figure(1)
	axes = fig.add_subplot(111)
	fig.tight_layout()
	#fig,axes=plt.subplots(nrows=1, ncols=1,figsize=(4,3))
	'''
	print plt.rcParams["legend.handlelength"], "legend handle len, see if can be reduced a bit"
	plt.figure(figsize=(8.,4.5),dpi=120)
	plt.plot(new_farr, perc_m1, 'ro-', markersize=mas, linewidth=lw, label='95ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, p9_m1, tailpc, markersize=mas, linewidth=lw, label='99ile')
	plt.plot(new_farr, med_m1, 'g.:', markersize=mas, linewidth=lw, label='Median')
        plt.plot(new_farr, mean_m1, 'b*--', markersize=mas, linewidth=lw, label='Mean')
        #plt.title('Absolute Deg Diff at displacement time : %f'%(t))
        plt.xlabel(xaxis, fontsize=fs)
        plt.ylabel(r'$\Delta$ Degree (rad)', fontsize=fs)
        plt.ylim(0, 3.2)
	#axes.axis(ymin=0.,ymax=3.2)
	#axes.tick_params(axis="x",labelsize=fs-3)
	#axes.tick_params(axis="y", labelsize=fs-3)
        xloc, xlab = plt.xticks(fontsize=fs-1)
	plt.yticks(fontsize=fs-3)
	plt.legend(loc=legloc, prop={"size":legsz}, ncol=2, handlelength=1.5, columnspacing=legcolsp, handletextpad=legtextpad)       
	plt.grid()
	plt_arrow = plt.arrow(opt_freq,1.5, 0,-0.5, head_width=1, head_length=0.2, length_includes_head=True, fc='k',ec='k')
	#print plt_arrow.shape, plt_arrow.head_length, plt_arrow.head_width
	plt.tight_layout()
        #fig.tight_layout()
	#fig.subplots_adjust(bottom = 0)
	#fig.subplots_adjust(top = 1)
	'''
	fig = plt.gcf()
        fig.set_size_inches(2.,.6)
        print fig.get_size_inches()
        '''
	fig=plt.gcf()
	plt.show()
        fig.savefig('Final_AbsDegMetric_Default_O4.pdf')

        p1 = plt.plot(new_farr, perc_m2, 'ro-', label='99ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, p9_m2, tailpc, label='99ile')
        plt.plot(new_farr, med_m2, 'g.:', label='Median')
        plt.plot(new_farr, mean_m2, 'b*--', label='Mean')
        #plt.title('Absolute Distance b/w Cam & Obj at displacement time : %f'%(t))
        plt.xlabel(xaxis, fontsize=fs)
        plt.ylabel('Abs Metric : Distance Diff', fontsize=fs)
        plt.ylim(0, 5)
        plt.xticks(fontsize=fs)
        plt.yticks(fontsize=fs)
        plt.legend(loc=legloc, prop={"size":legsz}, ncol=2, handlelength=1.5, columnspacing=legcolsp, handletextpad=legtextpad)
        plt.show()

	plt.figure(figsize=(8.,4.5),dpi=120)
        p2 = plt.plot(new_farr, perc_rxn, 'ro-', markersize=mas, linewidth=lw, label='95ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, p9_rxn, tailpc, markersize=mas, linewidth=lw, label='99ile')
	plt.plot(new_farr, med_rxn, 'g.:', markersize=mas, linewidth=lw, label='Median')
        plt.plot(new_farr, mean_rxn, 'b*--', markersize=mas, linewidth=lw, label='Mean')
        #plt.title('RxnTime at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel(xaxis, fontsize=fs)
        plt.ylabel('RT (sec)', fontsize=fs)
        plt.ylim(0, 0.45)
        plt.xticks(fontsize=fs-1)
        plt.yticks(fontsize=fs-2)
        plt.legend(loc=legloc, prop={"size":legsz}, ncol=2, columnspacing=legcolsp, handletextpad=legtextpad)
	plt.grid()
	plt_arrow = plt.arrow(opt_freq,0.35, 0,-0.08, head_width=1, head_length=0.03, length_includes_head=True, fc='k',ec='k')

	plt.tight_layout()
	'''
	fig = plt.gcf()
        fig.set_size_inches(2.,.6)
        print fig.get_size_inches()
        '''
        fig=plt.gcf()
	plt.show()
	fig.savefig('Final_RxnTime_Default_O4.pdf')

        p2 = plt.plot(new_farr, td_perc_rxn, 'ro-', markersize=9, linewidth=lw, label='95ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, td_p9_rxn, tailpc, markersize=9, linewidth=lw, label='99ile')
	plt.plot(new_farr, td_med_rxn, 'g.:', markersize=9, linewidth=lw, label='Median')
        plt.plot(new_farr, td_mean_rxn, 'b*--', markersize=9, linewidth=lw, label='Mean')
        #plt.title('TD RxnTime at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel('Publisher Frequency', fontsize=fs)
        plt.ylabel('TD RxnTime', fontsize=fs)
        plt.ylim(0, 0.45)
        plt.xticks(fontsize=fs)
        plt.yticks(fontsize=fs)
        plt.legend(loc=legloc, prop={"size":legsz}, ncol=2, columnspacing=legcolsp, handletextpad=legtextpad)
	fig = plt.gcf()
        fig.set_size_inches(2.,1.)
        print fig.get_size_inches()
        plt.show()
        fig.savefig('Final_TDRxnTime_Default_O4.pdf')

        new_perc_rxn = subtract_min(perc_rxn)
        new_med_rxn = subtract_min(med_rxn)
        new_mean_rxn = subtract_min(mean_rxn)

        new_td_perc_rxn = subtract_min(td_perc_rxn)
        new_td_med_rxn = subtract_min(td_med_rxn)
        new_td_mean_rxn = subtract_min(td_mean_rxn)

        plt.plot(new_farr, new_med_rxn, 'ro-', label='Median RxnTime')
        plt.plot(new_farr, new_td_med_rxn, 'g*:', label='Median TD RxnTime')
        plt.title('Median RxnTime vs RT w.r.t. TD publisher')
        plt.xlabel('Publisher freq')
        plt.legend()
        plt.show()

        plt.plot(new_farr, new_mean_rxn, 'ro-', label='Mean RxnTime')
        plt.plot(new_farr, new_td_mean_rxn, 'g*:', label='Mean TD RxnTime')
        plt.title('Mean RxnTime vs RT w.r.t. TD publisher')
        plt.xlabel('Publisher freq')
        plt.legend()
        plt.show()

        plt.plot(new_farr, new_perc_rxn, 'ro-', label='Tail RxnTime')
        plt.plot(new_farr, new_td_perc_rxn, 'g*:', label='Tail TD RxnTime')
        plt.title('Tail RxnTime vs RT w.r.t. TD publisher')
        plt.xlabel('Publisher freq')
        plt.legend()
        plt.show()

        # plt.plot(new_farr, m_rxn_ratio, 'g*:', label='Ratio')
        # plt.title('Ratio of Median metric to median rxn time at c1 : %s'%(sys.argv[4]))
        # plt.xlabel('Publisher Frequency')
        # plt.ylabel('Ratio')
        # plt.show()
        p2 = plt.plot(new_farr, perc_lat, 'ro-', label='95ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, p9_lat, 'y^:', label='99ile')
	plt.plot(new_farr, med_lat, 'g.:', label='Median')
        plt.plot(new_farr, mean_lat, 'b*--', label='Mean')
        plt.title('Latency at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Latency')
        plt.ylim(0, 0.2)
        plt.legend()
        plt.show()

        p2 = plt.plot(new_farr, td_perc_lat, 'ro-', label='95ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, td_p9_lat, 'y^:', label='99ile')
	plt.plot(new_farr, td_med_lat, 'g.:', label='Median')
        plt.plot(new_farr, td_mean_lat, 'b*--', label='Mean')
        plt.title('TD Latency at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Latency')
        plt.ylim(0, 0.2)
        plt.legend()
        plt.show()

        p2 = plt.plot(new_farr, perc_tput, 'ro-', label='95ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, p9_tput, 'y^:', label='99ile')
	plt.plot(new_farr, med_tput, 'g.:', label='Median')
        plt.plot(new_farr, mean_tput, 'b*--', label='Mean')
        plt.title('Tput at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Tput')
        plt.ylim(0, 0.14)
        plt.legend()
        plt.show()

        '''
        x = zip(med_rxn, med_m)
        x.sort()
        print x
        rtime = [y[0] for y in x]
        m = [y[1] for y in x]
        plt.plot(rtime, m, 'bo:', label='Metric')
        plt.title('Median RxnTime vs Median Perf Metric, c1 : %s'%(sys.argv[4]))
        plt.xlabel('Median RxnTime')
        plt.ylabel('Median Metric')
        plt.xlim(0, 0.2)
        plt.show()

        x = zip(perc_rxn, med_m)
        x.sort()
        print x
        rtime = [y[0] for y in x]
        m = [y[1] for y in x]
        plt.plot(rtime, m, 'bo:', label='Metric')
        plt.title('Tail RxnTime vs Median Perf Metric, c1 : %s'%(sys.argv[4]))
        plt.xlabel('Tail RxnTime')
        plt.ylabel('Median Metric')
        plt.xlim(0, 0.25)
        plt.show()
        '''
        
        #rel_perf_tail_improv_wrt_low_freq.append(perc_m[ind[10]]/perc_m[ind[opt_freq]])
        # abs_perf_tail_improv_wrt_low_freq.append(perc_m1[ind[10]]/perc_m1[ind[opt_freq]])

        #rel_perf_med_improv_wrt_low_freq.append(med_m[ind[10]]/med_m[ind[opt_freq]])
        # abs_perf_med_improv_wrt_low_freq.append(med_m1[ind[10]]/med_m1[ind[opt_freq]])
    # s_arr = ['o--', '*--', '^:', '<:', '>--']
    # for t in sorted(t_ind.keys()):
    #     plt.plot(new_farr, abs_deg_arr[t_ind[t]], s_arr[t_ind[t]], label='Speed:'+str(t/8.0))
    #     plt.title('Perf(Abs DegDiff) w.r.t diff obj speeds at cam speed %s'%(sys.argv[6]))
    #     plt.xlabel('Publisher Freq')
    #     plt.ylabel('Perf : ABsolute Deg Dif')
    #     plt.ylim(0.0, 3.2)
    #     plt.legend()
    #     plt.show()

    # plt.plot(human_speed_arr, rel_perf_tail_improv_wrt_low_freq, 'r-', label='Ratio of 99ile')
    # plt.plot(human_speed_arr, rel_perf_med_improv_wrt_low_freq, 'g:', label='Ratio of Median')
    # plt.title('Ratio of relative offset at 10 & 70Hz (~Opt) w.r.t Object speed')
    # plt.xlabel('Object Speed')
    # plt.ylabel('Ratio')
    # plt.legend()
    # plt.show()

    # plt.plot(human_speed_arr, abs_perf_tail_improv_wrt_low_freq, 'r-', label='Ratio of 99ile')
    # plt.plot(human_speed_arr, abs_perf_med_improv_wrt_low_freq, 'g:', label='Ratio of Median')
    # plt.title('Ratio of absolute offset at 10 & 70Hz (~Opt) w.r.t Object speed')
    # plt.xlabel('Object Speed')
    # plt.ylabel('Ratio')
    # plt.legend()
    # plt.show()
