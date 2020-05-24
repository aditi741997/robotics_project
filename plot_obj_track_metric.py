import sys
import os
import matplotlib.pyplot as plt
import math

farr = [12, 15, 17, 19, 21, 23, 30, 45, 67, 80, 100] #Largec1
farr = [10, 15, 21, 23, 25, 30, 33, 38, 45, 80, 100] # Smallc1 1c
farr = [10, 15, 17, 20, 23, 26, 28, 30, 32, 33, 35, 40, 60, 80, 100] # Smallc1 2c
farr = [9, 16, 23, 24, 25, 30, 55, 80]
farr = [10, 14, 15, 16, 20, 30, 60]

pre = ''

def read_actual_metric_file(fname):
    # m1 is human.y div by 3*2*t.oz*t.ow/(2*t.ow*t.ow - 1)
    print "File : ", fname
    arr_m1 = []
    arr_m2 = [] # angle difference in radians
    c = 5 #18 (num of lines for each model state)
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
                tc = float(fml[hpp+4].split(' ')[-2])

                # tb3 sin theta/2
                # ts = float(fml[hpp+16].split(' ')[-1][:-1])
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
                print "ERRORRR!! ", hpp
    sarr = sorted(arr_m1)
    sarr2 = sorted(arr_m2)
    ans = (sarr[(95*len(sarr))/100], sarr[len(sarr)/2], sum(sarr)/len(sarr))
    ans2 = (sarr2[(95*len(sarr2))/100], sarr2[len(sarr2)/2], sum(sarr2)/len(sarr2), sarr2[(99*len(sarr2))/100])
    print (ans, ans2)
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

        runs = [4,5,6,7,8,9]        
        for f in farr:
            for r in runs:
                # read tracker log to find metric vals.
                with open('%s_tracker_node_%i.%i.%i.out'%(pre, r, f, t), 'r') as fil:
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
                (a,b) = read_actual_metric_file('%s_perf_%i.%i.%i.out'%(pre, r, f, t))
                # add the value of this run to the metric arr of len(farr)
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
            # Divide all metric by len(runs)
            mean_m[ind[f]] /= len(runs)
            med_m[ind[f]] /= len(runs)
            perc_m[ind[f]] /= len(runs)
	    p9_m[ind[f]] /= len(runs)

            mean_rxn[ind[f]] /= len(runs)
            med_rxn[ind[f]] /= len(runs)
            perc_rxn[ind[f]] /= len(runs)
	    p9_rxn[ind[f]] /= len(runs)

            mean_tput[ind[f]] /= len(runs)
            med_tput[ind[f]] /= len(runs)
            perc_tput[ind[f]] /= len(runs)
	    p9_tput[ind[f]] /= len(runs)

            mean_newm1[ind[f]] /= len(runs)
            med_newm1[ind[f]] /= len(runs)
            perc_newm1[ind[f]] /= len(runs)
	    p9_newm1[ind[f]] /= len(runs)

            mean_lat[ind[f]] /= len(runs)
            med_lat[ind[f]] /= len(runs)
            perc_lat[ind[f]] /= len(runs)
	    p9_lat[ind[f]] /= len(runs)

            td_mean_lat[ind[f]] /= len(runs)
            td_med_lat[ind[f]] /= len(runs)
            td_perc_lat[ind[f]] /= len(runs)
	    td_p9_lat[ind[f]] /= len(runs)

            td_mean_rxn[ind[f]] /= len(runs)
            td_med_rxn[ind[f]] /= len(runs)
            td_perc_rxn[ind[f]] /= len(runs)
	    td_p9_rxn[ind[f]] /= len(runs)

            perc_m1[ind[f]] /= len(runs)
            med_m1[ind[f]] /= len(runs)
            mean_m1[ind[f]] /= len(runs)
	    p9_m1[ind[f]] /= len(runs)

            with open(fname, 'a') as f1:
                f1.write('%i %i 10RunAvg N3Latency 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, p9_lat[ind[f]], perc_lat[ind[f]], med_lat[ind[f]], mean_lat[ind[f]]))
                f1.write('%i %i 10RunAvg N3Latency w.r.t. TDNode 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, td_p9_lat[ind[f]], td_perc_lat[ind[f]], td_med_lat[ind[f]], td_mean_lat[ind[f]]))
                f1.write('%i %i 10RunAvg Tput 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, p9_tput[ind[f]], perc_tput[ind[f]], med_tput[ind[f]], mean_tput[ind[f]]))
                f1.write('%i %i 10RunAvg RxnTime 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, p9_rxn[ind[f]], perc_rxn[ind[f]], med_rxn[ind[f]], mean_rxn[ind[f]]))
                f1.write('%i %i 10RunAvg RxnTime w.r.t. TDNode 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, td_p9_rxn[ind[f]], td_perc_rxn[ind[f]], td_med_rxn[ind[f]], td_mean_rxn[ind[f]]))
                f1.write('%i %i 10RunAvg Perf Rel. Metric 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, p9_m[ind[f]], perc_m[ind[f]], med_m[ind[f]], mean_m[ind[f]]))
                f1.write('%i %i 10RunAvg Perf Rel. Metric1 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, p9_m[ind[f]], perc_newm1[ind[f]], med_newm1[ind[f]], mean_newm1[ind[f]]))
                f1.write('%i %i 10RunAvg Perf Abs Metric 99p : %f Tail, Med, Mean : %f %f %f #\n'%(f, t, p9_m1[ind[f]], perc_m1[ind[f]], med_m1[ind[f]], mean_m1[ind[f]]))

        abs_deg_arr.append(perc_m1)

        print new_farr
        p1 = plt.plot(new_farr, perc_m, 'ro-', label='95ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, p9_m, 'y^:', label='99ile')
	plt.plot(new_farr, med_m, 'g.:', label='Median')
        plt.plot(new_farr, mean_m, 'b*--', label='Mean')
        plt.title('Metric at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Rel Metric (offset)')
        plt.ylim(0.0, 1.0)
        plt.legend()
        plt.show()

        p5 = plt.plot(new_farr, perc_newm1, 'ro-', label='95ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, p9_newm1, 'y^:', label='99ile')
	plt.plot(new_farr, med_newm1, 'g.:', label='Median')
        plt.plot(new_farr, mean_newm1, 'b*--', label='Mean')
        plt.title('Metric1 at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Rel Metric1 (offset)')
        plt.ylim(0.0, 1.0)
        plt.legend()
        plt.show()

        p2 = plt.plot(new_farr, perc_rxn, 'ro-', label='95ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, p9_rxn, 'y^:', label='99ile')
	plt.plot(new_farr, med_rxn, 'g.:', label='Median')
        plt.plot(new_farr, mean_rxn, 'b*--', label='Mean')
        plt.title('RxnTime at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('RxnTime')
        plt.ylim(0, 0.35)
        plt.legend()
        plt.show()

        p2 = plt.plot(new_farr, td_perc_rxn, 'ro-', label='95ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, td_p9_rxn, 'y^:', label='99ile')
	plt.plot(new_farr, td_med_rxn, 'g.:', label='Median')
        plt.plot(new_farr, td_mean_rxn, 'b*--', label='Mean')
        plt.title('TD RxnTime at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('RxnTime')
        plt.ylim(0, 0.35)
        plt.legend()
        plt.show()

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
        p1 = plt.plot(new_farr, perc_m1, 'ro-', label='95ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, p9_m1, 'y^:', label='99ile')
	plt.plot(new_farr, med_m1, 'g.:', label='Median')
        plt.plot(new_farr, mean_m1, 'b*--', label='Mean')
        plt.title('Absolute Deg Diff at displacement time : %f'%(t))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Abs Metric (offset)')
        plt.ylim(0, 3.2)
        plt.legend()
        plt.show()

        p1 = plt.plot(new_farr, perc_m2, 'ro-', label='99ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, med_m2, 'g.:', label='Median')
        plt.plot(new_farr, mean_m2, 'b*--', label='Mean')
        plt.title('Absolute Distance b/w Cam & Obj at displacement time : %f'%(t))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Abs Metric : Distance Diff')
        plt.ylim(0, 10)
        plt.legend()
        plt.show()

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
