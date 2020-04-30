import sys
import os
import matplotlib.pyplot as plt
import math

farr = [10, 15, 20, 25, 30, 35, 40, 45, 50, 70, 90, 110]
# farr = [12, 20, 28, 36, 44, 52, 60, 70, 80, 90, 100, 130]

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
                arr_m1.append(abs(ty/hy))
                h_theta = math.atan(hy/3.0)
                t_theta = 2*math.atan(ts/tc)
                arr_m2.append(abs(t_theta - h_theta))
            except:
                print "ERRORRR!! ", hpp
    sarr = sorted(arr_m1)
    sarr2 = sorted(arr_m2)
    ans = (sarr[(95*len(sarr))/100], sarr[len(sarr)/2], sum(sarr)/len(sarr))
    ans2 = (sarr2[(95*len(sarr2))/100], sarr2[len(sarr2)/2], sum(sarr2)/len(sarr2))
    print ans, ans2
    return ans2

# read_actual_metric_file("2c_max5_new_perf_1008.out")

def cmps(x):
    return x[0]

if __name__ == '__main__':
    pre = sys.argv[2]

    ind = {}
    for i in range(len(farr)):
        ind[farr[i]] = i

    opt_freq = int(sys.argv[3])
    rel_perf_tail_improv_wrt_low_freq = []
    abs_perf_tail_improv_wrt_low_freq = []
    rel_perf_med_improv_wrt_low_freq = []
    abs_perf_med_improv_wrt_low_freq = []

    dist = 8.0
    # human_speed_arr = [dist/24, dist/20, dist/16, dist/12, dist/8, dist/4, dist/3, dist/2]

    for t in [8]:
        perc_m = [0.0 for x in farr]
        med_m = [0.0 for x in farr]
        mean_m = [0.0 for x in farr]

        perc_rxn = [0.0 for x in farr]
        med_rxn = [0.0 for x in farr]
        mean_rxn = [0.0 for x in farr]
        
        new_farr = [0.0 for x in farr]

        m_rxn_ratio = [1.0 for x in farr]

        with open(sys.argv[1], 'r') as af:
            afl = af.readlines()
            for l in afl:
                ls = l.split(' ')
                freq = int(ls[0])
                if int(ls[1]) == t and freq in farr:
                    new_farr[ind[freq]] = float(ls[-1][:-1])

        # Measuring from gz model states :
        # m2 : if tb3 pointing towards (x,y1) and human at (x,y2) then y1/y2
        # m1 : tb3_theta - human_theta absolute (diff in radians of orientation)
        perc_m1 = [0.0 for x in farr]
        med_m1 = [0.0 for x in farr]
        mean_m1 = [0.0 for x in farr]

        perc_m2 = [0.0 for x in farr]
        med_m2 = [0.0 for x in farr]
        mean_m2 = [0.0 for x in farr]

        perc_lat =  [0.0 for x in farr]
        med_lat = [0.0 for x in farr]
        mean_lat = [0.0 for x in farr]

        perc_tput =  [0.0 for x in farr]
        med_tput = [0.0 for x in farr]
        mean_tput = [0.0 for x in farr]

        for f in farr:
            # read tracker log to find metric vals.
            with open(pre + '_tracker_node_' + str(f) + str(t) + '.out', 'r') as fil:
                print "Reading for : ", f, t
                for l in fil.readlines():
                    larr = l.split(' ')
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
            m_rxn_ratio[ind[f]] = med_m[ind[f]]/(med_rxn[ind[f]]*med_rxn[ind[f]])
            
            # read new_perf files for m1, m2 :
            perc_m1[ind[f]], med_m1[ind[f]], mean_m1[ind[f]] = read_actual_metric_file(pre + '_new_perf_%i%i.out'%(f,t))

        print new_farr
        p1 = plt.plot(new_farr, perc_m, 'ro-', label='99ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, med_m, 'g.:', label='Median')
        plt.plot(new_farr, mean_m, 'b*--', label='Mean')
        plt.title('Metric at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Rel Metric (offset)')
        plt.ylim(0.0, 1.0)
        plt.legend()
        plt.show()

        p2 = plt.plot(new_farr, perc_rxn, 'ro-', label='99ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, med_rxn, 'g.:', label='Median')
        plt.plot(new_farr, mean_rxn, 'b*--', label='Mean')
        plt.title('RxnTime at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('RxnTime')
        plt.ylim(0, 0.25)
        plt.legend()
        plt.show()

        # plt.plot(new_farr, m_rxn_ratio, 'g*:', label='Ratio')
        # plt.title('Ratio of Median metric to median rxn time at c1 : %s'%(sys.argv[4]))
        # plt.xlabel('Publisher Frequency')
        # plt.ylabel('Ratio')
        # plt.show()

        p2 = plt.plot(new_farr, perc_lat, 'ro-', label='99ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, med_lat, 'g.:', label='Median')
        plt.plot(new_farr, mean_lat, 'b*--', label='Mean')
        plt.title('Latency at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Latency')
        plt.ylim(0, 0.2)
        plt.legend()
        plt.show()

        p2 = plt.plot(new_farr, perc_tput, 'ro-', label='99ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, med_tput, 'g.:', label='Median')
        plt.plot(new_farr, mean_tput, 'b*--', label='Mean')
        plt.title('Tput at displacement time : %f, %s'%(t, sys.argv[4]))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Tput')
        plt.ylim(0, 0.14)
        plt.legend()
        plt.show()

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

        p1 = plt.plot(new_farr, perc_m1, 'r-', label='99ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
        plt.plot(new_farr, med_m1, 'g:', label='Median')
        plt.plot(new_farr, mean_m1, 'b--', label='Mean')
        plt.title('Absolute Deg Diff at displacement time : %f'%(t))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Abs Metric (offset)')
        plt.legend()
        plt.show()

        rel_perf_tail_improv_wrt_low_freq.append(perc_m[ind[10]]/perc_m[ind[opt_freq]])
        # abs_perf_tail_improv_wrt_low_freq.append(perc_m1[ind[10]]/perc_m1[ind[opt_freq]])

        rel_perf_med_improv_wrt_low_freq.append(med_m[ind[10]]/med_m[ind[opt_freq]])
        # abs_perf_med_improv_wrt_low_freq.append(med_m1[ind[10]]/med_m1[ind[opt_freq]])

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
