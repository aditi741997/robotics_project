import sys
import os
import matplotlib.pyplot as plt
import math

#farr = [10, 15, 20, 25, 30, 33, 38, 45, 55, 67, 80, 100]
farr = [9, 12, 15, 17, 19, 21, 23, 30, 45, 67, 80, 100] #Largec1
farr = [10, 15, 17, 20, 23, 26, 28, 30, 32, 35, 40, 60, 80, 100] # Smallc1 2c
farr = [10, 15, 19, 21, 22, 23, 25, 30, 33, 38, 45, 67, 80, 100]
# farr = [12, 20, 28, 36, 44, 52, 60, 80, 90, 100, 120, 140]

pre = ''

# argv1 : prefix of log files
# argv2 : cpp or not
# argv3 : c2
# argv4 : t
# argv5 : default run or not (RTC, Dyn are non default)
# argv6 : file name to write latency vals.
if __name__ == '__main__':
    pre = sys.argv[1]
    cpp = int(sys.argv[2])
    need_actual_freq = (int(sys.argv[5]) == 1) # we dont need new_freq for RTC and DynamicAlgo.
    farr = farr if need_actual_freq else [15]
    fname = sys.argv[6]

    mean_lats = {}
    med_lats = {}
    new_farrs = {}

    c1 = int(sys.argv[3])
    t = int(sys.argv[4])

    # for c1 in ci.keys():
    #     print "Starting ", c1

    farr = farr if need_actual_freq else [15]

    ind = {}
    for i in range(len(farr)):
        ind[farr[i]] = i
    pre1 = pre + c1
    perc_lat =  [0.0 for x in farr]
    med_lat = [0.0 for x in farr]
    mean_lat = [0.0 for x in farr]

    td_perc_lat =  [0.0 for x in farr]
    td_med_lat = [0.0 for x in farr]
    td_mean_lat = [0.0 for x in farr]


    new_farr = [0.0 for x in farr]
    print farr
    if need_actual_freq:
        with open(pre1 + "_actual_freq.txt", 'r') as af:
            afl = af.readlines()
            for l in afl:
                ls = l.split(' ')
                freq = int(ls[0])
                if int(ls[1]) == t and freq in farr:
                    new_farr[ind[freq]] = float(ls[-1][:-1])
    else:
        new_farr = farr
        

    runs = [1,2,3,4,5,6,7,8,9,10]
    for f in farr:
        for r in runs:
            with open('%s_vision_node_%i.%i.%i.out'%(pre1, r, f, t), 'r') as fil:
                print "Reading vision for ", f, t, pre1, r
                for l in fil.readlines():
                    larr = l.split(' ')
                    if (cpp == 1):
                        #  if roscpp files :
                        if 'Latency at N2:' in l:
                            pl = float(larr[12])
                            medl = float(larr[11])
                            meanl = float(larr[10])
                        elif 'TD node' in l:
                            tdpl = float(larr[17])
                            tdmedl = float(larr[16])
                            tdmeanl = float(larr[15])
                perc_lat[ind[f]] = += pl
                med_lat[ind[f]] += medl
                mean_lat[ind[f]] += meanl
                
                td_perc_lat[ind[f]] += tdpl
                td_med_lat[ind[f]] += tdmedl
                td_mean_lat[ind[f]] += tdmeanl

        #average over 10 runs:
        perc_lat[ind[f]] /= len(runs)
        med_lat[ind[f]] /= len(runs)
        mean_lat[ind[f]] /= len(runs)
        
        td_perc_lat[ind[f]] /= len(runs)
        td_med_lat[ind[f]] /= len(runs)
        td_mean_lat[ind[f]] /= len(runs)
        with open(fname, 'a') as f1:
            f1.write('%i %i 10RunAvg N2Latency Tail, Med, Mean : %f %f %f #\n'%(f, t, perc_lat[ind[f]], med_lat[ind[f]], mean_lat[ind[f]]))
            f1.write('%i %i 10RunAvg N2Latency w.r.t. TDNode Tail, Med, Mean : %f %f %f #\n'%(f, t, td_perc_lat[ind[f]], td_med_lat[ind[f]], td_mean_lat[ind[f]]))

    print med_lat
    print new_farr


    x = 9 if (cpp == 1) else 5
    s = 'roscpp' if (cpp == 1) else 'rospy'
    plt.plot(new_farr, perc_lat, 'ro-', label='9%iile'%x)
    plt.plot(new_farr, mean_lat, 'b.--', label='mean')
    plt.plot(new_farr, med_lat, 'g*:', label='Median')
    plt.title('Mean, 9%iile Latency (N1->%s N2) c2=%dms, %s'%(x, s, c1, pre1))
    plt.xlabel('Publisher Frequency')
    plt.ylabel('Latency')
    plt.ylim(0, 0.15)
    plt.legend()
    plt.show()

    plt.plot(new_farr, td_perc_lat, 'ro-', label='9%iile'%x)
    plt.plot(new_farr, td_mean_lat, 'b.--', label='mean')
    plt.plot(new_farr, td_med_lat, 'g*:', label='Median')
    plt.title('Mean, 9%iile Latency (TD->N1->%s N2) c2=%dms, %s'%(x, s, c1, pre1))
    plt.xlabel('Publisher Frequency')
    plt.ylabel('Latency w.r.t. TDNode')
    plt.ylim(0, 0.09)
    plt.legend()
    plt.show()


