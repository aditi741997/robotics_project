import sys
import os
import matplotlib.pyplot as plt
import math

#farr = [10, 15, 20, 25, 30, 33, 38, 45, 55, 67, 80, 100]
farr = [10, 15, 19, 21, 22, 23, 25, 30, 33, 38, 45, 67, 80, 100] #Smallc1
#farr = [9, 12, 15, 17, 19, 21, 23, 30, 45, 67, 80, 100] #Largec1
#farr = [10, 15, 17, 20, 23, 26, 28, 30, 32, 35, 40, 60, 80, 100] # Smallc1 2c
# farr = [12, 20, 28, 36, 44, 52, 60, 70, 80, 90, 100, 130]

pre = ''

# argv1 : name of file with actual freqs
# argv2 : prefix of log files
if __name__ == '__main__':
    pre = sys.argv[1]
    cpp = int(sys.argv[2])
    need_actual_freq = (int(sys.argv[7]) == 1) # we dont need new_freq for RTC and DynamicAlgo.
    farr = farr if need_actual_freq else [15]

    mean_lats = {}
    med_lats = {}
    new_farrs = {}
    ci_264kb = {"_Newc1" : 20, "_c1" : 26, "_Medc1" : 16}
    ci_1mb = {"" : 16, "_Newc1" : 22, "_c1" : 34} # 
    # newc = {"_c1A" : 22, "_c1BA" : 30, "_c1BB" : 30} # "_c1A" : 22, "_c1AB" : 22, : python
    # newc = {"_c1" : 18, "_c1B" : 22, "_c1D" : 38} # cpp 
    newc = {"" : int(sys.argv[5])}
    if sys.argv[4] == "1mb":
        ci = ci_1mb
    elif sys.argv[4] == "264kb":
        ci = ci_264kb
    else:
        ci = newc
    t=int(sys.argv[6])
    for c1 in ci.keys():
        print "Starting ", c1
        if c1 == "" and sys.argv[4] == "1mb":
            farr[1] = 10
        # else:
        #     farr[1] = 12
        # if c1 == "_c1A" and sys.argv[4] == "new":
        #     farr[-2] = 115
        # else:
        #     farr[-2] = 120
        ind = {}
        for i in range(len(farr)):
            ind[farr[i]] = i
        pre1 = pre + c1
        perc_lat =  [0.0 for x in farr]
        med_lat = [0.0 for x in farr]
        mean_lat = [0.0 for x in farr]
        mean_perc_lat = [0.0 for x in farr]

        real_perc_lat =  [0.0 for x in farr]
        real_med_lat = [0.0 for x in farr]
        real_mean_lat = [0.0 for x in farr]
	
	td_perc_lat = [0.0 for x in farr]
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

        ss = 'new_' if sys.argv[4] == "264kb" else ''
	runs = [1,2,3,4,5,6,7,8,9,10]
        for f in farr:
	    for r in runs:
                with open('%s_preprocess_node_%i.%i.%i.out'%(pre1, r, f, t), 'r') as fil:
                    print "Reading for ", f, t, pre1, r
                    for l in fil.readlines():
                        larr = l.split(' ')
                        if (cpp == 1):
                            # c1n_latency if roscpp files :
                            if 'c1n_latency' in l:
                                if ":," in l:
                                    pl= float(larr[2][:-1])
                                    medl = float(larr[3][:-1])
                                    meanl = float(larr[4][:-1])
                                else:
                                    pl = float(larr[6][:-1])
                                    medl = float(larr[7][:-1])
                                    meanl = float(larr[8][:-1])
			    elif "Latency w.r.t. TDNode" in l:
				tdpl = float(larr[12])
				tdmedl = float(larr[13])
				tdmeanl = float(larr[14])
                        else:
                            if 'latency of msg arrival at N1' in l:
                                perc_lat[ind[f]] = float(larr[14][:-2])
                                med_lat[ind[f]] = float(larr[13][:-1])
                                mean_lat[ind[f]] = float(larr[12][:-1])
		    perc_lat[ind[f]] += pl
		    med_lat[ind[f]] += medl
		    mean_lat[ind[f]] += meanl
		    
		    td_perc_lat[ind[f]] += tdpl
		    td_med_lat[ind[f]] += tdmedl
		    td_mean_lat[ind[f]] += tdmeanl

		    print perc_lat, med_lat, td_mean_lat
	    #average over 10 runs:
	    perc_lat[ind[f]] /= len(runs)
	    med_lat[ind[f]] /= len(runs)
	    mean_lat[ind[f]] /= len(runs)
	    
	    td_perc_lat[ind[f]] /= len(runs)
	    td_med_lat[ind[f]] /= len(runs)
	    td_mean_lat[ind[f]] /= len(runs)

            if (cpp == 0):
                with open(pre1 + '_' + ss + 'preprocess_lat_' + str(f) + str(t) + '.txt', 'r') as ff:
                    arr = [x.split(' ') for x in ff.readlines()[:-1]]
                    lat_arr = [float(x[2][:-1]) for x in arr]
                    ll = len(lat_arr)
                    lat_arr = sorted(lat_arr)
                    if perc_lat[ind[f]] == 0:
                        perc_lat[ind[f]] = lat_arr[(95*ll)/100]
                        med_lat[ind[f]] = lat_arr[ll/2]
                        mean_lat[ind[f]] = sum(lat_arr)/ll
                    real_recv_times = {}
                    for x in arr:
                        real_recv_times[int(x[0])] = float(x[1])
                    # print real_recv_times[3000], "real recv time at N1 for msg id 5"

                # for real lat : read real send time from gz logs :
                with open('/home/aditi/catkin_ws/Apr_Cam_RT_Logs/' + pre1 + '_CamLogs_' + str(f) + '.out', 'r') as fil:
                    arr = [x.split(' ') for x in fil.readlines()[:-1]]
                    real_msg_ts = [0.0 for x in arr]
                    for x in arr:
                        real_msg_ts[int(x[0])] = float(x[2][:-1])
                    print real_msg_ts[5], real_msg_ts[17]

                real_lat_arr = []
                for k in sorted(real_recv_times.keys()):
                    real_lat_arr.append(real_recv_times[k] - real_msg_ts[k])
                    if k%500 == 3:
                        print real_lat_arr[-1], k
                real_lat_arr.sort()
                rl = len(real_lat_arr)
                real_perc_lat[ind[f]] = real_lat_arr[(rl*95)/100]
                real_med_lat[ind[f]] = real_lat_arr[(rl)/2]
                real_mean_lat[ind[f]] = sum(real_lat_arr)/rl

            mean_perc_lat[ind[f]] = perc_lat[ind[f]] + mean_lat[ind[f]]

        print new_farr
        print perc_lat, med_lat, mean_lat
	print td_perc_lat, td_med_lat, td_mean_lat
        x = 5
        s = 'roscpp' if (cpp == 1) else 'rospy'
        plt.plot(new_farr, perc_lat, 'ro-', label='9%iile'%x)
        plt.plot(new_farr, mean_lat, 'b.--', label='mean')
        plt.plot(new_farr, med_lat, 'g*:', label='Median')
        plt.title('Mean, 9%iile Latency (gz->%s) c1=%dms, %s'%(x, s, ci[c1], pre1))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Latency')
        plt.ylim(0, 0.095)
        plt.legend()
        plt.show()

        plt.plot(new_farr, td_perc_lat, 'ro-', label='9%iile'%x)
        plt.plot(new_farr, td_mean_lat, 'b.--', label='mean')
        plt.plot(new_farr, td_med_lat, 'g*:', label='Median')
        plt.title('Mean, 9%iile Latency w.r.t. TDNode (gz->%s) c1=%dms, %s'%(x, s, ci[c1], pre1))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Latency wrt TD')
        plt.legend()
        plt.show()

	plt.plot(new_farr, real_perc_lat, 'ro-', label='9%iile'%x)
        plt.plot(new_farr, real_mean_lat, 'b.--', label='mean')
        plt.plot(new_farr, real_med_lat, 'g*:', label='Median')
        plt.title('Mean, 9%iile RealTime Latency (gz->%s) c1=%dms, %s'%(x, s, ci[c1], pre1))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('RT Latency')
        plt.legend()
        plt.show()

        mean_lats[c1] = mean_lat
        med_lats[c1] = med_lat
        new_farrs[c1] = new_farr

    for x in ci.keys():
        plt.plot(new_farrs[x], mean_lats[x], '*:', label='Mean for %dms, %s'%(ci[x], pre))
        plt.title("Mean latency at " + sys.argv[3])
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Mean latency')
        plt.legend()
        plt.show()

    for x in ci.keys():
        plt.plot(new_farrs[x], med_lats[x], '*:', label='Median for %dms, %s'%(ci[x], pre))
        plt.title("Median latency at " + sys.argv[3])
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Median latency')
        plt.legend()
        plt.show()


    # latency w.r.t. time :
    with open(sys.argv[4], 'r') as f:
        larr = f.readlines()
        num_elem = len(larr) - 2
        iarr = [i for i in range(num_elem)]
        lat_arr = []
        for l in larr[2:]:
            lat_arr.append(float(l[:-1]))

        plt.plot(iarr, lat_arr, 'o-', label='Msg Latency')
        plt.title('Latency w.r.t. Time at 60Hz (Actual : 38Hz)')
        plt.xlabel('Received Message Number')
        plt.ylabel('Latency')
        plt.legend()
        plt.show()
