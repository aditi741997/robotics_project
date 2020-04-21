import sys
import os
import matplotlib.pyplot as plt
import math

# farr = [10, 30, 50, 70]
farr = [6, 12, 30, 40, 50, 60, 70, 100]

pre = ''

# argv1 : name of file with actual freqs
# argv2 : prefix of log files
if __name__ == '__main__':
    pre = sys.argv[1]
    cpp = int(sys.argv[2])


    mean_lats = {}
    med_lats = {}
    new_farrs = {}
    ci_264kb = {"_Newc1" : 20, "_c1" : 26, "_Medc1" : 16}
    ci_1mb = {"" : 16, "_Newc1" : 22, "_c1" : 34} # 
    ci = ci_1mb if sys.argv[4] == "1mb" else ci_264kb
    t=8
    for c1 in ci.keys():
        print "Starting ", c1
        if c1 == "" and sys.argv[4] == "1mb":
            farr[1] = 10
        else:
            farr[1] = 12
        ind = {}
        for i in range(len(farr)):
            ind[farr[i]] = i
        pre1 = pre + c1
        perc_lat =  [0.0 for x in farr]
        med_lat = [0.0 for x in farr]
        mean_lat = [0.0 for x in farr]
        mean_perc_lat = [0.0 for x in farr]

        new_farr = [0.0 for x in farr]

        with open(pre1 + "_actual_freq.txt", 'r') as af:
            afl = af.readlines()
            for l in afl:
                ls = l.split(' ')
                freq = int(ls[0])
                if int(ls[1]) == t and freq in farr:
                    new_farr[ind[freq]] = float(ls[-1][:-1])

        ss = 'new_' if sys.argv[4] != "1mb" else ''
        for f in farr:
            with open(pre1 + '_' + ss + 'preprocess_node_' + str(f) + str(t) + '.out', 'r') as fil:
                print "Reading for ", f, t
                for l in fil.readlines():
                    larr = l.split(' ')
                    if (cpp == 1):
                        # c1n_latency if roscpp files :
                        if 'c1n_latency' in l:
                            perc_lat[ind[f]] = float(larr[2][:-1])
                            med_lat[ind[f]] = float(larr[3][:-1])
                            mean_lat[ind[f]] = float(larr[4][:-1])
                    else:
                        if 'latency of msg arrival at N1' in l:
                            perc_lat[ind[f]] = float(larr[14][:-2])
                            med_lat[ind[f]] = float(larr[13][:-1])
                            mean_lat[ind[f]] = float(larr[12][:-1])
            if med_lat[ind[f]] == 0.0:
                # need to read lat file
                with open(pre1 + '_' + ss + 'preprocess_lat_' + str(f) + str(t) + '.txt', 'r') as ff:
                    lat_arr = [float(x[:-1]) for x in ff.readlines()[:-1]]
                    ll = len(lat_arr)
                    lat_arr = sorted(lat_arr)
                    perc_lat[ind[f]] = lat_arr[(95*ll)/100]
                    med_lat[ind[f]] = lat_arr[ll/2]
                    mean_lat[ind[f]] = sum(lat_arr)/ll

            mean_perc_lat[ind[f]] = perc_lat[ind[f]] + mean_lat[ind[f]]

        print new_farr
        print perc_lat
        x = 9 if (cpp == 1) else 5
        s = 'roscpp' if (cpp == 1) else 'rospy'
        plt.plot(new_farr, perc_lat, 'ro-', label='9%iile'%x)
        plt.plot(new_farr, mean_lat, 'b.--', label='mean')
        plt.plot(new_farr, med_lat, 'g*:', label='Median')
        plt.title('Mean, 9%iile Latency (gz->%s) c1=%dms, %s'%(x, s, ci[c1], pre))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Latency')
        plt.legend()
        plt.show()

        mean_lats[c1] = mean_lat
        med_lats[c1] = med_lat
        new_farrs[c1] = new_farr

    for x in ci.keys():
        plt.plot(new_farrs[x], mean_lats[x], '*:', label='Mean for %dms'%(ci[x]))
    plt.title("Mean latency at " + sys.argv[3])
    plt.xlabel('Publisher Frequency')
    plt.ylabel('Mean latency')
    plt.legend()
    plt.show()

    for x in ci.keys():
        plt.plot(new_farrs[x], med_lats[x], '*:', label='Median for %dms'%(ci[x]))
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