import sys
import os
import matplotlib.pyplot as plt
import math

farr = [10, 15, 20, 25, 30, 35, 40, 45, 50, 70]
# farr = [12, 20, 28, 36, 44, 52, 60, 80, 90, 100, 120, 140]

pre = ''

# argv1 : name of file with actual freqs
# argv2 : prefix of log files
if __name__ == '__main__':
    pre = sys.argv[1]
    cpp = int(sys.argv[2])

    mean_lats = {}
    med_lats = {}
    new_farrs = {}

    ci = {"" : int(sys.argv[3])}
    t = 8
    for c1 in ci.keys():
        print "STarting ", c1

        ind = {}
        for i in range(len(farr)):
            ind[farr[i]] = i
        pre1 = pre + c1
        perc_lat =  [0.0 for x in farr]
        med_lat = [0.0 for x in farr]
        mean_lat = [0.0 for x in farr]

        new_farr = [0.0 for x in farr]
        print farr
        with open(pre1 + "_actual_freq.txt", 'r') as af:
            afl = af.readlines()
            for l in afl:
                ls = l.split(' ')
                freq = int(ls[0])
                if int(ls[1]) == t and freq in farr:
                    new_farr[ind[freq]] = float(ls[-1][:-1])
        
        for f in farr:
            with open(pre1 + '_vision_node_' + str(f) + str(t) + '.out', 'r') as fil:
                print "Reading vision for ", f, t, pre1
                for l in fil.readlines():
                    larr = l.split(' ')
                    if (cpp == 1):
                        #  if roscpp files :
                        if 'Latency at N2:' in l:
                            perc_lat[ind[f]] = float(larr[12])
                            med_lat[ind[f]] = float(larr[11])
                            mean_lat[ind[f]] = float(larr[10])

        print med_lat
        print new_farr

        x = 9 if (cpp == 1) else 5
        s = 'roscpp' if (cpp == 1) else 'rospy'
        plt.plot(new_farr, perc_lat, 'ro-', label='9%iile'%x)
        plt.plot(new_farr, mean_lat, 'b.--', label='mean')
        plt.plot(new_farr, med_lat, 'g*:', label='Median')
        plt.title('Mean, 9%iile Latency (N1->%s N2) c2=%dms, %s'%(x, s, ci[c1], pre1))
        plt.xlabel('Publisher Frequency')
        plt.ylabel('Latency')
        plt.ylim(0, 0.12)
        plt.legend()
        plt.show()


