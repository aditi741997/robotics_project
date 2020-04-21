import sys
import os
import matplotlib.pyplot as plt
import math

sndbuf = [0.25, 0.66, 1.32, 3, 6, 9.25, 11]

if __name__ == '__main__':
    ind = {}
    for i in range(len(sndbuf)):
        ind[sndbuf[i]] = i

    perc_lat =  [0.0 for x in sndbuf]
    med_lat = [0.0 for x in sndbuf]
    mean_lat = [0.0 for x in sndbuf]

    # default rb is 6291456, take as input, vary sndbuf
    rb = '' if sys.argv[1] == 'default' else '_P_RB' + sys.argv[1]
    for sb in sndbuf:
        s = str(sb)
        if sb < 1.0:
            s = s[1:]
        with open('Test_G_SB' + s + rb + '_new_preprocess_node_718.out', 'r') as fil:
            print "Reading for ", s
            for l in fil.readlines():
                larr = l.split(' ')
                if 'latency of msg arrival at N1' in l:
                    perc_lat[ind[sb]] = float(larr[14][:-2])
                    med_lat[ind[sb]] = float(larr[13][:-1])
                    mean_lat[ind[sb]] = float(larr[12][:-1])

    plt.plot(sndbuf, perc_lat, 'r-', label='95ile')
    plt.plot(sndbuf, med_lat, 'g:', label='Median')
    plt.plot(sndbuf, mean_lat, 'b--', label='Mean')
    plt.title('N1 Latency (gz->rospy) w.r.t. Gz send buffer (c1=32ms, msg sz=1MB ~ 10*10^5)' + rb)
    plt.xlabel('Gazebo SndBuf (SB = 2*x*10^5)')
    plt.ylabel('Latency at N1')
    plt.legend()
    plt.show()

    # sb=1.32, vary rcvbuf

    sb = '1.32'
    rcvbuf = [0.25, 0.66, 1.32, 3, 6, 7.5, 9, 9.25, 9.5]

    rind = {}
    for i in range(len(rcvbuf)):
        rind[rcvbuf[i]] = i

    perc_rlat =  [0.0 for x in rcvbuf]
    med_rlat = [0.0 for x in rcvbuf]
    mean_rlat = [0.0 for x in rcvbuf]

    for rb in rcvbuf:
        with open('Test_G_SB' + sb + '_P_RB' + str(rb) + '_new_preprocess_node_718.out', 'r') as fil:
            print "Reading for sb:Rcv", sb, rb
            for l in fil.readlines():
                larr = l.split(' ')
                if 'latency of msg arrival at N1' in l:
                    perc_rlat[rind[rb]] = float(larr[14][:-2])
                    med_rlat[rind[rb]] = float(larr[13][:-1])
                    mean_rlat[rind[rb]] = float(larr[12][:-1])

    plt.plot(rcvbuf, perc_rlat, 'r-', label='95ile')
    plt.plot(rcvbuf, med_rlat, 'g:', label='Median')
    plt.plot(rcvbuf, mean_rlat, 'b--', label='Mean')
    plt.title('N1 Latency (gz->rospy) w.r.t. Py rcv buffer (c1=32ms, msg sz=1MB ~ 10*10^5)')
    plt.xlabel('Python RcvBuf (RB = 2*x*10^5)')
    plt.ylabel('Latency at N1')
    plt.legend()
    plt.show()