import sys
import os
import matplotlib.pyplot as plt

# farr = [10, 20, 40, 60, 80, 100, 120, 150, 175, 200]
farr = [10, 40, 60, 90, 150, 200]

ind = {}
for i in range(len(farr)):
    ind[farr[i]] = i

for t in [8]:
    perc_m = [0.0 for x in farr]
    med_m = [0.0 for x in farr]
    mean_m = [0.0 for x in farr]

    perc_rxn = [0.0 for x in farr]
    med_rxn = [0.0 for x in farr]
    mean_rxn = [0.0 for x in farr]
    for f in farr:
        # read tracker log to find metric vals.
        with open('max5_new_tracker_node_' + str(f) + str(t) + '.out', 'r') as fil:
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
    p1 = plt.plot(farr, perc_m, 'r-', label='99ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
    plt.plot(farr, med_m, 'g:', label='Median')
    plt.plot(farr, mean_m, 'b--', label='Mean')
    plt.title('Metric at displacement time : %f'%(t))
    plt.xlabel('Publisher Frequency')
    plt.ylabel('Metric (offset)')
    plt.legend()
    plt.show()

    p2 = plt.plot(farr, perc_rxn, 'r-', label='99ile') #, farr, med_c1, 'g:', label='Median', farr, mean_c1, 'b--', label='Mean')
    plt.plot(farr, med_rxn, 'g:', label='Median')
    plt.plot(farr, mean_rxn, 'b--', label='Mean')
    plt.title('Metric at displacement time : %f'%(t))
    plt.xlabel('Publisher Frequency')
    plt.ylabel('RxnTime')
    plt.legend()
    plt.show()