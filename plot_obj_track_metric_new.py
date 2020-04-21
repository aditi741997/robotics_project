import sys
import os
import matplotlib.pyplot as plt
import math
import plot_obj_track_metric

# farr = [10, 20, 40, 60, 80, 100, 120, 150, 175, 200]
# farr = [10, 70, 80, 85, 90, 95, 100, 200]
# farr = [10, 30, 50, 70, 100, 150, 175, 200]
# farr = [10, 30, 50, 60, 65, 70, 85, 100, 160, 200]

farr = [10, 70]

ind = {}
for i in range(len(farr)):
    ind[farr[i]] = i

hs = int(sys.argv[1])

rel_offset_perc_ratio = []
rel_offset_med_ratio = []

abs_offset_perc_ratio = []
abs_offset_med_ratio = []

robo_speed_arr = [1,2,3,4,5]

for robo_speed in robo_speed_arr:
    new_farr = [0.0 for x in farr]

    perc_m1 = [0.0 for x in farr]
    med_m1 = [0.0 for x in farr]
    mean_m1 = [0.0 for x in farr]

    perc_m = [0.0 for x in farr]
    med_m = [0.0 for x in farr]
    mean_m = [0.0 for x in farr]

    with open('3c_max%i_actual_freq.txt'%(robo_speed), 'r') as af:
        afl = af.readlines()
        for l in afl:
            ls = l.split(' ')
            freq = int(ls[0])
            if int(ls[1]) == hs and freq in farr:
                new_farr[ind[freq]] = float(ls[-1][:-1])

    # m1, m defined in plot_obj_track_metric file.
    for f in farr:
        # read tracker log to find metric vals.
        with open('3c_max%i_new_tracker_node_'%(robo_speed) + str(f) + str(hs) + '.out', 'r') as fil:
            for l in fil.readlines():
                larr = l.split(' ')
                if "perf_metric" in l:
                    mean_m[ind[f]] = float(larr[7][:-1])
                    med_m[ind[f]] = float(larr[8][:-1])
                    perc_m[ind[f]] = float(larr[9][:-1])

        perc_m1[ind[f]], med_m1[ind[f]], mean_m1[ind[f]] = plot_obj_track_metric.read_actual_metric_file('3c_max%i_new_perf_%i%i.out'%(robo_speed, f,hs))

    rel_offset_perc_ratio.append(perc_m[ind[10]]/perc_m[ind[70]])
    rel_offset_med_ratio.append(med_m[ind[10]]/med_m[ind[70]])

    abs_offset_perc_ratio.append(perc_m1[ind[10]]/perc_m1[ind[70]])
    abs_offset_med_ratio.append(med_m1[ind[10]]/med_m1[ind[70]])

plt.plot(robo_speed_arr, rel_offset_perc_ratio, 'r-', label='Ratio of 99ile')
plt.plot(robo_speed_arr, rel_offset_med_ratio, 'g:', label='Ratio of Median')
plt.title('Ratio of relative offset at 10 & 70Hz (~Opt) w.r.t Robo speed')
plt.xlabel('Max Robo Speed')
plt.ylabel('Ratio')
plt.legend()
plt.show()

plt.plot(robo_speed_arr, abs_offset_perc_ratio, 'r-', label='Ratio of 99ile')
plt.plot(robo_speed_arr, abs_offset_med_ratio, 'g:', label='Ratio of Median')
plt.title('Ratio of absolute offset at 10 & 70Hz (~Opt) w.r.t Robo speed')
plt.xlabel('Max Robo Speed')
plt.ylabel('Ratio')
plt.legend()
plt.show()
