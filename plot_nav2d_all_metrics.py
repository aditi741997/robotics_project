import sys
import matplotlib.pyplot as plt
import numpy as np
import math

# For each metric, we read 3 files : F/5, F, 5F
num_e = 2
num_r = int(sys.argv[5])
efs = [sys.argv[1], sys.argv[2]] #, sys.argv[3]] #e.g. tb3_gz_5c_scanF2, ...

param_varying = [sys.argv[3], sys.argv[4] ] #, sys.argv[6]]

# Plot Median RT
medrt_S_LC_LP = [0.0 for x in range(num_e)]
medrt_S_MapCB_NavC_LP = [0.0 for x in range(num_e)]
medrt_S_MapCB_NavP_NavC_LP = [0.0 for x in range(num_e)]
medrt_S_MapCB_MapU_NavP_NavC_LP = [0.0 for x in range(num_e)]


for ei in range(num_e):
    ename = efs[ei]
    
    mean_medrt_S_LC_LP = 0.0
    mean_medrt_S_MapCB_NavC_LP = 0.0
    mean_medrt_S_MapCB_NavP_NavC_LP = 0.0
    mean_medrt_S_MapCB_MapU_NavP_NavC_LP = 0.0

    mean_75perc_rt_S_LC_LP = 0.0
    mean_75perc_rt_S_MapCB_NavC_LP = 0.0
    mean_75perc_rt_S_MapCB_NavP_NavC_LP = 0.0
    mean_75perc_rt_S_MapCB_MapU_NavP_NavC_LP = 0.0
    
    for run in range(1,num_r+1,1):
        with open('nav2d_robot_logs_' + ename + '_run' + str(run) + '.err', 'r') as of:
            a=0.0
            b=0.0
            c=0.0
            d=0.0
            a75p=0.0
            b75p=0.0
            c75p=0.0
            d75p=0.0
            for l in of.readlines():
                if "RT wrt S-LC-LP" in l:
                    a= float(l.split(' ')[13])
                    a75p = float(l.split(' ')[22][:-1])
                elif "RT wrt S-MapCB-NavC-LP" in l:
                    b= float(l.split(' ')[13])
                    b75p = float(l.split(' ')[22][:-1])
                elif "RT wrt S-MapCB-NavP-NavC-LP" in l:
                    c= float(l.split(' ')[13])
                    c75p = float(l.split(' ')[22][:-1])
                elif "RT wrt S-MapCB-MapU-NavP-NavC-LP" in l:
                    # print l, l.split(' ')[13]
                    d= float(l.split(' ')[13])
                    d75p = float(l.split(' ')[22][:-1])
        mean_medrt_S_LC_LP += a
        mean_medrt_S_MapCB_NavC_LP += b
        mean_medrt_S_MapCB_NavP_NavC_LP += c
        mean_medrt_S_MapCB_MapU_NavP_NavC_LP += d

    medrt_S_LC_LP[ei] = mean_medrt_S_LC_LP/num_r
    medrt_S_MapCB_NavC_LP[ei] = mean_medrt_S_MapCB_NavC_LP/num_r
    medrt_S_MapCB_NavP_NavC_LP[ei] = mean_medrt_S_MapCB_NavP_NavC_LP/num_r
    medrt_S_MapCB_MapU_NavP_NavC_LP[ei] = mean_medrt_S_MapCB_MapU_NavP_NavC_LP/num_r


def plot_smt(arr, yl, titl):
    ngroups = num_e
    fig, ax = plt.subplots()
    index = np.arange(ngroups)
    bar_width = 0.2
    opacity = 0.8
    plt.bar(index, arr, alpha=opacity, color='b', label=titl)
    plt.ylabel(yl)
    plt.title(titl)
    plt.xticks(index, param_varying)
    plt.show()
    

print 'Avg MedianRT across ' + str(num_r) + ' runs.', medrt_S_LC_LP
plot_smt(medrt_S_LC_LP, 'RT (Sec)', 'Median RT wrt S-LC-LP')
plot_smt(medrt_S_MapCB_NavC_LP, 'RT (sec)', 'Median RT wrt S-MapCB-NavC-LP')
plot_smt(medrt_S_MapCB_NavP_NavC_LP, 'RT (sec)', 'Median RT wrt S-MapCB-NavP-NavC-LP')
plot_smt(medrt_S_MapCB_MapU_NavP_NavC_LP, 'RT (sec)', 'Median RT wrt S-MapCB-MapU-NavP-NavC-LP')
# plt.bar(index, medrt_S_LC_LP, alpha=opacity, color='b', label='Med RT S-LC-LP')
# # plt.xlabel()
# plt.ylabel('RT (sec)')
# plt.title('Median RT wrt S-LC-LP')
# plt.xticks(index, param_varying)
# plt.show()

def get_room_no(x,y):
    # given x,y of robot, find room#
    if (x >= -3.5) and (x <= 3.5) and (y >= -2.5) and (y <= 0.5):
        return 1
    elif (x >= 0) and (x <= 2) and (y >=1) and (y <= 5):
        return 2
    elif (x >= -7.5) and (x <= -5.5) and (y >= 1.5) and (y <= 4.5):
        return 3
    elif (x >= 2.5) and (x <= 7.5) and (y >= 0) and (y <= 4.5) :
        return 4 #room5
    elif (x >= 5) and (x <= 7.5) and (y >= -5) and (y <= -0.5) :
        return 5 #room6
    else:
        return -1 

def get_obstacle_no_stage(x,y):
    if (x >= -7) and (x <= -1) and (y >= -1) and (y <= 5):
        return 1 # robot2 is line_no+1
    elif (x >= -15) and (x <= -9) and (y >= -2) and (y <= +4):
        return 2 # robot3 is line_no+2
    elif (x >= 1) and (x <= 7) and (y >= 6) and (y <= 12) :
        return 3 # robot4 is line_no+3
    else:
        return -1

def get_dist(x1,y1,x2,y2):
    return math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))

# Plot avg dist to obstacle : 5rooms - 5cases
# room1 : x [-3.5,3.5] y [-2.5,0.5]
# room2 : x [0,2] y [1,5]
# room3 : x [-7.5, -5.5] y [1.5, 4.5] 
# room5 :
# room6 : 

avg_obst_dist_arr = [0.0 for x in range(num_e)]
obst_dist_arr = [[] for x in range(num_e)]

num_obst = 5
for ei in range(num_e):
    ename = efs[ei]
    avg_obst_d = 0.0

    for run in range(1, num_r+1, 1):
        r_avg_obst_dist = 0.0
        num_read = 0
        with open('../robot_nav2d_obstacleDist_logs_' + ename + '_run' + str(run) + '.txt', 'r') as obf:
            obfl = obf.readlines()
            # split into groups of 6*[]lines. for measure_obstacleDist script.
            # numl = 6*4
            # split into groups of 4lines for stageros.
            numl = 4*1
            print "Reading ", ename, ", run:", run
            for i in range(len(obfl)/numl):
                # for tb3 - gz:
                # rob_pos = i*numl
                # rob_x = float( obfl[tb3_pos].split(' ')[-1][:-1] )
                # rob_y = float( obfl[tb3_pos+1].split(' ')[-1][:-1] )
                # obst_ind = get_room_no(rob_x, rob_y)

                # for stage:
                rob_pos = i*numl
                rob_pos_l = obfl[rob_pos].split(' ')
                rob_x = float(rob_pos_l[1])
                rob_y = float(rob_pos_l[2][:-1])
                obst_ind = get_obstacle_no_stage(rob_x, rob_y)
                
                if (obst_ind > -1):
                    # 4lines for each robot in tb3-gazebo:
                    # obst_x = float( obfl[tb3_pos + 4*obst_ind].split(' ')[-1][:-1] )
                    # obst_y = float( obfl[tb3_pos + 4*obst_ind + 1].split(' ')[-1][:-1] )

                    # 1line per obstacle for stageros:
                    obst_line = obfl[rob_pos + obst_ind].split(' ')
                    obst_x = float(obst_line[1])
                    obst_y = float(obst_line[2][:-1]) # remove \n.

                    dist = get_dist(rob_x, rob_y, obst_x, obst_y)

                    r_avg_obst_dist += dist
                    num_read += 1
                    obst_dist_arr[ei].append(dist)

                    if i % 200 == 5:
                        print "i:", i, ", tb3_pos:", rob_pos, rob_x, rob_y, obst_x, obst_y, obst_ind
                    if dist > 15.0:
                        print "i:", i, ", tb3_pos:", rob_pos, rob_x, rob_y, obst_x, obst_y, obst_ind, " DIST : ", dist
        print "Num reads: ", num_read  
        if (num_read) > 0:
            avg_obst_d += r_avg_obst_dist/num_read
            print "Avg dist for this run :", r_avg_obst_dist/num_read
        else:
            avg_obst_d += 100

    avg_obst_dist_arr[ei] = avg_obst_d/num_r

print "Avg obst dist :", avg_obst_dist_arr
plot_smt(avg_obst_dist_arr, "Avg dist (m)", "Avg Distance of Robot to Closest Obstacle [In room]")

sty = ['g-', 'b*', 'r--']
# Plotting cdf of distance:
for ei in range(num_e):
    arr = obst_dist_arr[ei]
    arr = np.sort(arr)
    pr = 1. * np.arange(len(arr))/(len(arr) - 1)
    plt.plot(arr, pr, sty[ei], label=efs[ei])

plt.ylabel('CDF')
plt.legend()
plt.show()


# Plot Map Area/Size
# Read mapper output
avg_map_sz = [0 for x in range(num_e)]
avg_unknown_ratio = [00 for x in range(num_e)]

for ei in range(num_e):
    ename = efs[ei]
    sum_mpsz = 0
    sum_unk_ratio = 0.0
    for run in range(1,num_r+1,1):
        mpsz = 0
        unk_ratio = 0.0
        with open('nav2d_robot_logs_' + ename + '_run' + str(run) + '.err', 'r') as mf:
            for l in mf.readlines():
                if 'ratio of unknown/total area' in l:
                    a = l.split(' ')[-2]
                    # Later maybe change to just int(a).
                    mpsz = int(a)
                    unk_ratio = float(l.split(' ')[-3])/mpsz
        sum_mpsz += mpsz
        sum_unk_ratio += unk_ratio
        print "For expt", ename, ", run:", run, " Map Area: ", mpsz, " UNcovered: ", unk_ratio
    avg_map_sz[ei] = sum_mpsz/num_r
    avg_unknown_ratio[ei] = sum_unk_ratio/num_r

plot_smt(avg_map_sz, 'Avg MapSize in #Cells', 'Avg MapSize at EoR')
plot_smt(avg_unknown_ratio, 'UnknownArea Ratio', 'Avg UnknownArea Ratio in Map')

# TODO: For stage-p3at, ONLY count exploration runs with total explored area > 710000. 
# Anything less than that denotes incomplete exploration.
# Plot #Fails
num_fails = [0 for x in range(num_e)]
avg_time = [0.0 for x in range(num_e)]

for ei in range(num_e):
    ename = efs[ei]
    ct = 0
    avgt = 0.0
    for run in range(1,num_r+1,1):
        fail = False # 0 : unifnished, 1 : fail, -1 : success.
        with open('nav2d_robot_logs_' + ename + '_run' + str(run) + '.err', 'r') as nf:
            for l in nf.readlines():
                if 'Exploration has failed' in l and 'Total_Time_Taken:' in l:
                    fail = True
                    print l.split(' ')[-1][:4] + "#"
                    avgt += float( l.split(' ')[-1][:4] ) #remove \n
                elif 'Exploration has finished' in l and 'Total_Time_Taken:' in l:
                    fail = False
                    avgt += float( l.split(' ')[-1][:4] ) #remove \n
                elif 'Exploration failed,' in l:
                    fail = True
        ct += fail

    num_fails[ei] = ct
    avg_time[ei] = avgt/num_r
    print "For ", ename, num_fails[ei], avg_time[ei]

plot_smt(num_fails, '#Failed Runs', '#Failed Runs out of %i'%(num_r) )
# plot_smt(avg_time, 'Avg RunTime', 'Avg RunTime over all runs')
