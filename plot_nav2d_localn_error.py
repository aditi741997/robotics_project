import sys, os, time
import rospy
import tf
import geometry_msgs
import numpy as np

from math import sqrt, pi

expt_id = sys.argv[1]

small_map = ((sys.argv[2]) == "small" ) # whether its the small map
smallest_map = (sys.argv[2] == "smallest")
small_map_obst = ("obst" in sys.argv[2])

start_rx = 0 if small_map else -5
start_ry = -2 if small_map else -5

runs = range(1,24)
for badr in [4,15,16]:
    runs.remove(badr)
print(runs, len(runs))

rospy.init_node('turtle_tf_listener')

# convert frm GT x,y,z,w to odom x,y,z,w
def convert_gt_pose_to_odom_pose(x,y,rz,rw):
    # rz,rw represent some angle
    orient = [0,0,rz,rw]  #geometry_msgs.msg.Quaternion( 0,0,rz,rw)
    rotate = [0,0,-0.382683,0.92388]
    euler = tf.transformations.euler_from_quaternion( (0,0,rz,rw) )
    # rotate by -45deg:
    new_orient = tf.transformations.quaternion_multiply(orient, rotate)
    euler2 = tf.transformations.euler_from_quaternion(new_orient)
    #print("euler [b4 rotating -45deg] for orient : , euler after: ", euler, orient, euler2)
    # new x,y:
    newx = (x-start_rx)*sqrt(0.5) + (y-start_ry)*sqrt(0.5) + 0.2
    newy = -1.0*(x-start_rx)*sqrt(0.5) + (y-start_ry)*sqrt(0.5)
    newz = 0.2
    return (newx, newy, euler2[2])

def get_angle_diff(y1,y2):
    d = y2-y1
    d += 2*pi if (d < -1.0*pi) else 0.0
    d -= 2*pi if (d > pi) else 0.0
    return d

per_run_ts_avg_pose_err = []
per_run_ts_avg_rot_err = []

for r in runs:
    time_scanposearr = {}
    all_scans_ts = {} # so we store GT pose of only these.
    exp_id = expt_id + "_run" + str(r)
    
    #run start time?
    start_st_i = 0.0
    with open("nav2d_robot_logs_" + exp_id + ".err", 'r') as f:
        for fl in f.readlines():
            if "received StartExploration service action" in fl:
                start_st_i = float( fl.split(' ')[2][:-2] )
    
    with open("../mapper_scansPose_"+ exp_id + ".txt", 'r') as fs:
        fsl = fs.readlines()
        li = 0
        while li < len(fsl):
            if "MU" in fsl[li]:
                st_ts = int(10*float(fsl[li].split(' ')[0]))
                li += 1
                time_scanposearr[st_ts] = []
                while li < len(fsl) and "MU" not in fsl[li]:
                    stuff = fsl[li].split(' ')[:4]
                    ts = int(stuff[0])
                    x_y_yaw = [float(x) for x in stuff[1:]]
                    time_scanposearr[st_ts].append( (ts, x_y_yaw) )
                    all_scans_ts[ts] = []
                    li += 1
                    if (li % 100 == 77):
                        print("ADDED This to arr :", st_ts, (ts, x_y_yaw) )
                if st_ts < 2102:
                    print(st_ts, ":", time_scanposearr[st_ts])
    # open obst GT logs.
    with open('../robot_nav2d_obstacleDist_logs_' + exp_id + '.txt', 'r') as f:
        num_obst = 2 if smallest_map else 1 if (not small_map_obst) else 8
        obfl = f.readlines()
        numl = (num_obst+3)
        # store pose for all TS present in all_scans_ts
        for o in range(len(obfl)//numl):
            rob_pos = o*numl + 1
            rob_pos_l = obfl[rob_pos].split(' ')
            rob_x = float(rob_pos_l[1])
            rob_y = float(rob_pos_l[2][:-1])
            pos_st_ts = float(obfl[o*numl].split(' ')[1])
            pos_st_ts = int(10*pos_st_ts)
            if pos_st_ts in all_scans_ts:
                oz = float( obfl[o*numl + num_obst + 2].split(' ')[6] )
                ow = float( obfl[o*numl + num_obst + 2].split(' ')[7] )
                all_scans_ts[pos_st_ts] = convert_gt_pose_to_odom_pose(rob_x,rob_y,oz,ow)
                if pos_st_ts < 2102:
                    print(pos_st_ts, " pos:", all_scans_ts[pos_st_ts])

    # for each elem in time_scanposearr, find avg x-err, y-err, yaw-err.
    ts_avgerror = {}
    ts_avg_roterr = {}
    for k,v in time_scanposearr.items():
        pos_err = [ sqrt( (x[1][0] - all_scans_ts[x[0]][0] )**2 + (x[1][1] - all_scans_ts[x[0]][1] )**2 ) for x in v ]
        rot_err = [ get_angle_diff( x[1][2], all_scans_ts[x[0]][2] ) for x in v]
        v1 = np.mean(pos_err)
        if k > 10*start_st_i:
            k1 = int( (k/10.0 - start_st_i )//50 )
            ts_avgerror[k1] = round(v1,3)
            ts_avg_roterr[k1] = round(np.mean(rot_err),3 )
        if len(pos_err) < 20:
            print("for TS: ", k, k1, " pos_error arr: ", pos_err, rot_err)
    per_run_ts_avg_pose_err.append(ts_avgerror)
    per_run_ts_avg_rot_err.append(ts_avg_roterr)
print("AVG POSE ERROR PER SLOT:", per_run_ts_avg_pose_err)
print('-')
print("AVG ROT ERROR PER SLOT:", per_run_ts_avg_rot_err)
