import rospy
import sys
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rospy.init_node('nav2d_move_obstacle_special')
rate = int(sys.argv[1])

obsts = ['robot_1', 'robot_2']
curr_robo_pose = (0.0, 0.0)
curr_robo_twistx = 0.00
curr_obst_poses = [(0.0, 0.0), (0.0, 0.0)]

got_robo_pose = False
got_obst_poses = [False, False]

sim_real_ratio = 4.8 #scale down vel by this ratio

def recv_pose(data, topic):
    if "0" in topic:
        global curr_robo_pose
        global curr_robo_twistx
        curr_robo_pose = (data.pose.pose.position.x, data.pose.pose.position.y)
        curr_robo_twistx = data.twist.twist.linear.x #, data.twist.twist.linear.y)
        global got_robo_pose
        if not got_robo_pose:
            print("GOT pose for ", topic, data)
            got_robo_pose = True
    else:
        global curr_obst_poses
        oi = obsts.index(topic)
        curr_obst_poses[oi] = (data.pose.pose.position.x, data.pose.pose.position.y)
        global got_obst_poses
        if not got_obst_poses[oi]:
            print("GOT pose for ", topic, data)
            got_obst_poses[oi] = True

# subscribe to robo pose & obst pose:
pose_subs = [rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, callback=recv_pose, callback_args='robot_0') ]
obst_vel_pubs = []

for i in obsts:
    pose_subs.append( rospy.Subscriber('/'+i+'/base_pose_ground_truth', Odometry, callback=recv_pose, callback_args=i ) )
    # obst vel pubs:
    obst_vel_pubs += [ rospy.Publisher('/'+i+'/cmd_vel', Twist, queue_size = 1 ) ]


# V1: start moving the obst when robox ~ obx
# V2: start moving when robox ~ ox - 1.25
def start_move():
    #if got_obst_pose and got_robo_pose and abs(curr_robo_pose[0] - curr_obst_pose[0] ) < 0.1:
    got_all_obst_poses = (len(filter(lambda x: (not x) , got_obst_poses)) == 0)
    
    # robot is close to atl one of the obstacles [dist = 1.25m, which is what the robo will cover in 1s]
    robo_close = filter( lambda x: ( abs(curr_robo_pose[0] - (x[0] - 1.25) ) < 0.05 ), curr_obst_poses ) 
   
    # check robot is moving towards right?
    if  got_all_obst_poses and got_robo_pose and len(robo_close) > 0: # and (curr_robo_twistx > 0.0):
        return True
    else:
        return False # maybe smt more complex later


# V1: move the obst with speed s.t. reaches robox + 1.4, robot in 1s from start time.
# V2: move obst with speed s.t. reaches robox+1.25,roboy ~ obx, roboy in 1s from start time.
time_limit = 1.0
def get_vel(ob_goal, st_time, ind):
    xdiff = ob_goal[0] - curr_obst_poses[ind][0]
    ydiff = ob_goal[1] - curr_obst_poses[ind][1]
    time_remain = time_limit - (time.time()-st_time)
    print("xdif: %f, ydif: %f, ind: %i, goal: "%(xdiff, ydiff, ind), ob_goal)
    if ( (abs(ydiff) < 0.1)) or (time_remain < 0.0): #(abs(xdiff) < 0.05) and 
        return (0,0)
    else:
        # V2: robot tilted by 90deg, so set velx equal to ydiff vel ..
        return ( ( ydiff ) /(sim_real_ratio*time_remain), 0 )
        # V1: set vx, vy. But robo moves only in x direction.
        # return ( time_remain/(sim_real_ratio*xdiff), time_remain /(sim_real_ratio*ydiff) )

# set goal to robo y if that lies in range
def get_goal(ind, oep):
    return oep[ind] 
    #if curr_

nmi = 0

while not start_move():
    time.sleep(1.0/rate)
    nmi += 1
    if nmi % 20 == 0:
        print("Have not moved for %i iters, "%(nmi), "robopose: ", curr_robo_pose, "robo twistx: ", curr_robo_twistx, " obstpose: ", curr_obst_poses)

# start moving:
# V2: obst guards both the entry points
dist = 0.7 #1.0
obst_endpoints = [(curr_obst_poses[0][0], curr_obst_poses[0][1] - dist), (curr_obst_poses[1][0], curr_obst_poses[1][1] + dist + 0.2)]
goals = [get_goal(i, obst_endpoints) for i in range(len(obsts))]
start_time = time.time()

curr_vels = [get_vel(goals[i], start_time, i) for i in range(len(goals))]
print("ROBOT IS HERE!! Setting GOALs: ", goals)

while abs(curr_vels[0][0]) > 0.00 or (abs(curr_vels[1][0]) > 0.00): # V1: and abs(curr_vel[1]) > 0.00:
    print("CHASING THE ROBO with vel=", curr_vels)
    for i in range(len(curr_vels)):
        cvel = Twist()
        cvel.linear.x = curr_vels[i][0]
        cvel.linear.y = curr_vels[i][1]
        cvel.linear.z = 0
        cvel.angular.x = 0
        cvel.angular.y = 0
        cvel.angular.z = 0
        obst_vel_pubs[i].publish(cvel)
    time.sleep(1.0/rate)
    curr_vels = [get_vel(goals[i], start_time, i) for i in range(len(goals))]

for i in range(len(curr_vels)):
    cvel = Twist()
    print('ZEROING out cvel: ', cvel)
    obst_vel_pubs[i].publish(cvel)
