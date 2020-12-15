# Node to control the obstacle robots.
import sys
import rospy
from geometry_msgs.msg import Twist
import time
from nav_msgs.msg import Odometry

obstacle_ids = ['robot_1', 'robot_2', 'robot_3', 'robot_4', 'robot_5', 'robot_6', 'robot_7', 'robot_8']

# directions:
obstacle_dir_x = [1 for x in obstacle_ids]
obstacle_dir_y = [0 for x in obstacle_ids]
curr_targets = ['target1' for x in obstacle_ids]

# current poses, as gottenfrom ground_truth_poses.
curr_poses_x = [0.0 for x in obstacle_ids]
curr_poses_y = [0.0 for x in obstacle_ids]

# turn1, turn2 are the poses from which the robot needs to flip the vel direction.
# robot starts from 0, with target1, and flips vel+target regularly.
turn_poses_x = {'target1': [-2.75, -12, 5.25, 5.3, 13.2, -2.8, -10.9, 2.25], 'target0': [-5.25, -12, 2.75, 2.8, 10.7, -5.3, -13.4, 2.25]}
turn_poses_y = {'target1': [2,2.5,9,3.1,-4.9,11,11,-9.25], 'target0': [2,0,9,3.1,-4.9,11,11,-11.75]}

cmd_vel_pubs = []
pose_subs = []

rate = float(sys.argv[1])
vel = float(sys.argv[2])
# distance to be covered per side:
# Nov: dont need this, will use turn_poses.
dist = float(sys.argv[3])

obstacle_len = [dist for x in obstacle_ids] # length to be covered b4 flipping vel.
obstacle_rel_len = [ (x*rate)/vel for x in obstacle_len] # #iterations of v1 before flipping.

print obstacle_rel_len, " #Iterations for each obstacle.", "RATE: ", rate, " VEL:", vel

# update current posn of all obstacles.
def recv_pose(data, topic):
	# rospy.loginfo(rospy.get_caller_id() + "I heard hdr.frame: %s, pose: %s, topic: %s", data.header.frame_id, str(data.pose), topic )
	oid = obstacle_ids.index(topic)
	curr_poses_x[oid] = data.pose.pose.position.x
	curr_poses_y[oid] = data.pose.pose.position.y

def reached(i):
	# checks if obst i has reached its current target.
	t = curr_targets[i]
	if ( (abs(curr_poses_x[i] - turn_poses_x[t][i]) < 0.1) and (abs(curr_poses_y[i] - turn_poses_y[t][i]) < 0.1) ):
		rospy.loginfo('Obstacle %i reached target %s, curr_poses x,y:%f,%f, turn_poses x,y:%f,%f'%(i, t,curr_poses_x[i],curr_poses_y[i],turn_poses_x[t][i], turn_poses_y[t][i]) )
		return True
	else:
		return False
	

rospy.init_node('nav2d_obstacle_vel_publisher')
# Make publishers:
for j in range(len(obstacle_ids)):
    cmd_vel_pubs.append(rospy.Publisher('/'+obstacle_ids[j]+'/cmd_vel', Twist, queue_size=1))
    pose_subs.append(rospy.Subscriber('/'+obstacle_ids[j]+'/base_pose_ground_truth', Odometry, callback=recv_pose, callback_args=obstacle_ids[j]) )

i = 0
r = rospy.Rate(rate)
tstart = time.time()
while True:
    for j in range(len(obstacle_ids)):
        # if (i%obstacle_rel_len[j]) == (obstacle_rel_len[j] - 1):
	if reached(j):
            # Reversing direction
            print "Flipping direction for obstacle: ", obstacle_ids[j], ", curr target was: ", curr_targets[j]
            obstacle_dir_x[j] *= -1.0
            obstacle_dir_y[j] *= -1.0
	    curr_targets[j] = 'target' + str((int(curr_targets[j][-1])+1)%2)
        cvel = Twist()
        cvel.linear.x = obstacle_dir_x[j] * vel
        cvel.linear.y = obstacle_dir_y[j] * vel
        cvel.linear.z = 0
        cvel.angular.x = 0
        cvel.angular.y = 0
        cvel.angular.z = 0
        cmd_vel_pubs[j].publish(cvel)
    i += 1
    if i%50 == 5:
        print "Iteration i : ", i, "Time since start: ", time.time() - tstart
    # r.sleep()
    time.sleep(1.0/rate)
