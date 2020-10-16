# Node to control the obstacle robots.
import sys
import rospy
from geometry_msgs.msg import Twist
import time

obstacle_ids = ['robot_1', 'robot_2', 'robot_3']

# directions:
obstacle_dir_x = [1, 1, 1]
obstacle_dir_y = [0, 0, 0]

# distance to be covered per side:

cmd_vel_pubs = []

rate = float(sys.argv[1])
vel = float(sys.argv[2])
dist = float(sys.argv[3])

obstacle_len = [dist for x in obstacle_ids] # length to be covered b4 flipping vel.
obstacle_rel_len = [ (x*rate)/vel for x in obstacle_len] # #iterations of v1 before flipping.

print obstacle_rel_len, " #Iterations for each obstacle.", "RATE: ", rate, " VEL:", vel

rospy.init_node('nav2d_obstacle_vel_publisher')
# Make publishers:
for j in range(len(obstacle_ids)):
    cmd_vel_pubs.append(rospy.Publisher('/'+obstacle_ids[j]+'/cmd_vel', Twist, queue_size=1))

i = 0
r = rospy.Rate(rate)
tstart = time.time()
while True:
    for j in range(len(obstacle_ids)):
        if (i%obstacle_rel_len[j]) == (obstacle_rel_len[j] - 1):
            # Reversing direction
            print "Flipping direction for obstacle: ", obstacle_ids[j]
            obstacle_dir_x[j] *= -1.0
            obstacle_dir_y[j] *= -1.0
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