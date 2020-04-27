import rospy
import sys
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
# from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from pympler.asizeof import asizeof

lat_arr = []
recv_arr = [0]
first_msg_recv = [0.0]
lim = 1000
num_msg = 0

def calc_primes(duh):
    i = 1
    num = 1
    primes = 0
    while (num <= duh):
        i = 2
        while (i <= num):
            if (num%i == 0):
                break
            i += 1
        if (i == num):
            primes += 1
        num += 1

def image_cb(data):
    # st = rospy.get_time()
    lat = (rospy.Time.now() - data.stamp).to_sec()
    lat_arr.append(lat)
    recv_arr[0] += 1
    calc_primes(lim)
    if (recv_arr[0] == 1):
        first_msg_recv[0] = rospy.get_time()
    if (data.seq % 100 == 5):
        s = sorted(lat_arr)
        print asizeof(data)
        print "Mean, median, tail of latency : ", sum(s)/len(s), s[len(s)/2], s[(95*len(s))/100], len(s)
        print recv_arr[0], " msgs received in time : ", rospy.get_time() - first_msg_recv[0]
    # if (data.seq > (98*num_msg)/100):
    #     rospy.signal_shutdown("blah")

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    lim = int(sys.argv[1])
    # num_msg = int(sys.argv[2])
    image_sub = rospy.Subscriber("/camera1/image_raw", Header, image_cb, queue_size=1, tcp_nodelay=True)
    rospy.spin()

# ROSPY -> ROSPY (img) : 10, 50, 150 This is for msg siz ~8MB, but img is ~1MB
# no delay true :
# lat : 45-48, 40-42, 40-42
# tput : 10, 25, 25
# no delay false :
# 10, 50, 150
# lat : 46-49, 40-43, 41-43
# tput : 10, 25, 24

# ROSCPP -> ROSPY (hdr) : 10,50,150
# Same for No delay true/false :
# lat : 2-3, 2-3, 1.9-2.7
# tput : 10, 50, 150

# ROSPY -> ROSPY (hdr) : 10, 50, 150
# No delay true :
# lat : , , 2-2.7
# tput : , , 150