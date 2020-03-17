import rospy
import cv2
#import cv2.cv as cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from rbx1_vision.msg import roi_time

class PreProcessImg(object):
    def __init__(self, name, limit):
        self.limit = limit
        self.node_name = name

        rospy.init_node(name)
        rospy.loginfo("Starting %s"%(name))

        self.image_pub = rospy.Publisher("/camera1/rgb/image_raw", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback, queue_size=1)

        self.cb_time_arr = deque([], 25000)

        rospy.on_shutdown(self.cleanup)

    def calc_primes(self):
        i = 1
        num = 1
        primes = 0
        while (num <= self.limit):
            i = 2
            while (i <= num):
                if (num%i == 0):
                    break
                i += 1
            if (i == num):
                primes += 1
            num += 1

        rospy.loginfo("Found %i primes"%(primes))

    def image_callback(self, data):
        start = time.time()
        
        self.calc_primes()
        self.image_pub.publish(data)

        dur = time.time() - start
        self.cb_time_arr.append(dur)

    def cleanup(self):
        print "Shutting down pre process node."
        s = sorted(self.cb_time_arr)
        print "Length of arr : ", len(s)
        print "Mean, Median, Tail of cb time : ", sum(s)/len(s), s[len(s)/2], s[(95*len(s))/100]


if __name__ == '__main__':
    try:
        node_name = "preprocess"
        PreProcessImg(node_name, float(sys.argv[1]))
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down pre process Node except."