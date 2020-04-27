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
import numpy as np

if __name__ == '__main__':
    # msg = Image()
    # msg.header.frame_id = 'camera_img'
    # msg.height = 480
    # msg.width = 640
    # msg.encoding = "rgb8"
    # msg.is_bigendian = 0
    # msg.step = 1920
    # msg.data = [178 for x in range(1920*480/8)]
    msg = Header()
    s = ""
    for i in range(1920*480):
        s += '0'
    msg.frame_id = s
    f = int(sys.argv[1])
    image_pub = rospy.Publisher("/camera1/image_raw", Header, queue_size=1, tcp_nodelay=True)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(f)
    seq = 0
    print asizeof(msg), f
    while not rospy.is_shutdown():
        msg.seq = seq
        msg.stamp = rospy.Time.now()
        # msg.header.seq = seq
        # msg.header.stamp = rospy.Time.now()
        image_pub.publish(msg)
        seq += 1
        if (seq%50 == 3):
            print "Publishing ", seq
        r.sleep()