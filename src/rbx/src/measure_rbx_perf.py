import rospy
# import cv2
#import cv2.cv as cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from gazebo_msgs.msg import ModelStates
# from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
# from rbx1_vision.msg import roi_time

class MeasureObjectTrackPerf(object):
    def __init__(self, name, fname):
        self.node_name = name

        rospy.init_node(name)
        rospy.loginfo("Starting %s : rbx WALA"%(name))

        self.state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.state_cb, queue_size=3)

        rospy.on_shutdown(self.cleanup)
        self.write_to_file = fname
        self.arr_posn = deque([], 20000)

    def state_cb(self, msg):
        human_index = msg.name.index("hooman")
        tb3_index = msg.name.index("turtlebot3_waffle_pi")
        self.arr_posn.append((msg.pose[human_index].position.y, msg.pose[tb3_index].orientation))
        # sleep for a while to drop msgs
        time.sleep(0.01)

    def cleanup(self):
        rospy.loginfo("Shutting down Measure node")
        with open(self.write_to_file, 'w') as wf:
            new_arr = list(self.arr_posn)
            wf.write("\n".join('h_p %s \nt_p %s ' % x for x in new_arr))


if __name__ == '__main__':
    node_name = "measure_perf"
    MeasureObjectTrackPerf(node_name, sys.argv[1])
    rospy.spin()