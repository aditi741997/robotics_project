import rospy
from sensor_msgs.msg import RegionOfInterest, CameraInfo
from geometry_msgs.msg import Twist, PointStamped
import thread
import sys
from collections import deque
from rbx1_vision.msg import roi_time
import time

class ObjectTracker2():
    def __init__(self, cam_topic):
        rospy.init_node("object_tracker2")
                
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        self.param = 5.0

        # The maximum rotation speed in radians per second
        self.max_rotation_speed = rospy.get_param("~max_rotation_speed", self.param)
        
        # The minimum rotation speed in radians per second
        self.min_rotation_speed = rospy.get_param("~min_rotation_speed", 0.5)
        
        # Sensitivity to target displacements.  Setting this too high
        # can lead to oscillations of the robot.
        self.gain = rospy.get_param("~gain", self.param)

        # The x threshold (% of image width) indicates how far off-center
        # the ROI needs to be in the x-direction before we react
        self.x_threshold = rospy.get_param("~x_threshold", 0.1)

        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # Intialize the movement command
        self.move_cmd = Twist()

        # Set flag to indicate when the ROI stops updating
        self.target_visible = False

         # We will get the image width and height from the camera_info topic
        self.image_width = 0
        self.image_height = 0

        self.sum_cb_time = 0.0
        self.cb_count = 0
        self.arr_max_vals = 30000
        self.arr_cb_time = deque([],self.arr_max_vals)

        self.sum_metric = 0.0
        self.arr_metric = deque([], self.arr_max_vals)

        self.sum_rxn_time = 0.0
        self.arr_rxn_time = deque([], self.arr_max_vals)
        self.last_in_stamp = 0.0

        self.sum_lat = 0.0
        self.arr_lat = deque([], self.arr_max_vals)

        self.sum_tput = 0.0
        self.arr_tput = deque([], self.arr_max_vals)
        self.last_out_stamp = 0.0

        self.perc = 95

        self.moving = False
        self.start_moving_in = 0.0
        self.arr_time_move = deque([], self.arr_max_vals)

        rospy.loginfo("Waiting for camera_info topic...")
        rospy.wait_for_message(cam_topic + '/camera_info', CameraInfo)
        
        # Subscribe the camera_info topic to get the image width and height
        sub_info = rospy.Subscriber(cam_topic + '/camera_info', CameraInfo, self.get_camera_info, queue_size=1)

        # Wait until we actually have the camera data
        while self.image_width == 0 or self.image_height == 0:
            rospy.sleep(1)
        sub_info.unregister()

        # Subscribe to the ROI topic and set the callback to update the robot's motion
        # rospy.Subscriber('roi', RegionOfInterest, self.move_robot, queue_size=1)
        rospy.Subscriber('roi', roi_time, self.move_robot, queue_size=1)
        
        # Wait until we have an ROI to follow
        rospy.loginfo("Waiting for messages on /roi...")
        rospy.wait_for_message('roi', RegionOfInterest)
        
        rospy.loginfo("ROI messages detected. Starting tracker...")

    def move_robot(self, msg):
        start = time.time()
        if msg.width == 0 or msg.height == 0:
            self.target_visible = False
            self.move_cmd = Twist()
        else:
            self.target_visible = True

            target_offset_x = msg.x_offset + msg.width / 2 - self.image_width / 2
    
            try:
                percent_offset_x = float(target_offset_x) / (float(self.image_width) / 2.0)
                # if percent_offset_x < 0:
                #     m = percent_offset_x * -1.0
                # self.sum_metric += m
                # self.arr_metric.append(m)
            except:
                percent_offset_x = 0

            m = percent_offset_x
            if percent_offset_x < 0:
                m *= -1.0
                if m == percent_offset_x:
                    rospy.loginfo("ERROR! m is now same as percent_offset_x!!!")
            self.sum_metric += m
            self.arr_metric.append(m)
    
            # Rotate the robot only if the displacement of the target exceeds the threshold
            if abs(percent_offset_x) > self.x_threshold:
                # Set the rotation speed proportional to the displacement of the target
                try:
                    # if not self.moving:
                    #     self.moving = True
                    #     reached_goal = False
                    #     self.start_moving_in = msg.stamp.to_sec()

                    speed = self.gain * percent_offset_x
                    if speed < 0:
                        direction = -1
                    else:
                        direction = 1
                    self.move_cmd.angular.z = -direction * max(self.min_rotation_speed,
                                                min(self.max_rotation_speed, abs(speed)))
                except:
                    self.move_cmd = Twist()
            else:
                # Otherwise stop the robot
                self.move_cmd = Twist()
                # if self.moving:
                #     self.moving = False
                #     self.arr_time_move.append(rospy.get_time() - self.start_moving_in)
                #     rospy.loginfo("Reached goal %f"%(self.start_moving_in))

        # Publishing cmd vel now
        self.cmd_vel_pub.publish(self.move_cmd)
        end = time.time()
        duration = end - start


        # c2 time :
        self.cb_count += 1
        self.sum_cb_time += duration
        self.arr_cb_time.append(duration)

        if (self.cb_count > 5):
            # print "DUH"
            # Latency : ox - ix. Should be ~ c1+c2 if f< 1/c1
            lat = (rospy.Time.now() - msg.stamp).to_sec()
            self.sum_lat += lat
            self.arr_lat.append(lat)

            # Tput : ox - o(x-1). Should be ~ 1/f if f< 1/c1
            if (self.last_out_stamp > 0.0):
                tput = rospy.get_time() - self.last_out_stamp
                self.sum_tput += tput
                self.arr_tput.append(tput)

            if (self.last_in_stamp > 0.0):
                rxn_time = rospy.get_time() - self.last_in_stamp
                self.sum_rxn_time += rxn_time
                self.arr_rxn_time.append(rxn_time)

        self.last_out_stamp = rospy.get_time()

        # RxnTime : ox - i(x-1). Should be 1/f + c1+c2 if f< 1/c1
        self.last_in_stamp = msg.stamp.to_sec()

        # if (self.cb_count%5 < 2):
        #     print self.cb_count, self.last_in_stamp, self.last_out_stamp
        #     print "Time : ", rospy.get_time(), "Latency : ", self.arr_lat
        #     print "Tput : ", self.arr_tput
        #     print "RxnTime : ", self.arr_rxn_time

    def get_camera_info(self, msg):
        rospy.loginfo("get camera info CALLED!! SHould be done only before starting!!!")
        self.image_width = msg.width
        self.image_height = msg.height

    def print_metrics(self, a, m):
        s = sorted(a)
        print "Mean, Median, Tail of %s : %f, %f, %f"%(m, sum(s)/len(s), s[len(s)/2], s[(self.perc*len(s))/100])

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        self.print_metrics(self.arr_cb_time, "cb_time c2")
        self.print_metrics(self.arr_metric, "perf_metric offset")
        self.print_metrics(self.arr_lat, "latency")
        self.print_metrics(self.arr_tput, "tput")
        self.print_metrics(self.arr_rxn_time, "rxn_time")
        self.print_metrics(self.arr_time_move, "Time to reach")
        print "Arr time to goal : ", str(self.arr_time_move)

        # s = sorted(self.arr_cb_time)
        # print "Mean, Median, Tail of cb time : ", self.sum_cb_time/self.cb_count, s[len(s)/2], s[(len(s)*self.perc)/100]
        
        # ms = sorted(self.arr_metric)
        # print "Mean, Median, Tail of metric value : ", self.sum_metric/self.cb_count, ms[len(ms)/2], ms[(len(ms)*self.perc)/100]
        # print "Avg over count, Median cb time over, Median metric over", self.cb_count, len(s), len(ms)
        
        # lats = sorted(self.arr_lat)
        # print "Mean, Median, Tail of latency : ", self.sum_lat/self.cb_count, lats[len(lats)/2], lats[(self.perc*len(lats))/100]
        # tputs = sorted(self.arr_tput)
        # print "Mean, Median, Tail of latency : ", self.sum_tput/(), tputs[len()]        
        # rospy.sleep(1)

if __name__ == '__main__':
    try:
        camera_topic = sys.argv[1]
        print "Init tracker!"
        ObjectTracker2(camera_topic)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Object tracking node terminated.")
