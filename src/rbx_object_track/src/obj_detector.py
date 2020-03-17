import rospy
import cv2
import imutils
# import cv2.cv as cv
from rbx1_vision.ros2opencv2 import ROS2OpenCV2

class ObjDetector(ROS2OpenCV2):
    def __init__(self, node_name):
        super(ObjDetector, self).__init__(node_name)

        self.detect_box = None

        # Track the number of hits and misses
        self.hits = 0
        self.misses = 0
        self.hit_rate = 0

    def process_image(self, img):
        try:
            # Create a greyscale version of the image
            # grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            self.detect_box = self.detect_obj(img)
            if self.detect_box is not None:
                self.hits += 1
            else:
                self.misses += 1            
            # Keep tabs on the hit rate so far
            if ((self.hits + self.misses) > 0):
                self.hit_rate = float(self.hits) / (self.hits + self.misses)
                rospy.loginfo("Hit rate : " + str(self.hit_rate))
        except:
            pass

        return img

    def detect_obj(self, img):
        greenLower = (29, 86, 6)
        greenUpper = (64, 255, 255)
        blurred = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)

        cnts = imutils.grab_contours(cnts)
        center = None

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            box = cv2.boundingRect(c)
            # rospy.loginfo("FOUND A green box!")
            return box
        else:
            return None

            # ((x, y), radius) = cv2.minEnclosingCircle(c)
            # M = cv2.moments(c)
            # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

if __name__ == '__main__':
    try:
        node_name = "obj_detector"
        ObjDetector(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down obj detector node."
        cv2.destroyAllWindows()
