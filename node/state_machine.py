#!/usr/bin/env python3

import rospy
import cv2

from state_clue_detect import Clue_DetectState
from state_drive_green import Drive_GreenState
from state_pedestrian import PedestrianState
from state_truck import TruckState
from state_idle import Idle

from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge, CvBridgeError


class StateMachine:
    def __init__(self):
        self.bridge = CvBridge()
        self.move = Twist()
        self.image_data = None
        self.lidar_data = None
        
        self.threshold = 180
        self.linear_speed = 2
        self.kp = 0.03
        
        self.red_count = 0
        self.pink_count = 0
        self.clue_board = 0

        self.idle = False
        self.board_detected = False
        self.green_road = True
        self.dirt_road = False
        self.off_road = False


        self.pub_time = rospy.Publisher("/score_tracker", String, queue_size = 1, latch = True)
        self.pub_vel = rospy.Publisher("/B1/cmd_vel", Twist, queue_size=1)
        self.pub_processed_cam = rospy.Publisher("/processed_img", Image, queue_size = 1)

        self.sub_clk = rospy.Subscriber("/clock", Clock, self.clock_cb)
        self.sub_cam = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.image_cb, queue_size=1)
        
        #self.sub = rospy.Subscriber("/scan", LaserScan, self.lidar_cb)

        self.states = {
           "Clue_Detect": Clue_DetectState(self),
           "Drive_Green": Drive_GreenState(self),
           "Pedestrian": PedestrianState(self),
           "Truck": TruckState(self),
           "Idle": Idle(self)
        }

        self.current_state = self.states["Idle"]
        self.current_state.enter()

    ## Receives image data through ROS, analyzes the location of the track in the image using OpenCV, 
    #  and sends appropriate valeu for yaw to the robot to realign itself with the track.
    #  @param self The object pointer
    #  @param data The image data received from the gazebo simulation through ROS
    def image_cb(self, data):
        try:
            self.image_data = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        frame_height, frame_width, _ = self.image_data.shape

        ## Crops the frame so only the bottom part is used for recognizing the path
        img_cropped = self.image_data[220:frame_height-10, 0:frame_width]
        img_gray = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2GRAY)
        _, img_bin = cv2.threshold(img_gray, self.threshold, 255, cv2.THRESH_BINARY)

        edges = cv2.Canny(img_bin, 100, 200)
        
        contours, hierarchy = cv2.findContours(img_bin, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
        contour_color = (0, 255, 0)
        contour_thick = 5

        img_color = cv2.cvtColor(img_bin, cv2.COLOR_GRAY2BGR)

        for cnt in contours:
            if cv2.contourArea(cnt) > 1200:
                with_contours = cv2.drawContours(img_color, [cnt], 0, (0,255,0), 5)

        # convert single-channel binary image to BGR so we can draw colored contours
        

        #with_contours = cv2.drawContours(img_color, contours, 0, (0,255,0), 5)
        #with_contours = cv2.drawContours(img_color, contours, -1, contour_color, contour_thick)

        ros_img = self.bridge.cv2_to_imgmsg(with_contours, encoding="bgr8")
        self.pub_processed_cam.publish(ros_img)

        # TODO: Add clue detection(CNN) code here
        # if homography detected, makes self.board_detected = True
        # In each state, it'll transition to "Send_Clue" state
 
    #def lidar_cb(self, data):
        #self.lidar_data  data

    def clock_cb(self, data):
            self.timer = data

    def run(self):
        rate = rospy.Rate(20)
        self.pub_time.publish("Team14,password,0,START")

        while not rospy.is_shutdown():

            self.next_state = self.states[self.current_state.run()]

            if self.next_state != self.current_state:
                self.current_state.exit()
                self.current_state = self.next_state
                self.current_state.enter()
            
            rate.sleep()

## Main function which initializes the state machine and instanciate callback function whenvever new data is ready
#
def main():
    rospy.init_node("state_machine")
    sm = StateMachine()
    sm.run()

if __name__ == "__main__":
    main()