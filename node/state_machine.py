#!/usr/bin/env python3


import cv2
import rospy

from state_clue_detect import Clue_DetectState
from state_paved_road import Paved_RoadState
from state_dirt_road import Dirt_RoadState
from state_narrow_road import Narrow_RoadState
from state_pedestrian import PedestrianState
from state_roundabout import RoundaboutState
from state_mountain import MountainState
from state_truck import TruckState
from state_idle import Idle

from sensor_msgs.msg import Image
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

        self.red_count = 0
        self.pink_count = 0
        self.clue_board = 0

        self.prev_red_pixels = None
        self.prev_pink_pixels = None
        
        self.cross_walk = False
        self.idle = False
        self.board_detected = False


        self.sub_clk = rospy.Subscriber("/clock", Clock, self.clock_cb)
        self.sub_cam = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.image_cb, queue_size=1)

        self.pub_vel = rospy.Publisher("/B1/cmd_vel", Twist, queue_size=1)
        self.pub_time = rospy.Publisher("/score_tracker", String, queue_size = 1, latch = True)

        self.pub_tape_cam = rospy.Publisher("/tape_img", Image, queue_size = 1)
        self.pub_processed_cam = rospy.Publisher("/processed_img", Image, queue_size = 1)


        self.states = {
           "Clue_Detect": Clue_DetectState(self),
           "Paved_Road": Paved_RoadState(self),
           "Dirt_Road": Dirt_RoadState(self),
           "Narrow_Road": Narrow_RoadState(self),
           "Pedestrian": PedestrianState(self),
           "Roundabout": RoundaboutState(self),
           "Mountain": MountainState(self),
           "Truck": TruckState(self),
           "Idle": Idle(self)
        }

        self.current_state = self.states["Mountain"]
        self.current_state.enter()


    ## Receives image data through ROS, analyzes the location of the track in the image using OpenCV, 
    #  and sends appropriate valeu for yaw to the robot to realign itself with the track.
    #  @param self The object pointer
    #  @param data The image data received from the gazebo simulation through ROS
    def image_cb(self, data):
        try:
            self.image_data = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_tape()
        except CvBridgeError as e:
            print(e)


        # TODO: Add clue detection(CNN) code here
        # if homography detected, makes self.board_detected = True
        # In each state, it'll transition to "Send_Clue" state
 

    def clock_cb(self, data):
            self.timer = data
    

    def detect_tape(self):

            if self.image_data is None:
                return
            
            img = self.image_data
            
            frame_height, frame_width, _ = img.shape

            top = int (frame_height * 0.75)
            bottom = int (frame_height * 0.95)
            left = int (frame_width * 0.4)
            right = frame_width

            img_cropped = img[top:bottom, left:right]

            hsv = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2HSV)

            lower_red1 = (0, 150, 230)
            upper_red1 = (10, 255, 255)

            lower_red2 = (170, 150, 230)
            upper_red2 = (179, 255, 255)

            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)

            mask_red = mask_red1 | mask_red2

            current_red_pixels = int(mask_red.sum() / 255)
            if self.prev_red_pixels is None:
                 self.prev_red_pixels = current_red_pixels
                 return
            
            change_in_red_pixel = current_red_pixels - self.prev_red_pixels

            if change_in_red_pixel < 0 and self.red_count == 1:
                 self.cross_walk = True
                            
            if change_in_red_pixel > 4500 and  current_red_pixels > 27000:
                 self.red_count = 1
                 if self.cross_walk is True:
                      self.red_count = 2

            #rospy.loginfo(f"{change_in_red_pixel}, {current_red_pixels}")
            img_a = self.bridge.cv2_to_imgmsg(mask_red, encoding="mono8")
            self.pub_tape_cam.publish(img_a)

            self.prev_red_pixels = current_red_pixels

            lower_pink = (130, 100, 200)
            upper_pink = (170, 255, 255)

            mask_pink = cv2.inRange(hsv, lower_pink, upper_pink)

            current_pink_pixels = int(mask_pink.sum() / 255)
            if self.prev_pink_pixels is None:
                 self.prev_pink_pixels = current_pink_pixels
                 return
            
            change_in_pink_pixel = current_pink_pixels - self.prev_pink_pixels
            # Just for now.
            if change_in_pink_pixel < 0 and self.pink_count == 1:
                 self.cross_walk = True
                            
            if change_in_pink_pixel > 4500 and  current_pink_pixels > 27000:
                 self.pink_count = 1
                 if self.cross_walk is True:
                      self.pink_count = 2


                 
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