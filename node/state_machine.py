#!/usr/bin/env python3

import rospy
import cv2

from state_drive_green import Drive_GreenState
from state_pedestrian import Pedestrian
from state_truck import Truck

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class State_transition:
    def __init__(self, previous_state, event, next_state):
        self.previous_state = previous_state
        self.event = event
        self.next_state = next_state

class StateMachine:
    def __init__(self):
        self.bridge = CvBridge()
        self.img_data = None
        self.threshold = 161
        self.linear_speed = 4 
        self.kp = 0.07

        self.pub = rospy.Publisher("/B1/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.image_cb, queue_size=1)

        self.states = {
           "Green_Normal": Drive_GreenState(self),
           "Pedestrian": Pedestrian(self),
           "Truck": Truck(self)
        }

        self.current_state = self.states["Green_NormalState"]
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

    def run(self):
        while not rospy.is_shutdown():

            self.next_state = self.current_state.run()

            if self.next_state != self.current_state:
                self.current_state.exit()
                self.current_state = self.next_state
                self.current_state.enter()

## Main function which initializes the state machine and instanciate callback function whenvever new data is ready
#
def main():
    rospy.init_node("state_machine")
    sm = StateMachine()
    sm.run()
    rospy.spin()

if __name__ == "__main__":
    main()