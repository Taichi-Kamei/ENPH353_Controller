#!/usr/bin/env python3

import rospy
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class StateMachine:
    def __init__(self):
        self.bridge = CvBridge()
        self.threshold = 161
        self.linear_speed = 4 
        self.kp = 0.07

        self.pub = rospy.Publisher("/B1/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.image_cb, queue_size=1)

    ## Receives image data through ROS, analyzes the location of the track in the image using OpenCV, 
    #  and sends appropriate valeu for yaw to the robot to realign itself with the track.
    #  @param self The object pointer
    #  @param data The image data received from the gazebo simulation through ROS
    def image_cb(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        frame_height, frame_width, _ = img.shape

        ## Crops the frame so only the bottom part is used for recognizing the path
        img_cropped = img[220:frame_height-10, 0:frame_width]
        img_gray = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2GRAY)
        _, img_bin = cv2.threshold(img_gray, self.threshold, 255, cv2.THRESH_BINARY)
        img_inv = cv2.bitwise_not(img_bin)

        M = cv2.moments(img_inv)
        move = Twist()

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])

            #Computes the error from the center of the frame and uses it for proproportional control of the yaw
            error = (frame_width / 2.0) - cx
            move.linear.x  = self.linear_speed
            move.angular.z = self.kp * error
        else:
            move.linear.x  = 1.0
            move.angular.z = 0.5

        self.pub.publish(move)

## Main function which initializes the state machine and instanciate callback function whenvever new data is ready
#
def main():
    rospy.init_node("state_machine")
    sm = StateMachine()
    rospy.spin()

if __name__ == "__main__":
    main()