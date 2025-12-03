import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError


class Off_RoadState:

    def __init__(self, state_machine):
        self.state_machine = state_machine
        self.bridge = CvBridge()
        self.threshold = 180    
        self.linear_speed = 1
        self.kp = 0.04
        self.perpendicular1 = False
        self.perpendicular2 = False
        self.adjusted = False
        self.rough_perpendicular = False
        self.precise_perpendicular = False
        self.lock_board = False
        self.look_around = False

        self.prev_angle = None
        self.aligned = 0


    def enter(self):
        rospy.loginfo("Entering Off Road state")
        self.state_machine.move.linear.x  = 0
        self.state_machine.move.angular.z = 0
        self.state_machine.pub_vel.publish(self.state_machine.move)


    def run(self):

        img = self.state_machine.image_data
        angle = self.orient_tape(img)
        rospy.loginfo(f"angle: {angle}")
        cnt, frame_width = self.detect_pink_tape(img)
        if cnt is not None:
            rospy.loginfo(cv2.contourArea(cnt))
        
        if self.perpendicular1 is False:

            if angle is None and self.prev_angle is not None:
                
                self.state_machine.move.linear.x  = -0.5
                self.state_machine.move.angular.z = 0
                self.state_machine.pub_vel.publish(self.state_machine.move)
                rospy.sleep(0.2)
                self.state_machine.move.linear.x  = 0
                self.state_machine.move.angular.z = 3
                self.state_machine.pub_vel.publish(self.state_machine.move)
                rospy.sleep(0.3)
                self.state_machine.move.linear.x  = 0.5
                self.state_machine.move.angular.z = 0
                self.state_machine.pub_vel.publish(self.state_machine.move)
                rospy.sleep(0.3)
                self.state_machine.move.linear.x  = 0
                self.state_machine.move.angular.z = -3
                self.state_machine.pub_vel.publish(self.state_machine.move)
                rospy.sleep(0.3)
                self.state_machine.move.linear.x  = 0
                self.state_machine.move.angular.z = 0
            elif angle is not None:
                error = angle - 90
                #rospy.loginfo(error)
                self.state_machine.move.linear.x  = 0
                self.state_machine.move.angular.z = -0.2 * error
                self.prev_angle = angle
                
                if abs(error) <= 2.7:
                    self.perpendicular1 = True
            else:
                self.state_machine.move.linear.x  = 0
                self.state_machine.move.angular.z = 0

            self.state_machine.pub_vel.publish(self.state_machine.move)

        elif self.adjusted is False:
            self.state_machine.move.linear.x  = 1
            self.state_machine.move.angular.z = 0
            self.state_machine.pub_vel.publish(self.state_machine.move)
            rospy.sleep(1.3)
            self.state_machine.move.linear.x  = 0
            self.state_machine.move.angular.z = 3
            self.state_machine.pub_vel.publish(self.state_machine.move)
            rospy.sleep(0.9)
            self.state_machine.move.linear.x  = self.linear_speed
            self.state_machine.move.angular.z = 0
            self.state_machine.pub_vel.publish(self.state_machine.move)
            self.adjusted = True
            rospy.sleep(5)
                
        elif self.aligned <= 3 and cnt is not None:
            M = cv2.moments(cnt)

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # shifts the frame center to the right so robot will 
                # be little off to the right of pink tape
                shift = 60
                error = (frame_width / 2 - shift) - cx 
                
                if abs(error) <= 5:
                    self.aligned += 1
                
                self.state_machine.move.linear.x  = 0
                self.state_machine.move.angular.z = self.kp * error
                self.state_machine.pub_vel.publish(self.state_machine.move)
        elif cnt is not None and cv2.contourArea(cnt) <= 1000:
            self.state_machine.move.linear.x  = self.linear_speed
            self.state_machine.move.angular.z = 0
            self.state_machine.pub_vel.publish(self.state_machine.move)
        elif cnt is not None and cv2.contourArea(cnt) >= 9500 and self.lock_board is False:
            self.look_around = True
        
        if self.look_around is True and self.lock_board is False:
            self.state_machine.move.linear.x  = 0
            self.state_machine.move.angular.z = 3
            self.state_machine.pub_vel.publish(self.state_machine.move)
            rospy.sleep(0.2)
            self.state_machine.move.linear.x  = 0
            self.state_machine.move.angular.z = 0
            self.state_machine.pub_vel.publish(self.state_machine.move)
            self.lock_board = self.detect_board_for_transition(img)

        elif self.lock_board:
            return "Clue_Detect"

        # and self.perpendicular2 is False
        # elif angle is not None and self.rough_perpendicular is False:
        #     self.perpendicular2 = True
        #     error = angle
        #     rospy.loginfo(f"error: {error}")
        #     self.state_machine.move.linear.x  = 0
        #     self.state_machine.move.angular.z = 0.15 * error
        #     self.state_machine.pub_vel.publish(self.state_machine.move)
        #     if abs(error) <= 2.7:
        #         self.rough_perpendicular = True
            
        # elif angle is not None and self.precise_perpendicular is False:
        #     #TODO write code for state transition
        #     if abs(error) <= 2.7:
        #         return "Mountain"

        return "Off_Road"
    

    def exit(self):
        rospy.loginfo("Exiting Off Road state")
        self.state_machine.was_off_road_state = True
        self.state_machine.board_left = True


    def detect_board_for_transition(self, img):
         
        if img is None:
            return False
        
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_blue = (80, 125, 0)
        upper_blue = (160, 255, 255)
        mask_board = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, hierarchy = cv2.findContours(mask_board, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) >= 1:
            contour = max(contours, key=cv2.contourArea)
            
            cnt_area = cv2.contourArea(contour)
            threshold = 11000

            rospy.loginfo(f"area: {cnt_area}")

            # if cnt_area > threshold and cnt_area < threshold + 2000:
            #     return True
            
            threshold = 20000
            if cnt_area > threshold:
                return True
    
        return False
        


    def orient_tape(self, img):

        if img is None:
            return None
        
        frame_height, frame_width, _ = img.shape

        top = int (frame_height * 0.75)
        bottom = int (frame_height * 0.95)
        left = int (frame_width * 0.4)
        right = frame_width

        img_cropped = img[top:bottom, left:right]
        hsv = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2HSV)

        lower_pink = (130, 100, 200)
        upper_pink = (170, 255, 255)

        mask_pink = cv2.inRange(hsv, lower_pink, upper_pink)
        edges = cv2.Canny(mask_pink, 50, 150)
        lines = cv2.HoughLines(edges, 1, np.pi/180, 100)
        if lines is None:
            return None
    
        rho, theta = lines[0][0]
        angle_deg = theta * 180 / np.pi
        #rospy.loginfo(angle_deg)

        img_a = self.bridge.cv2_to_imgmsg(edges, encoding="mono8")
        self.state_machine.pub_tape_cam.publish(img_a)

        return angle_deg
    
    
    def detect_pink_tape(self, img):
        if img is None:
            return None, None
        
        frame_height, frame_width, _ = img.shape

        top = 0
        bottom = int (frame_height * 0.95)
        left = 0
        right = frame_width

        img_cropped = img[top:bottom, left:right]

        hsv = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2HSV)

        lower_pink = (130, 100, 200)
        upper_pink = (170, 255, 255)

        mask_pink = cv2.inRange(hsv, lower_pink, upper_pink)
        
        contours, hierarchy = cv2.findContours(mask_pink, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return None, frame_width


        # img_a = self.bridge.cv2_to_imgmsg(mask_pink, encoding="mono8")
        # self.state_machine.pub_tape_cam.publish(img_a)

        with_contours = cv2.drawContours(img_cropped, contours, -1, (0,255,0), 5)
        img_a = self.bridge.cv2_to_imgmsg(with_contours, encoding="bgr8")
        self.state_machine.pub_processed_cam.publish(img_a)

        return max(contours, key=cv2.contourArea), frame_width
