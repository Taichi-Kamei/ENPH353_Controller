import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
import math

class Drive_GreenState:

    def __init__(self, state_machine):
        self.state_machine = state_machine
        self.bridge = CvBridge()
        self.threshold = 170    
        self.linear_speed = 1
        self.kp = 0.03


    def enter(self):
        rospy.loginfo("Entering Drive Green State")


    def run(self):
        
        img = self.state_machine.image_data
    
        if self.state_machine.red_count == 1:

            # self.state_machine.move.linear.x  = 0.5
            # self.state_machine.move.angular.z = 0
            # self.state_machine.pub_vel.publish(self.state_machine.move)

            return "Pedestrian"
        
        if self.state_machine.pink_count == 1:
            #TODO For now idle but later dirt raod state
            return "Idle"
        
        self.drive(img, self.linear_speed)

        return "Drive_Green"


    def exit(self):
        rospy.loginfo("Exiting Drive Green State")
    

    def drive(self, img, speed):
        
        if img is None:
            return

        frame_height, frame_width, _ = img.shape

        top = int (frame_height * 0.55)
        bottom = frame_height
        left = 0
        right = frame_width
        #left line
        left_L = 0
        right_L = int(frame_width * 0.4)
        #right line
        left_R = int(frame_width * 0.6)
        right_R = frame_width


        # img_cropped_L = img[top:bottom, left_L:right_L]
        # img_gray_L = cv2.cvtColor(img_cropped_L, cv2.COLOR_BGR2GRAY)
        # _, img_bin_L = cv2.threshold(img_gray_L, self.threshold, 255, cv2.THRESH_BINARY)

        # contours_L, hierarchy = cv2.findContours(img_bin_L, cv2.RETR_TREE,
        #                                cv2.CHAIN_APPROX_SIMPLE)
        # contours_L = [cnt for cnt in contours_L if cv2.contourArea(cnt) > 1200]

        # img_cropped_R = img[top:bottom, left_R:right_R]
        # img_gray_R = cv2.cvtColor(img_cropped_R, cv2.COLOR_BGR2GRAY)
        # _, img_bin_R = cv2.threshold(img_gray_R, self.threshold, 255, cv2.THRESH_BINARY)

        # contours_R, hierarchy = cv2.findContours(img_bin_R, cv2.RETR_TREE,
        #                                cv2.CHAIN_APPROX_SIMPLE)
        # contours_R = [cnt for cnt in contours_R if cv2.contourArea(cnt) > 1200]


        # if len(contours_L) > 0:
        #     M_left = cv2.moments(contours_L[0])
        #     cx_left = int(M_left["m10"] / M_left["m00"])
        
        # if len(contours_R) > 0:
        #     M_right = cv2.moments(contours_R[0])
        #     cx_right = int(M_right["m10"] / M_right["m00"])

        
        # if len(contours_L) > 0 and len(contours_R) > 0:         
        #     lane_center = (cx_left + left_R + cx_right) // 2

        #     error = (frame_width / 2.0) - lane_center
        #     self.state_machine.move.linear.x  = speed
        #     self.state_machine.move.angular.z = self.kp * error
        # elif len(contours_L) > 0:
        #     self.state_machine.move.linear.x  = 0.3
        #     self.state_machine.move.angular.z = -3
        # else:
        #     self.state_machine.move.linear.x  = 0.3
        #     self.state_machine.move.angular.z = 3


        # img_cropped_Middle = img[top:bottom, right_L:left_R]

        # with_contours_L = cv2.drawContours(img_cropped_L, contours_L, -1, (0,255,0), 5)
        # with_contours_R = cv2.drawContours(img_cropped_R, contours_R, -1, (0,255,0), 5)

        # with_contours = cv2.hconcat([with_contours_L, img_cropped_Middle, with_contours_R])


        img_cropped = img[top:bottom, left:right]
        img_gray = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2GRAY)
        _, img_bin = cv2.threshold(img_gray, self.threshold, 255, cv2.THRESH_BINARY)

        
        contours, hierarchy = cv2.findContours(img_bin, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1300]

        contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[0], reverse=True)


        if len(contours) >= 2:
            cnt_right = contours[0]
            cnt_left = contours[-1]

            M_left = cv2.moments(cnt_left)
            M_right = cv2.moments(cnt_right)

            if M_left["m00"] > 0 and M_right["m00"] > 0:
                cx_left = int(M_left["m10"] / M_left["m00"])
                cy_left = int(M_left["m01"] / M_left["m00"])
                cx_right = int(M_right["m10"] / M_right["m00"])
                cy_right = int(M_right["m01"] / M_right["m00"])

                lane_center = (cx_left + cx_right) // 2
                #lane_center = math.sqrt((cx_left - cx_right)**2 + (cy_left - cy_right)**2) // 2

                error = (frame_width / 2.0) - lane_center
                if error <= 10:
                    self.state_machine.move.linear.x  = speed
                    self.state_machine.move.angular.z = 0.005 * error
                else:
                    self.state_machine.move.linear.x  = speed
                    self.state_machine.move.angular.z = self.kp * error

        elif len(contours) == 1:
            M = cv2.moments(contours[0])

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                if cx <= frame_width * 0.5:
                    #self.state_machine.move.linear.x  = 0.5
                    if cy >= frame_height * 0.3:
                        self.state_machine.move.linear.x  = 0.7
                        self.state_machine.move.angular.z = -1
                    elif cy >= 0.5:
                        self.state_machine.move.linear.x  = 0.4
                        self.state_machine.move.angular.z = -3
                    else:
                        self.state_machine.move.linear.x  = 0.2
                        self.state_machine.move.angular.z = -4
                else:
                    if cy >= frame_height * 0.3:
                        self.state_machine.move.linear.x  = 0.7
                        self.state_machine.move.angular.z = 1
                    elif cy >= 0.5:
                        self.state_machine.move.linear.x  = 0.4
                        self.state_machine.move.angular.z = 3
                    else:
                        self.state_machine.move.linear.x  = 0.2
                        self.state_machine.move.angular.z = 4

            
        with_contours = cv2.drawContours(img_cropped, contours, -1, (0,255,0), 5)

        img_a = self.bridge.cv2_to_imgmsg(with_contours, encoding="bgr8")
        self.state_machine.pub_processed_cam.publish(img_a)
        self.state_machine.pub_vel.publish(self.state_machine.move)