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

        frame_height, frame_width, _= img.shape

        top = int (frame_height * 0.55)
        bottom = frame_height
        left = 0
        right = frame_width

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

            # if len(contours) == 3:
            #     cnt_right = contours[0]
            #     cnt_left = contours[1]

            M_left = cv2.moments(cnt_left)
            M_right = cv2.moments(cnt_right)

            if M_left["m00"] > 0 and M_right["m00"] > 0:
                cx_left = int(M_left["m10"] / M_left["m00"])
                cy_left = int(M_left["m01"] / M_left["m00"])
                cx_right = int(M_right["m10"] / M_right["m00"])
                cy_right = int(M_right["m01"] / M_right["m00"])

                cx_center = (cx_left + cx_right) // 2
                cy_difference = cy_left - cy_right

                slope = 1.15
                higher_cy = cy_right
                if cy_difference > 60:
                    higher_cy = cy_left
                    slope = -1 * slope

                center_shift = slope * higher_cy
                error = center_shift + (frame_width / 2.0) - cx_center

                #correction_factor = 1.0
                # if cy_difference > 60:    
                #     #rospy.loginfo(f"center: {lane_center}/{frame_width / 2} diff: {cy_difference} Cy_R: {cy_right} / {frame_height}")
                #     if cy_right < 0.2 * frame_height:
                #         correction_factor = 1.5
                #     elif cy_right < 0.4 * frame_height:
                #         correction_factor = 1.3
                #     elif cy_right > 0.7 * frame_height:
                #         correction_factor = 1.1
                # elif cy_difference < -60:
                #     #rospy.loginfo(f"center: {lane_center}/{frame_width / 2} diff: {cy_difference} Cy_L: {cy_left} / {frame_height}")
                #     if cy_left < 0.2 * frame_height:
                #         correction_factor = 0.5
                #     elif cy_left < 0.4 * frame_height:
                #         correction_factor = 0.7
                #     elif cy_left > 0.7 * frame_height:
                #         correction_factor = 0.9

                # error = (frame_width / 2.0) - (lane_center * correction_factor)
                #rospy.loginfo(abs(cx_left - cx_right))

                if abs(cx_left - cx_right) <= 300:
                    contours = contours[:-1]
                else:
                    #rospy.loginfo(f"error: {error} angular: {self.kp * error}")
                    # if abs(error) <= 25:
                    #     self.state_machine.move.linear.x  = 2
                    #     self.state_machine.move.angular.z = 0
                    # else:
                    self.state_machine.move.linear.x  = speed
                    self.state_machine.move.angular.z = self.kp * error

        if len(contours) == 1:
            M = cv2.moments(contours[0])

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # The center of the lane shifts significantly from the center of the frame during steep curve
                # I did "Required shift from frame center proportional to Cy" and it worked well
                # (slope value was experimentally chosen)
                slope = 3
                if cx <= frame_width * 0.5:
                    slope = -1 * slope

                center_shift = slope * cy
                error = center_shift + (frame_width / 2.0) - cx

                self.state_machine.move.linear.x  = self.linear_speed
                self.state_machine.move.angular.z = self.kp * error

                #rospy.loginfo(f"cx: {cx} cy: {cy}")
                #rospy.loginfo(f"error: {error} angular: {self.kp * error} normalized: {cy_normalized}")
        
        with_contours = cv2.drawContours(img_cropped, contours, -1, (0,255,0), 5)

        img_a = self.bridge.cv2_to_imgmsg(with_contours, encoding="bgr8")
        self.state_machine.pub_processed_cam.publish(img_a)
        self.state_machine.pub_vel.publish(self.state_machine.move)