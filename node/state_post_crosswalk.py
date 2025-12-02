import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError

class Post_CrosswalkState:

    def __init__(self, state_machine):
        self.state_machine = state_machine
        self.bridge = CvBridge()
        self.threshold = 160    
        self.linear_speed = 1.2
        self.kp = 0.05


    def enter(self):
        rospy.loginfo("Entering Post Crosswalk State")


    def run(self):
        
        img = self.state_machine.image_data

        if self.state_machine.board_contour is not None:
            return "Clue_Detect"
    
        self.drive(img, self.linear_speed)

        return "Post_Crosswalk"


    def exit(self):
        rospy.loginfo("Exiting Post Crosswalk State")
    

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

        contour_data = [(cv2.boundingRect(c), c) for c in contours]
        # Sorts from right to left
        contour_data.sort(key=lambda x: x[0][0], reverse = True)

        if len(contour_data) >= 2:
            
            (xR,yR,wR,hR), cnt_right = contour_data[0]
            (xL,yL,wL,hL), cnt_left = contour_data[-1]

            M_right = cv2.moments(cnt_right)
            M_left = cv2.moments(cnt_left)   

            if M_left["m00"] > 0 and M_right["m00"] > 0:

                cx_left = int(M_left["m10"] / M_left["m00"])
                cy_left = int(M_left["m01"] / M_left["m00"])
                cx_right = int(M_right["m10"] / M_right["m00"])
                cy_right = int(M_right["m01"] / M_right["m00"])

                cx_center = (cx_left + cx_right) // 2
                cy_difference = cy_left - cy_right # if it's positive, right line Cy is higher (visually it is but opposite for calc since cy is from the top)

                # if xL == 0 and xR + wR == frame_width and abs(cx_left - cx_right) >= 60:   
                #     cx_center = (cx_left + cx_right) // 2
                #     cy_difference = cy_left - cy_right

                #     slope = 1.1
                #     if cy_difference > 60:
                #         higher_cy = cy_left
                #         slope = -1 * slope
                #         center_shift = slope * higher_cy
                #     elif cy_difference < 60:
                #         higher_cy = cy_right
                #         center_shift = slope * higher_cy
                #     else:
                #         center_shift = 0
                    
                #     error = center_shift + (frame_width / 2.0) - cx_center
                #     if abs(error) < 100:
                #         self.state_machine.move.linear.x  = 0.8
                #         self.state_machine.move.angular.z = 0

                # slope = 1.16
                # center_shift = 0

                # if cy_difference > 60:
                #     higher_cy = cy_left
                #     slope = -1 * slope
                #     center_shift = slope * higher_cy
                # elif cy_difference < 60:
                #     higher_cy = cy_right
                #     center_shift = slope * higher_cy
                
                # error = center_shift + (frame_width / 2.0) - cx_center
                # rospy.loginfo(f"xL: {xL}, {xL + wL}, {frame_width}")
                # rospy.loginfo(f"xR: {xR}, {xR + wR}, {frame_width}")

                # if abs(cx_left - cx_right) <= 400:
                #     contour_data = contour_data[:1]

                #old code:
                correction_factor = 1.0
                if cy_difference > 60:    
                    #rospy.loginfo(f"center: {lane_center}/{frame_width / 2} diff: {cy_difference} Cy_R: {cy_right} / {frame_height}")
                    if cy_right < 0.2 * frame_height:
                        correction_factor = 1.5
                    elif cy_right < 0.4 * frame_height:
                        correction_factor = 1.3
                    elif cy_right > 0.7 * frame_height:
                        correction_factor = 1.1
                elif cy_difference < -60:
                    #rospy.loginfo(f"center: {lane_center}/{frame_width / 2} diff: {cy_difference} Cy_L: {cy_left} / {frame_height}")
                    if cy_left < 0.2 * frame_height:
                        correction_factor = 0.5
                    elif cy_left < 0.4 * frame_height:
                        correction_factor = 0.7
                    elif cy_left > 0.7 * frame_height:
                        correction_factor = 0.9
                error = (frame_width / 2.0) - (cx_center * correction_factor)
                #rospy.loginfo(abs(cx_left - cx_right))
                # Was inside else statement!!
                #rospy.loginfo(f"error: {error} angular: {self.kp * error}")
                if abs(error) <= 25:
                    self.state_machine.move.linear.x  = self.linear_speed
                    self.state_machine.move.angular.z = 0
                else:
                    self.state_machine.move.linear.x  = self.linear_speed
                    self.state_machine.move.angular.z = self.kp * error
                
                self.state_machine.pub_vel.publish(self.state_machine.move)

        elif len(contour_data) == 1:

            M = cv2.moments(contour_data[0][1])
            (x,y,w,h),_ = contour_data[0]

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                if (y + h == frame_height and y <= 0.7 * frame_height and (x <= 0.2 * frame_width or x >= 0.75 * frame_width)) or (x == 0 or x + w == frame_width):

                    slope = 2.7
                    if cx <= frame_width * 0.5:
                        slope = -1 * slope

                    center_shift = slope * cy
                    error = center_shift + (frame_width / 2.0) - cx

                    self.state_machine.move.linear.x  = self.linear_speed
                    self.state_machine.move.angular.z = self.kp * error
                else:
                    self.state_machine.move.linear.x  = speed
                    self.state_machine.move.angular.z = 0
        

        for (x, y, w, h), cnt in contour_data:
            cv2.rectangle(img_cropped, (x, y), (x + w, y + h), (0, 255, 0), 3)
        with_contours = cv2.drawContours(img_cropped, contours, -1, (0,255,0), 5)
        img_a = self.bridge.cv2_to_imgmsg(with_contours, encoding="bgr8")
        self.state_machine.pub_processed_cam.publish(img_a)

        self.state_machine.pub_vel.publish(self.state_machine.move)