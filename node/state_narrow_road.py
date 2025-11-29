import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError


class Narrow_RoadState:

    def __init__(self, state_machine):
        self.state_machine = state_machine
        self.bridge = CvBridge()
        self.threshold = 176    
        self.linear_speed = 2
        self.kp = 0.03

        self.old_cx = None
    

    def enter(self):
        rospy.loginfo("Entering Narrow Road state")


    def run(self):

        img = self.state_machine.image_data

        if self.state_machine.pink_count == 2:
            return "Off_Road"

        self.drive(img, self.linear_speed)

        return "Narrow_Road"
    

    def exit(self):
        rospy.loginfo("Exiting Narrow Road state")


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
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 4000]
        #Sorts the contours from right to left
        contour_data = [(c, cv2.boundingRect(c)) for c in contours]
        contour_data.sort(key=lambda x: x[0][0])

        if len(contours) >= 2:
            
            cnt_right = contour_data[0][0]
            cnt_left = contour_data[-1][0]

            x1,y1,w1,h1 = contour_data[0][1]
            x2,y2,w2,h2 = contour_data[-1][1]

            M_left = cv2.moments(cnt_left)
            M_right = cv2.moments(cnt_right)

            if M_left["m00"] > 0 and M_right["m00"] > 0:

                cx_left = int(M_left["m10"] / M_left["m00"])
                cy_left = int(M_left["m01"] / M_left["m00"])
                cx_right = int(M_right["m10"] / M_right["m00"])
                cy_right = int(M_right["m01"] / M_right["m00"])
                
                if abs(cx_left - cx_right) <= 300:
                    contours = contours[:-1]
                # y1 + h1 = frame_height or x1 = 0
                # y2 + h2 = frame_height or x2 + w2 = frame_width
                #cx_left < 0.2 * frame_width and cx_right > 0.8 * frame_width (y1 + h1 == frame_height (y2 + h2 == frame_height or
                else:              
                    bottom_tol = 3

                    left_bottom_ok  = (y1 + h1) >= frame_height - bottom_tol
                    right_bottom_ok = (y2 + h2) >= frame_height - bottom_tol

                    left_in_range  = y1 <= 0.5 * frame_height
                    right_in_range = y2 <= 0.5 * frame_height

                    left_is_left   = x1 <= 0.2 * frame_width
                    right_is_right = x2 >= 0.85 * frame_width    
                        
                    if left_bottom_ok and right_bottom_ok and left_in_range and right_in_range and left_is_left and right_is_right:
                        rospy.loginfo("wow")
                        cx_center = (cx_left + cx_right) // 2
                        cy_difference = cy_left - cy_right

                        slope = 1.16

                        if cy_difference > 60:
                            higher_cy = cy_left
                            slope = -1 * slope
                            center_shift = slope * higher_cy
                        elif cy_difference < 60:
                            higher_cy = cy_right
                            center_shift = slope * higher_cy
                        else:
                            center_shift = 0
                        
                        error = center_shift + (frame_width / 2.0) - cx_center
                        if abs(cx_left - cx_right) <= 300:
                            contours = contours[:-1]
                        elif abs(error) < 150:
                            self.state_machine.move.linear.x  = speed
                            self.state_machine.move.angular.z = 0
                        else:
                            self.state_machine.move.linear.x  = speed
                            self.state_machine.move.angular.z = self.kp * error

                    if x1 == 0 and x2 + w2 == frame_width and abs(cx_left - cx_right) >= 120:   
                            rospy.loginfo("yo")
                            cx_center = (cx_left + cx_right) // 2
                            cy_difference = cy_left - cy_right

                            slope = 1.16

                            if cy_difference > 60:
                                higher_cy = cy_left
                                slope = -1 * slope
                                center_shift = slope * higher_cy
                            elif cy_difference < 60:
                                higher_cy = cy_right
                                center_shift = slope * higher_cy
                            else:
                                center_shift = 0
                            
                            error = center_shift + (frame_width / 2.0) - cx_center
                            if abs(cx_left - cx_right) <= 300:
                                contours = contours[:-1]
                            elif abs(error) < 150:
                                self.state_machine.move.linear.x  = speed
                                self.state_machine.move.angular.z = 0
                            else:
                                self.state_machine.move.linear.x  = speed
                                self.state_machine.move.angular.z = self.kp * error

        elif len(contours) == 1:

            M = cv2.moments(contours[0])
            x,y,w,h = cv2.boundingRect(contours[0])

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                    #cy < 0.4 * frame_height or cx < 0.3 * frame_width or cx > 0.8 * frame_width

                if (y + h == frame_height and y <= 0.7 * frame_height and (x <= 0.2 * frame_width or x >= 0.75 * frame_width)) or (x == 0 or x + w == frame_width):
                    rospy.loginfo("single")
                    # The center of the lane shifts significantly from the center of the frame during steep curve
                    # I did "Required shift from frame center proportional to Cy" and it worked well
                    # (slope value was experimentally chosen)
                    slope = 3.7
                    if cx <= frame_width * 0.5:
                        slope = -1 * slope

                    center_shift = slope * cy
                    error = center_shift + (frame_width / 2.0) - cx

                    self.state_machine.move.linear.x  = self.linear_speed
                    self.state_machine.move.angular.z = self.kp * error
                else:
                    self.state_machine.move.linear.x  = speed
                    self.state_machine.move.angular.z = 0

        #         self.old_cx = cx
        # elif self.old_cx is not None:
        #     self.state_machine.move.linear.x  = 0.4
        #     if self.old_cx < 0.5 * frame_width:
        #         self.state_machine.move.angular.z = 0.2
        #     else:
        #         self.state_machine.move.angular.z = -0.2
        
        for (x, y, w, h), cnt in contour_data:
            cv2.rectangle(img_cropped, (x, y), (x + w, y + h), (0, 255, 0), 3)

        with_contours = cv2.drawContours(img_cropped, contours, -1, (0,255,0), 5)
        img_a = self.bridge.cv2_to_imgmsg(with_contours, encoding="bgr8")
        self.state_machine.pub_processed_cam.publish(img_a)

        self.state_machine.pub_vel.publish(self.state_machine.move)

    # def check_lane_boundary(self, contour_data, line):
        
    #     i = 0
    #     if lane is "left":
    #         i = 1
    #     x,y,w,h = contour_data[i][0]

