import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError


class Pre_Off_RoadState:

    def __init__(self, state_machine):
        self.state_machine = state_machine
        self.bridge = CvBridge()
        self.threshold = 172    
        self.linear_speed = 0.7
        self.kp = 0.02

        self.prev_pink_pixels = None
    

    def enter(self):
        rospy.loginfo("Entering Pre Off Road state")
        self.state_machine.move.linear.x  = 0
        self.state_machine.move.angular.z = 3
        self.state_machine.pub_vel.publish(self.state_machine.move)
        rospy.sleep(0.2)
        self.state_machine.move.linear.x  = 0.5
        self.state_machine.move.angular.z = 0
        self.state_machine.pub_vel.publish(self.state_machine.move)
        rospy.sleep(0.4)



    def run(self):

        img = self.state_machine.image_data

        if self.detect_pink(img):
            return "Off_Road"
        
        if self.detect_board_contour(img) is not None:
            return "Clue_Detect"

        self.drive(img, self.linear_speed)

        return "Pre_Off_Road"
    

    def exit(self):
        rospy.loginfo("Exiting Pre Off Road state")
        self.state_machine.move.linear.x  = -0.3
        self.state_machine.move.angular.z = 0
        self.state_machine.pub_vel.publish(self.state_machine.move)
        rospy.sleep(0.3)


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
        contour_data = [(cv2.boundingRect(c), c) for c in contours]
        contour_data.sort(key=lambda x: x[0][0])

        filtered = []
        for (x, y, w, h), cnt in contour_data:

            is_tall            = h >= 0.15 * frame_height
            is_not_middle      = x == 0 or x + w >= 0.95 * frame_width

            if ( (is_not_middle and is_tall)):
                filtered.append(((x, y, w, h), cnt))              

        if len(filtered) >= 2:
            
            (xR,yR,wR,hR), cnt_right = filtered[0]
            (xL,yL,wL,hL), cnt_left = filtered[-1]

            M_left = cv2.moments(cnt_left)
            M_right = cv2.moments(cnt_right)

            if M_left["m00"] > 0 and M_right["m00"] > 0:

                cx_left = int(M_left["m10"] / M_left["m00"])
                cy_left = int(M_left["m01"] / M_left["m00"])
                cx_right = int(M_right["m10"] / M_right["m00"])
                cy_right = int(M_right["m01"] / M_right["m00"])

                if xL == 0 and xR + wR == frame_width and abs(cx_left - cx_right) >= 120:   
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

        elif len(filtered) == 1:

            (x,y,w,h), cnt = filtered[0]

            is_bottom_touching = (y + h) >= frame_height - 2
            is_high_enough     = y <= 0.7 * frame_height
            is_left_edge       = x <= 0.2 * frame_width
            is_right_edge      = x + w >= 0.75 * frame_width
            is_touching_side   = (x == 0) or (x + w == frame_width)

            M = cv2.moments(cnt)

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                if (is_bottom_touching and is_high_enough and (is_left_edge or is_right_edge)) or is_touching_side:
                    rospy.loginfo("single")
                    # The center of the lane shifts significantly from the center of the frame during steep curve
                    # I did "Required shift from frame center proportional to Cy" and it worked well
                    # (slope value was experimentally chosen)
                    slope = 3.2
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


    def detect_board_contour(self, img):
         
        if img is None:
            return None
        
        frame_height, frame_width,_ = img.shape

        top = int (frame_height * 0.4)
        bottom = frame_height
        left = 0
        right = frame_width

        img_cropped = img[top:bottom, left:right]
        
        hsv = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2HSV)

        lower_blue = (105, 100, 50)
        upper_blue = (120, 255, 255)
        mask_board = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, hierarchy = cv2.findContours(mask_board, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
        

        with_contours = cv2.drawContours(img_cropped, contours, -1, (0,255,0), 5)
        img_a = self.bridge.cv2_to_imgmsg(with_contours, encoding="bgr8")
        self.state_machine.pub_tape_cam.publish(img_a)

        if len(contours) >= 1:
            contour = max(contours, key=cv2.contourArea)
            cnt_area = cv2.contourArea(contour)
            rospy.loginfo(f"detect area: {cnt_area}")
            threshold = 20000

            if cnt_area > threshold and cnt_area < threshold + 500:
                return contour
        
        return None


    def detect_pink(self, img):
        if img is None:
            return False
        
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

        current_pink_pixels = int(mask_pink.sum() / 255)
        if self.prev_pink_pixels is None:
                self.prev_pink_pixels = current_pink_pixels
                return False
        
        change_in_pink_pixel = current_pink_pixels - self.prev_pink_pixels


        if current_pink_pixels > 27500:
            return True
        
        self.prev_pink_pixels = current_pink_pixels

        return False

