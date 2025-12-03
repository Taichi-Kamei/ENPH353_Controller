import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError

class RoundaboutState:

    def __init__(self, state_machine):
        self.state_machine = state_machine
        self.bridge = CvBridge()
        self.threshold = 160
        self.linear_speed = 0.95
        self.kp = 0.04

        self.prev_pink_pixels = None
        self.turned_right = True
        self.second_left_turn = False


    def enter(self):
        rospy.loginfo("Entering Roundabout State")


    def run(self):
              
        img = self.state_machine.image_data

        if img is None:
            return "Roundabout"
        
        if self.detect_pink(img):
            return "Dirt_Road"
        
        if self.second_left_turn:
            return "Post_Roundabout"
        
        # if self.detect_board_contour(img) is not None:
        #     return "Clue_Detect"

        self.drive(img, self.linear_speed)

        return "Roundabout"


    def exit(self):
        rospy.loginfo("Exiting Roundabout State")
        self.turned_right = False
        self.second_left_turn = False

    
    def get_contour_data(self, img):
        if img is None:
            return

        frame_height, frame_width, _= img.shape

        top = int (frame_height * 0.55)
        bottom = frame_height
        left = 0
        right = frame_width

        img_cropped = img[top:bottom, left:right]
        cropped_width, cropped_height,_ = img_cropped.shape
        img_gray = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2GRAY)
        _, img_bin = cv2.threshold(img_gray, self.threshold, 255, cv2.THRESH_BINARY)
        
        contours, hierarchy = cv2.findContours(img_bin, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1300]
        contour_data = [(cv2.boundingRect(c), c) for c in contours]

        for (x, y, w, h), cnt in contour_data:
            cv2.rectangle(img_cropped, (x, y), (x + w, y + h), (0, 255, 0), 3)
        with_contours = cv2.drawContours(img_cropped, contours, -1, (0,255,0), 5)
        img_a = self.bridge.cv2_to_imgmsg(with_contours, encoding="bgr8")
        self.state_machine.pub_processed_cam.publish(img_a)
    
        return contour_data, frame_height, frame_width
    

    def drive(self, img, speed):
        
        contour_data,frame_height, frame_width = self.get_contour_data(img)

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

                if xL == 0 and xR + wR == frame_width and abs(cx_left - cx_right) >= 60:   
                    cx_center = (cx_left + cx_right) // 2
                    cy_difference = cy_left - cy_right

                    slope = 1.1
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
                    if abs(error) < 110:
                        self.state_machine.move.linear.x  = 0.8
                        self.state_machine.move.angular.z = 0.1

        elif len(contour_data) == 1:

            M = cv2.moments(contour_data[0][1])
            (x,y,w,h),_ = contour_data[0]

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                if (y + h == frame_height and y <= 0.7 * frame_height and (x <= 0.2 * frame_width or x >= 0.75 * frame_width)) or (x == 0 or x + w == frame_width):
                    
                    slope = 1.7

                    if self.second_left_turn:
                        slope = 3

                    if cx <= frame_width * 0.5:
                        slope = -1 * slope

                    center_shift = slope * cy
                    error = center_shift + (frame_width / 2.0) - cx

                    rospy.loginfo(error)
                    if self.turned_right and self.detect_board_for_transition(img):
                        self.second_left_turn = True
                        rospy.loginfo("2nd turn")
                        
                    elif error < -120:
                        self.turned_right = True

                    self.state_machine.move.linear.x  = self.linear_speed
                    self.state_machine.move.angular.z = self.kp * error
                else:
                    self.state_machine.move.linear.x  = speed
                    self.state_machine.move.angular.z = 0

        self.state_machine.pub_vel.publish(self.state_machine.move)

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
            threshold = 3900

            rospy.loginfo(f"area: {cnt_area}")

            if cnt_area > threshold:
                return True
        
        return False
    

    def detect_board_contour(self, img):
         
        if img is None:
            return None
        
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        frame_height, frame_width,_ = hsv.shape

        lower_blue = (80, 125, 0)
        upper_blue = (160, 255, 255)
        mask_board = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, hierarchy = cv2.findContours(mask_board, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) >= 1:
            contour = max(contours, key=cv2.contourArea)
            cnt_area = cv2.contourArea(contour)
            rospy.loginfo(f"detect area: {cnt_area}")
            threshold = 5000

            if cnt_area > threshold and cnt_area < threshold + 500:
                return contour
        
        return None


    def detect_pink(self, img):
        if img is None:
            return
        
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
                        
        if change_in_pink_pixel > 4500 and  current_pink_pixels > 25000:
            return True