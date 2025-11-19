import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError

class Drive_GreenState:

    def __init__(self, state_machine):
        self.state_machine = state_machine
        self.bridge = CvBridge()
        self.threshold = 170    
        self.linear_speed = 2
        self.kp = 0.03


    def enter(self):
        rospy.loginfo("Entering Drive Green State")


    def run(self):
        img = self.state_machine.image_data

        if self.detect_red(img) and self.state_machine.red_count == 0:
                self.state_machine.red_count += 1
                return "Pedestrian"
        
        self.drive(img)
        
        return "Drive_Green"


    def exit(self):
        rospy.loginfo("Exiting Drive Green State")
    

    def drive(self, img):
        
        if img is None:
            return

        frame_height, frame_width, _ = img.shape

        top = int (frame_height * 0.5)
        bottom = frame_height
        left = 0
        right = frame_width

        ## Crops the frame so only the bottom part is used for recognizing the path
        img_cropped = img[top:bottom, left:right]
        img_gray = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2GRAY)
        _, img_bin = cv2.threshold(img_gray, self.threshold, 255, cv2.THRESH_BINARY)
        
        contours, hierarchy = cv2.findContours(img_bin, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1200]

        contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[0])

        move = self.state_machine.move

        if len(contours) >= 2:
            cnt_left = contours[0]
            cnt_right = contours[-1]

            M_left = cv2.moments(cnt_left)
            M_right = cv2.moments(cnt_right)

            if M_left["m00"] > 0 and M_right["m00"] > 0:
                cx_left = int(M_left["m10"] / M_left["m00"])
                cx_right = int(M_right["m10"] / M_right["m00"])

                lane_center = (cx_left + cx_right) // 2

                error = (frame_width / 2.0) - lane_center
                move.linear.x  = self.linear_speed
                move.angular.z = self.kp * error

        with_contours = cv2.drawContours(img_cropped, contours, -1, (0,255,0), 5)

        img_a = self.bridge.cv2_to_imgmsg(with_contours, encoding="bgr8")
        self.state_machine.pub_processed_cam.publish(img_a)
        self.state_machine.pub_vel.publish(move)

    
    def detect_red(self, img):
        
        if img is None:
            return
        
        frame_height, frame_width, _ = img.shape

        top = int (frame_height * 0.9)
        bottom = frame_height
        left = 0
        right = frame_width

        img_cropped = img[top:bottom, left:right]

        hsv = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2HSV)

        lower_red1 = (0, 100, 100)
        upper_red1 = (10, 255, 255)

        lower_red2 = (170, 100, 100)
        upper_red2 = (180, 255, 255)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 | mask2

        return mask.sum() > 0