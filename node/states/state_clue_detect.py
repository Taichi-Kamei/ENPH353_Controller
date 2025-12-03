import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError

from clue_detector import ClueDetector

class Clue_DetectState:

    def __init__(self, state_machine):
        self.state_machine = state_machine
        self.bridge = CvBridge()
        self.clue_detector = ClueDetector()

        self.aligned = False
        self.clue_sent = False
        self.board_position_left = True

        self.linear_speed = 1
        self.kp = 0.02

        self.transition_key = None
        self.transition_from_clue = {
           "1": "Paved_Road",
           "2": "Paved_Road",
           "3": "Truck",
           "4": "Roundabout",
           "5": "Narrow_Road",
           "6": "Pre_Off_Road",
           "7": "Mountain",
           "8": "Idle"   
        }

        
    def enter(self):
        rospy.loginfo("Entering Clue Detect state")

        self.state_machine.move.linear.x  = 0.5
        self.state_machine.move.angular.z = 0
        self.state_machine.pub_vel.publish(self.state_machine.move)
        rospy.sleep(0.2)
        self.state_machine.move.linear.x  = 0
        self.state_machine.move.angular.z = 0
        self.state_machine.pub_vel.publish(self.state_machine.move)


    def run(self):

        img = self.state_machine.image_data

        M_initial = cv2.moments(self.state_machine.board_contour)
        if M_initial["m00"] > 0:
                cx = int(M_initial["m10"] / M_initial["m00"])

                if cx > 0.5 * self.state_machine.frame_width_board:
                    self.board_position_left = False

        if img is None:
            return "Clue_Detect"
        
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        frame_height, frame_width,_ = hsv.shape

        lower_blue = (105, 150, 50)
        upper_blue = (120, 255, 150)
        mask_board = cv2.inRange(hsv, lower_blue, upper_blue)

        contours, hierarchy = cv2.findContours(mask_board, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        #contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 15000]
        # contour = self.state_machine.board_contour
        # frame_width = self.state_machine.frame_width_board

        if len(contours) >= 1:
            contour = max(contours, key=cv2.contourArea)
            cnt_area = cv2.contourArea(contour)
            M = cv2.moments(contour)

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                if self.aligned and self.clue_sent:
                    
                    slope = 0.4

                    shift = slope * cy / 2
                    if self.board_position_left:
                        shift = -shift

                    error =  (frame_width / 2) - (cx + shift)

                    self.state_machine.move.linear.x  = 0
                    self.state_machine.move.angular.z = -self.kp * error
                    self.state_machine.pub_vel.publish(self.state_machine.move)
                    rospy.loginfo(f"shift: {shift}, error: {abs(error)}")
                
                    if abs(error) >= 80 and abs(error) <= 120:
                        
                        return self.transition_from_clue[self.transition_key]
                    
                elif self.aligned:
                    
                    self.state_machine.move.linear.x  = 0.3
                    self.state_machine.move.angular.z = 0
                    self.state_machine.pub_vel.publish(self.state_machine.move)
                    rospy.sleep(0.2)
                    self.state_machine.move.linear.x  = 0
                    self.state_machine.move.angular.z = 0
                    self.state_machine.pub_vel.publish(self.state_machine.move)
                    self.clue_detect(img)
                else:
                    
                    error = (frame_width / 2) - cx

                    if abs(error) <= 20:
                        self.aligned = True
                    else:
                        self.state_machine.move.linear.x  = 0
                        self.state_machine.move.angular.z = self.kp * error
                        self.state_machine.pub_vel.publish(self.state_machine.move)
                                      
        return "Clue_Detect"


    def exit(self):
        rospy.loginfo("Exiting Clue Detect State")

        self.state_machine.board_detected = 0
        self.aligned = False
        self.go_close = False
        self.clue_sent = False

    
    def clue_detect(self, img):
        
        id, value = self.clue_detector.detect_clue(img)

        if self.clue_detector.get_board_mask() is not None:
            board_mask = self.bridge.cv2_to_imgmsg(self.clue_detector.get_board_mask(), encoding="bgr8")
            self.state_machine.pub_board_mask_cam.publish(board_mask)

        if self.clue_detector.get_plate() is not None:
            plate = self.bridge.cv2_to_imgmsg(self.clue_detector.get_plate(), encoding="bgr8")
            self.state_machine.pub_flattened_plate_cam.publish(plate)

        if self.clue_detector.get_letter_mask() is not None:
            letter_mask = self.bridge.cv2_to_imgmsg(self.clue_detector.get_letter_mask(), encoding="bgr8")
            self.state_machine.pub_letters_cam.publish(letter_mask)

        if value is not None:
            if self.state_machine.was_narrow_state:
                self.state_machine.pub_time.publish(f"Team14,password,6,{value}")
                self.transition_key = "6"
            else:
                self.state_machine.pub_time.publish(f"Team14,password,{id},{value}")
                self.transition_key = f"{id}"
            
            
            self.clue_sent = True
            
