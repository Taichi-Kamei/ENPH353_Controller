import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError

from clue_detector import ClueDetector

class Clue_DetectState:

    def __init__(self, state_machine):
        self.state_machine = state_machine

        self.linear_speed = 1
        self.kp = 0.03
        
        self.aligned = False
        self.clue_sent = False

        self.board_position_left = True

        #Kinda chopped since some prev_prev_state is Clue_Detect state and some aren't but deal with it
        self.transition_from_clue = {
           ("Paved_Road",     "Paved_Road")    : "Paved_Road",
           ("Pedestrian",     "Post_Crosswalk"): "Paved_Road",
           ("Clue_Detect",    "Paved_Road")    : "Truck",
           ("Truck",          "Roundabout")    : "Roundabout",
           ("Roundabout",     "Dirt_Road")     : "Narrow_Road",
           ("Clue_Detect",    "Narrow_Road")   : "Narrow_Road",
           ("Narrow_Road",    "Off_Road")      : "Mountain",
           ("Clue_Detect",     "Mountain")     : "Idle",
        }

        self.bridge = CvBridge()
        self.clue_detector = ClueDetector()

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

        M_initial = cv2.moments(self.state_machine.board_contour)
        if M_initial["m00"] > 0:
                cx = int(M_initial["m10"] / M_initial["m00"])

                if cx > 0.5 * self.state_machine.frame_width_board:
                    self.board_position_left = False

        if self.state_machine.image_data is None:

            return "Clue_Detect"
        
        hsv = cv2.cvtColor(self.state_machine.image_data, cv2.COLOR_BGR2HSV)

        frame_height, frame_width,_ = hsv.shape

        lower_blue = (80, 125, 0)
        upper_blue = (160, 255, 255)
        mask_board = cv2.inRange(hsv, lower_blue, upper_blue)

        contours, hierarchy = cv2.findContours(mask_board, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        #contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 15000]
        # contour = self.state_machine.board_contour
        # frame_width = self.state_machine.frame_width_board

        if len(contours) >= 1:
            contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(contour)

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])


                if self.aligned and self.clue_sent:
                    shift = 80
                    if self.board_position_left:
                        shift = -shift

                    error = (frame_width / 2) - (cx + shift)

                    self.state_machine.move.linear.x  = 0
                    self.state_machine.move.angular.z = -self.kp * error
                    self.state_machine.pub_vel.publish(self.state_machine.move)
                    rospy.loginfo(error)
                
                    if abs(error) >= 70:
                        
                        return self.transition_from_clue[
                            (self.state_machine.str_prev_prev_state, self.state_machine.str_prev_state)]
                    
                elif self.aligned:
                    
                    self.state_machine.move.linear.x  = 0.5
                    self.state_machine.move.angular.z = 0
                    self.state_machine.pub_vel.publish(self.state_machine.move)
                    rospy.sleep(0.2)
                    self.state_machine.move.linear.x  = 0
                    self.state_machine.move.angular.z = 0
                    self.state_machine.pub_vel.publish(self.state_machine.move)
                    self.clue_detect()

                else:
                    
                    error = (frame_width / 2) - cx
                    rospy.loginfo(error)
                    
                    self.state_machine.move.linear.x  = 0
                    self.state_machine.move.angular.z = self.kp * error
                    self.state_machine.pub_vel.publish(self.state_machine.move)
                    
                    if abs(error) <= 20:
                        self.aligned = True  
                    
        return "Clue_Detect"


    def exit(self):
        rospy.loginfo("Exiting Clue Detect State")

        self.state_machine.board_detected = 0
        self.aligned = False
        self.go_close = False

    
    def clue_detect(self):
        id, value = self.clue_detector.detect_clue(self.state_machine.image_data)

        board_mask = self.bridge.cv2_to_imgmsg(self.clue_detector.get_board_mask(), encoding="bgr8")
        self.state_machine.pub_board_mask_cam.publish(board_mask)

        plate = self.bridge.cv2_to_imgmsg(self.clue_detector.get_plate(), encoding="bgr8")
        self.state_machine.pub_flattened_plate_cam.publish(plate)

        letter_mask = self.bridge.cv2_to_imgmsg(self.clue_detector.get_letter_mask(), encoding="bgr8")
        self.state_machine.pub_letters_cam.publish(letter_mask)

        
        # self.state_machine.pub_time.publish(f"Team14,password,{id},{value}")
        print(f"Team14,password,{id},{value}")
        # self.clue_sent = True
