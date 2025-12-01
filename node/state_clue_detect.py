import cv2
import rospy

class Clue_DetectState:

    def __init__(self, state_machine):
        self.state_machine = state_machine

    def enter(self):
        rospy.loginfo("Entering Pedestrian state")
        self.state_machine.move.linear.x  = 0
        self.state_machine.move.angular.z = 0
        self.state_machine.pub_vel.publish(self.state_machine.move)


    def run(self):
        contour, frame_width = self.state_machine.board_contour
        M = cv2.moments(contour)

        if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                error = (frame_width / 2) - cx
                
                if abs(error) <= 3:
                    self.clue_detect()

                
                self.state_machine.move.linear.x  = 0
                self.state_machine.move.angular.z = self.kp * error
                self.state_machine.pub_vel.publish(self.state_machine.move)

        


        return self.state_machine.transition_states[self.state_machine.prev_state]


    def exit(self):
        self.state_machine.board_detected = False
        rospy.loginfo("Exiting Pedestrian State")
        self.state_machine.prev_state = "Clue_Detect"

    
    def clue_detect(self):
         #TODO
         pass