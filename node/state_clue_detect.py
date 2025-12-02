import cv2
import rospy

class Clue_DetectState:

    def __init__(self, state_machine):
        self.state_machine = state_machine

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
                    #TODO Add clue transmission code here
                    self.clue_detect()

                    return self.transition_from_clue[self.state_machine.str_prev_prev_state, self.state_machine.str_prev_state]

                self.state_machine.move.linear.x  = 0
                self.state_machine.move.angular.z = self.kp * error
                self.state_machine.pub_vel.publish(self.state_machine.move)


        return "Clue_Detect"


    def exit(self):
        self.state_machine.board_detected = False
        rospy.loginfo("Exiting Pedestrian State")

    
    def clue_detect(self):
         #TODO
         pass