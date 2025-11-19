import rospy

class Clue_DetectState:

    def __init__(self, state_machine):
        self.state_machine = state_machine

    def enter(self):
        rospy.loginfo("Entering Pedestrian state")


    def run(self):
        

        pass


    def exit(self):
        self.state_machine.board_detected = False
        rospy.loginfo("Exiting Pedestrian State")
        