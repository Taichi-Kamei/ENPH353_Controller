class Pedestrian:
    def __init__(self, state_machine):
        self.state_machine = state_machine


    def enter(self):
        self.state_machine.rospy.loginfo("Entering Pedestrian state")


    def run(self):
        pass


    def exit(self):
        self.state_machine.rospy.loginfo("Exiting Pedestrian State")