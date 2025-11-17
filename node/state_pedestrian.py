class Pedestrian:
    def __init__(self, state_machine):
        self.state_machine = state_machine


    def enter(self):
        rospy.loginfo("Entering Pedestrian state")


    def run(self):
        pass


    def exit(self):
        rospy.loginfo("Exiting Pedestrian State")