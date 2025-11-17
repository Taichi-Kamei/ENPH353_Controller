class Truck:
    def __init__(self, state_machine):
        self.state_machine = state_machine
    
    def enter(self):
        rospy.loginfo("Entering truck state")

    def run(self):
        pass

    def exit(self):
        rospy.loginfo("Exiting truck state")