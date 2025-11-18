class TruckState:
    def __init__(self, state_machine):
        self.state_machine = state_machine
    
    def enter(self):
        self.state_machine.rospy.loginfo("Entering truck state")

    def run(self):
        pass

    def exit(self):
        self.state_machine.rospy.loginfo("Exiting truck state")