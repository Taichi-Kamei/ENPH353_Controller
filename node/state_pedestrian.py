import rospy

class PedestrianState:
    def __init__(self, state_machine):
        self.state_machine = state_machine


    def enter(self):
        rospy.loginfo("Entering Pedestrian state")
        self.state_machine.pub_time.publish("team14,1234,-1,END")


    def run(self):



        return "Pedestrian"


    def exit(self):
        rospy.loginfo("Exiting Pedestrian State")