import rospy


class Idle:

    def __init__(self, state_machine):
        self.state_machine = state_machine
    
    def enter(self):
        rospy.loginfo("Entering Idle")

    def run(self):
        #self.state_machine.pub_time.publish(end_time)

        return "Idle"
    
    def exit(self):
        rospy.loginfo("Exiting Idle")