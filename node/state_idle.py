import rospy


class Idle:
    

    def __init__(self, state_machine):
        self.state_machine = state_machine
    
    def enter(self):
        rospy.loginfo("Entering Idle")
        
    def run(self):
        
        # This is done just for the time trial.
        if self.state_machine.idle is False:
            self.state_machine.pub_time.publish("team14,1234,-1,END")
            self.state_machine.idle = True
        
        return "Idle"
    
    def exit(self):
        rospy.loginfo("Exiting Idle")