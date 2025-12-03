import rospy


class Idle:
    

    def __init__(self, state_machine):
        self.state_machine = state_machine
    
    def enter(self):
        rospy.loginfo("Entering Idle")
        self.state_machine.pub_time.publish("team14,1234,-1,END")
        
    def run(self):
        
        self.state_machine.move.linear.x  = 0
        self.state_machine.move.angular.z = 0
        self.state_machine.pub_vel.publish(self.state_machine.move)

        return "Idle"
    
    def exit(self):
        rospy.loginfo("Exiting Idle")