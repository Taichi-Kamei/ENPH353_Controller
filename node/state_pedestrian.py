import rospy

class PedestrianState:

    def __init__(self, state_machine):
        self.state_machine = state_machine
        self.move = self.state_machine.move


    def enter(self):
        rospy.loginfo("Entering Pedestrian state")
        
        self.move.linear.x = 0
        self.move.linear.z = 0
        

    def run(self):

        self.move.linear.x = 2
        self.move.linear.z = 0

        self.state_machine.pub_vel.publish(self.move)


        if self.state_machine.red_count == 2:
            return "Drive_Green"

        return "Pedestrian"


    def exit(self):
        rospy.loginfo("Exiting Pedestrian State")
        #self.state_machine.pub_time.publish("team14,1234,-1,END")