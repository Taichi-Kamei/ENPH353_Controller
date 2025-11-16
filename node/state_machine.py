import rospy
import cv2
import transition

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class StateMachine:
    def __init__(self):
        self.bridge = CvBridge()
        self.threshold = 161
        self.linear_speed = 4 
        self.kp = 0.07

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.image_cb, queue_size=1)


## Main function which initializes the state machine and instanciate callback function whenvever new data is ready
#
def main():
    rospy.init_node("state_machine")
    sm = StateMachine()
    rospy.spin()

if __name__ == "main":
    main()