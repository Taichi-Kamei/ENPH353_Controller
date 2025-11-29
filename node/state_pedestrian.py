import rospy
import cv2

from cv_bridge import CvBridge, CvBridgeError

class PedestrianState:

    def __init__(self, state_machine):
        self.state_machine = state_machine

        self.bridge = CvBridge()
        self.prev_image = None
        self.current_image = None


    def enter(self):
        rospy.loginfo("Entering Pedestrian state")
        
        self.state_machine.move.linear.x = 0
        self.state_machine.move.angular.z = 2
        self.state_machine.pub_vel.publish(self.state_machine.move)
        rospy.sleep(0.2)
        self.state_machine.move.linear.x = 0
        self.state_machine.move.angular.z = 0
        self.state_machine.pub_vel.publish(self.state_machine.move)
        

    def run(self):
        
        if self.state_machine.red_count == 2:
            return "Paved_Road"
        
        self.current_image = self.state_machine.image_data
        frame_height, frame_width, _ = self.current_image.shape

        if self.current_image is None:
            return "Pedestrian"

        top = int (frame_height * 0.55)
        bottom = frame_height
        left = 0
        right = frame_width

        self.current_image = self.current_image[top:bottom, left:right]

        if self.prev_image is None:
            self.prev_image = self.current_image
            return "Pedestrian"

        diff_img = cv2.absdiff(self.prev_image, self.current_image)
        gray_img = cv2.cvtColor(diff_img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY)

        if thresh.sum() == 0:
            self.state_machine.move.linear.x = 2
            self.state_machine.move.angular.z = 0

            self.state_machine.pub_vel.publish(self.state_machine.move)

            rospy.loginfo("No Pedestrian detected")
        else:
            rospy.loginfo("Pedestrian detected")

        self.prev_image = self.current_image

        img_a = self.bridge.cv2_to_imgmsg(thresh, encoding="mono8")
        self.state_machine.pub_processed_cam.publish(img_a)

        return "Pedestrian"


    def exit(self):
        rospy.loginfo("Exiting Pedestrian State")
        #self.state_machine.pub_time.publish("team14,1234,-1,END")