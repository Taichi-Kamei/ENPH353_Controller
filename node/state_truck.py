import rospy
import cv2

from cv_bridge import CvBridge, CvBridgeError

class TruckState:
    def __init__(self, state_machine):
        self.state_machine = state_machine
        self.bridge = CvBridge()

        self.count = 0

        self.prev_image = None
        self.current_image = None
    
    def enter(self):
        rospy.loginfo("Entering truck state")
        self.state_machine.move.linear.x = 0
        self.state_machine.move.angular.z = 0
        self.state_machine.pub_vel.publish(self.state_machine.move)

    def run(self):
            
            self.current_image = self.state_machine.image_data

            if self.current_image is None:
                return "Truck"
            
            frame_height, frame_width, _ = self.current_image.shape

            top = int (frame_height * 0.6)
            bottom = frame_height
            left = 0
            right = int(0.4 * frame_width)

            self.current_image = self.current_image[top:bottom, left:right]

            if self.prev_image is None:
                self.prev_image = self.current_image
                return "Truck"

            diff_img = cv2.absdiff(self.prev_image, self.current_image)
            gray_img = cv2.cvtColor(diff_img, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY)

            img_a = self.bridge.cv2_to_imgmsg(thresh, encoding="mono8")
            self.state_machine.pub_processed_cam.publish(img_a)

            if thresh.sum() == 0:
                rospy.loginfo("No Truck detected")
                self.count += 1
                rospy.sleep(0.2)
                if self.count >= 5:
                    return "Roundabout"
            else:
                self.state_machine.move.linear.x = 0
                self.state_machine.move.angular.z = 0
                self.state_machine.pub_vel.publish(self.state_machine.move)
                rospy.loginfo("Truck detected")

            self.prev_image = self.current_image
            return "Truck"

    def exit(self):
        rospy.loginfo("Exiting truck state")
        self.state_machine.prev_state = "Truck"
        
        self.state_machine.move.linear.x = 0
        self.state_machine.move.angular.z = 2
        self.state_machine.pub_vel.publish(self.state_machine.move)
        rospy.sleep(0.3)