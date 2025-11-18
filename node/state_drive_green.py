import cv2
import rospy


class Drive_GreenState:

    def __init__(self, state_machine):
        self.state_machine = state_machine

        self.threshold = 161
        self.linear_speed = 2
        self.kp = 0.03


    def enter(self):
        rospy.loginfo("Entering Drive Green State")


    def run(self):
        img = self.state_machine.image_data

        if self.detect_red(img) and self.state_machine.red_count == 0:
                self.state_machine.red_count += 1
                return "Pedestrian"
        
        self.drive(img)
        #self.state_machine.pub_time.publish(end_timer)

        return "Drive_Green"


    def exit(self):
        rospy.loginfo("Exiting Drive Green State")
    

    def drive(self, img):
        
        if img is None:
            return

        frame_height, frame_width, _ = img.shape

        ## Crops the frame so only the bottom part is used for recognizing the path
        img_cropped = img[220:frame_height-10, 0:frame_width]
        img_gray = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2GRAY)
        _, img_bin = cv2.threshold(img_gray, self.threshold, 255, cv2.THRESH_BINARY)

        M = cv2.moments(img_bin)
        move = self.state_machine.move

        ros_img = self.state_machine.bridge.cv2_to_imgmsg(img_bin, encoding="mono8")
        self.state_machine.pub_processed_cam.publish(ros_img)

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])

            #Computes the error from the center of the frame and uses it for proproportional control of the yaw
            error = (frame_width / 2.0) - cx
            move.linear.x  = self.linear_speed
            move.angular.z = self.kp * error
        else:
            move.linear.x  = 1.0
            move.angular.z = 0.5

        self.state_machine.pub_vel.publish(move)

    
    def detect_red(self, img):
        
        #ToDo

        return False