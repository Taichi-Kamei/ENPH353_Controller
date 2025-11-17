import cv2

class Drive_GreenState:

    def __init__(self, state_machine):
        self.state_machine = state_machine

    def enter(self):
        rospy.loginfo("Entering Drive Green State")

    def run(self):
        img = self.state_machine.image_data

        frame_height, frame_width, _ = img.shape

        ## Crops the frame so only the bottom part is used for recognizing the path
        img_cropped = img[220:frame_height-10, 0:frame_width]
        img_gray = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2GRAY)
        _, img_bin = cv2.threshold(img_gray, self.threshold, 255, cv2.THRESH_BINARY)
        img_inv = cv2.bitwise_not(img_bin)

        M = cv2.moments(img_inv)
        move = self.state_machine.Twist()

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])

            #Computes the error from the center of the frame and uses it for proproportional control of the yaw
            error = (frame_width / 2.0) - cx
            move.linear.x  = self.linear_speed
            move.angular.z = self.kp * error
        else:
            move.linear.x  = 1.0
            move.angular.z = 0.5

        self.state_machine.pub.publish(move)



    def exit(self):
        rospy.loginfo("Exiting Drive Green State")
    