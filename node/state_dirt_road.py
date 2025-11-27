import cv2
import rospy

from Cv_Bridge import CvBridge, CvBridgeError


class Dirt_RoadState:

    def __init__(self, state_machine):
        self.state_machine = state_machine
        self.bridge = CvBridge()
    

    def enter(self):
        rospy.loginfo("Entering Dirt Road state")


    def run(self):

        img = self.state_machine.image_data

        if self.state_machine.pink_count == 2:
            return "Off_Road"

        self.drive(img, self.linear_speed)

        return "Dirt_Road"
    

    def exit(self):
        rospy.loginfo("Exiting Dirt Road state")

    def drive(self, img, speed):
        
        if img is None:
            return

        frame_height, frame_width, _= img.shape

        top = int (frame_height * 0.55)
        bottom = frame_height
        left = 0
        right = frame_width

        img_cropped = img[top:bottom, left:right]
        img_gray = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2GRAY)
        _, img_bin = cv2.threshold(img_gray, self.threshold, 255, cv2.THRESH_BINARY)
        
        contours, hierarchy = cv2.findContours(img_bin, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1300]
        #Sorts the contours from right to left
        contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[0], reverse=True)

        if len(contours) >= 2:
            
            cnt_right = contours[0]
            cnt_left = contours[-1]

            M_left = cv2.moments(cnt_left)
            M_right = cv2.moments(cnt_right)

            if M_left["m00"] > 0 and M_right["m00"] > 0:

                cx_left = int(M_left["m10"] / M_left["m00"])
                cy_left = int(M_left["m01"] / M_left["m00"])
                cx_right = int(M_right["m10"] / M_right["m00"])
                cy_right = int(M_right["m01"] / M_right["m00"])

                cx_center = (cx_left + cx_right) // 2
                cy_difference = cy_left - cy_right

                slope = 1.15
                center_shift = 0

                if cy_difference > 60:
                    higher_cy = cy_left
                    slope = -1 * slope
                    center_shift = slope * higher_cy
                elif cy_difference < 60:
                    higher_cy = cy_right
                    center_shift = slope * higher_cy
                
                error = center_shift + (frame_width / 2.0) - cx_center

                if abs(cx_left - cx_right) <= 300:
                    contours = contours[:-1]
                else:
                    self.state_machine.move.linear.x  = speed
                    self.state_machine.move.angular.z = self.kp * error

        if len(contours) == 1:

            M = cv2.moments(contours[0])

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # The center of the lane shifts significantly from the center of the frame during steep curve
                # I did "Required shift from frame center proportional to Cy" and it worked well
                # (slope value was experimentally chosen)
                slope = 3
                if cx <= frame_width * 0.5:
                    slope = -1 * slope

                center_shift = slope * cy
                error = center_shift + (frame_width / 2.0) - cx

                self.state_machine.move.linear.x  = self.linear_speed
                self.state_machine.move.angular.z = self.kp * error
        
        with_contours = cv2.drawContours(img_cropped, contours, -1, (0,255,0), 5)
        img_a = self.bridge.cv2_to_imgmsg(with_contours, encoding="bgr8")
        self.state_machine.pub_processed_cam.publish(img_a)

        self.state_machine.pub_vel.publish(self.state_machine.move)