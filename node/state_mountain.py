import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError


class MountainState:

    def __init__(self, state_machine):
        self.state_machine = state_machine
        self.bridge = CvBridge()
        self.threshold = 180    
        self.linear_speed = 0.6
        self.kp = 0.02


    def enter(self):
        rospy.loginfo("Entering Mountain state")


    def run(self):

        img = self.state_machine.image_data
        self.drive(img, self.linear_speed)

        return "Mountain"
    

    def exit(self):
        rospy.loginfo("Exiting Mountain state")


    def drive(self, img, speed):
        
        if img is None:
            return

        frame_height, frame_width, _= img.shape

        top = int (frame_height * 0.6)
        bottom = frame_height
        left = 0
        right = frame_width

        img_cropped = img[top:bottom, left:right]
        img_hsv = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2HSV)

        lower_blue = (90, 8, 120)
        upper_blue = (140, 120, 255)
        mask_blue = cv2.inRange(img_hsv, lower_blue, upper_blue)
        img_no_blue = cv2.bitwise_and(img_cropped, img_cropped, mask=cv2.bitwise_not(mask_blue))

        img_hsv2 = cv2.cvtColor(img_no_blue, cv2.COLOR_BGR2HSV)

        lower_grayish = (30, 0, 120)
        upper_grayish = (70, 40, 230)
        mask_gray = cv2.inRange(img_hsv2, lower_grayish, upper_grayish)
        img_no_blue_gray = cv2.bitwise_and(img_no_blue, img_no_blue, mask=cv2.bitwise_not(mask_gray))

        img_grayscaled = cv2.cvtColor(img_no_blue_gray, cv2.COLOR_BGR2GRAY)
        _, img_bin = cv2.threshold(img_grayscaled, self.threshold, 255, cv2.THRESH_BINARY)
        
        contours, hierarchy = cv2.findContours(img_bin, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 5000]
        contour_data = [(cv2.boundingRect(c), c) for c in contours]
        contour_data.sort(key=lambda x: x[0][0], reverse=True)
        #contour_data.sort(key=lambda x: cv2.contourArea(x[1]), reverse = True)

        filtered = []
        for (x, y, w, h), cnt in contour_data:

            is_bottom_touching = (y + h) >= frame_height - 2
            # is_left_edge       = x <= 0.1 * frame_width
            # is_right_edge      = x + w >= 0.7 * frame_width (is_left_edge or is_right_edge) (is_bottom_touching and is_tall and is_not_middle) or
            is_tall            = h >= 0.15 * frame_height
            is_not_middle      = x <= 0.05 * frame_width or x + w >= 0.95 * frame_width

            if ( (is_not_middle and is_tall)):
                filtered.append(((x, y, w, h), cnt))


        if len(filtered) >= 2:
            
            (xL,yL,wL,hL), cnt_left = filtered[0]
            (xR,yR,wR,hR), cnt_right = filtered[1]
            #rospy.loginfo(cv2.contourArea(cnt_left))

            M_left = cv2.moments(cnt_left)
            M_right = cv2.moments(cnt_right)

            if M_left["m00"] > 0 and M_right["m00"] > 0 and hL >= 0.4 * frame_height and hR >= 0.4 * frame_height:
                rospy.loginfo("yo")
                cx_left = int(M_left["m10"] / M_left["m00"])
                cy_left = int(M_left["m01"] / M_left["m00"])
                cx_right = int(M_right["m10"] / M_right["m00"])
                cy_right = int(M_right["m01"] / M_right["m00"])

                left_at_edge  = (xL == 0)
                right_at_edge = (xR + wR == frame_width)

                wide_enough = abs(cx_left - cx_right) >= 300
                
                right_valid = (yR <= 3) and (yR + hR <= 0.4 * frame_height)
                left_valid  = (yL >= 3) and (yL + hL <= 0.4 * frame_height)

                if abs(cx_left - cx_right) <= 300:
                            contour_data = contour_data[:1]

                if left_at_edge and right_at_edge and wide_enough and right_valid and left_valid:   
                        
                        rospy.loginfo(f"double, L:{cx_left}/{frame_width}, R:{cx_right}/{frame_width}")
                        cx_center = (cx_left + cx_right) // 2
                        cy_difference = cy_left - cy_right

                        slope = 1.16

                        if cy_difference > 60:
                            higher_cy = cy_left
                            slope = -1 * slope
                            center_shift = slope * higher_cy
                        elif cy_difference < 60:
                            higher_cy = cy_right
                            center_shift = slope * higher_cy
                        else:
                            center_shift = 0
                        
                        error = center_shift + (frame_width / 2.0) - cx_center
                        
                        if abs(error) < 150:
                            self.state_machine.move.linear.x  = speed
                            self.state_machine.move.angular.z = 0
                        else:
                            self.state_machine.move.linear.x  = speed
                            self.state_machine.move.angular.z = self.kp * error

        if len(filtered) >= 1:

            (x,y,w,h), cnt = filtered[0]
            rospy.loginfo(cv2.contourArea(cnt))

            is_bottom_touching = (y + h) >= frame_height - 2
            is_left_edge       = x <= 0.1 * frame_width
            is_right_edge      = x + w >= 0.7 * frame_width
            is_tall = h >= 0.2 * frame_height
            is_not_middle = x <= 0.2 * frame_width or x + w >= 0.8 * frame_width

            M = cv2.moments(cnt)

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # if ((is_bottom_touching and is_tall and (is_left_edge or is_right_edge)) or (is_not_middle and is_tall)):
                rospy.loginfo(f"single, {x}/{frame_width}")
                # The center of the lane shifts significantly from the center of the frame during steep curve
                # I did "Required shift from frame center proportional to Cy" and it worked well
                # (slope value was experimentally chosen)
                slope = 2
                if cx <= frame_width * 0.5:
                    slope = -1 * slope

                center_shift = slope * cy
                error = center_shift + (frame_width / 2.0) - cx

                self.state_machine.move.linear.x  = self.linear_speed
                self.state_machine.move.angular.z = self.kp * error
                #else:
        else:
            self.state_machine.move.linear.x  = speed
            self.state_machine.move.angular.z = 0            

        with_contours = img_no_blue_gray
        
        for (x, y, w, h), cnt in filtered:
            cv2.rectangle(img_no_blue_gray, (x, y), (x + w, y + h), (0, 255, 0), 3)
            with_contours = cv2.drawContours(img_no_blue_gray, [cnt], -1, (0,255,0), 5)

        img_a = self.bridge.cv2_to_imgmsg(with_contours, encoding="bgr8")
        #img_a = self.bridge.cv2_to_imgmsg(img_bin, encoding="mono8")
        self.state_machine.pub_processed_cam.publish(img_a)

        self.state_machine.pub_vel.publish(self.state_machine.move)

