import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError

class Pre_TruckState:

    def __init__(self, state_machine):
        self.state_machine = state_machine
        self.bridge = CvBridge()
        self.threshold = 160    
        self.linear_speed = 1.2
        self.kp = 0.05


    def enter(self):
        rospy.loginfo("Entering Pre Truck State")


    def run(self):
        
        img = self.state_machine.image_data

        if img is None:
            return "Pre_Truck"

        cnt_data, frame_height, frame_width = self.get_contour_data(img)


        if self.check_intersection(cnt_data, frame_height, frame_width):
            return "Truck"
        
        self.drive(cnt_data, frame_height, frame_width, self.linear_speed)

        return "Pre_Truck"


    def exit(self):
        rospy.loginfo("Exiting Pre Truck State")

    
    def check_intersection(self, contour_data, frame_height, frame_width):
        
        if len(contour_data) >= 1:

            highest_contour = min(contour_data, key=lambda x: x[0][1])
            (_,y,w,h),_ = highest_contour

            if y + h <= 0.2 * frame_height and h < 0.05 * frame_height and w >= 0.6 * frame_width:
                return True

        return False

    
    def get_contour_data(self, img):
        if img is None:
            return

        frame_height, frame_width, _= img.shape

        top = int (frame_height * 0.55)
        bottom = frame_height
        left = 0
        right = frame_width

        img_cropped = img[top:bottom, left:right]
        cropped_width, cropped_height,_ = img_cropped.shape
        img_gray = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2GRAY)
        _, img_bin = cv2.threshold(img_gray, self.threshold, 255, cv2.THRESH_BINARY)
        
        contours, hierarchy = cv2.findContours(img_bin, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1300]
        contour_data = [(cv2.boundingRect(c), c) for c in contours]

        for (x, y, w, h), cnt in contour_data:
            cv2.rectangle(img_cropped, (x, y), (x + w, y + h), (0, 255, 0), 3)
        with_contours = cv2.drawContours(img_cropped, contours, -1, (0,255,0), 5)
        img_a = self.bridge.cv2_to_imgmsg(with_contours, encoding="bgr8")
        self.state_machine.pub_processed_cam.publish(img_a)
    
        return contour_data, frame_height, frame_width
    

    def drive(self, contour_data, frame_height, frame_width, speed):

        contour_data.sort(key=lambda x: x[0][0], reverse = True)

        if len(contour_data) >= 2:
            
            (xR,yR,wR,hR), cnt_right = contour_data[0]
            (xL,yL,wL,hL), cnt_left = contour_data[-1]

            M_right = cv2.moments(cnt_right)
            M_left = cv2.moments(cnt_left)   

            if M_left["m00"] > 0 and M_right["m00"] > 0:

                cx_left = int(M_left["m10"] / M_left["m00"])
                cy_left = int(M_left["m01"] / M_left["m00"])
                cx_right = int(M_right["m10"] / M_right["m00"])
                cy_right = int(M_right["m01"] / M_right["m00"])

                cx_center = (cx_left + cx_right) // 2
                cy_difference = cy_left - cy_right # if it's positive, right line Cy is higher (visually it is but opposite for calc since cy is from the top)

                if xL == 0 and xR + wR == frame_width and abs(cx_left - cx_right) >= 60:   
                    cx_center = (cx_left + cx_right) // 2
                    cy_difference = cy_left - cy_right

                    slope = 1.1
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
                    if abs(error) < 100:
                        self.state_machine.move.linear.x  = 0.8
                        self.state_machine.move.angular.z = 0

        elif len(contour_data) == 1:

            M = cv2.moments(contour_data[0][1])
            (x,y,w,h),_ = contour_data[0]

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                if (y + h == frame_height and y <= 0.7 * frame_height and (x <= 0.2 * frame_width or x >= 0.75 * frame_width)) or (x == 0 or x + w == frame_width):
                    #rospy.loginfo("single")

                    # The center of the lane shifts significantly from the center of the frame during steep curve
                    # I did "Required shift from frame center proportional to Cy" and it worked well
                    # (slope value was experimentally chosen)
                    slope = 2.7
                    if cx <= frame_width * 0.5:
                        slope = -1 * slope

                    center_shift = slope * cy
                    error = center_shift + (frame_width / 2.0) - cx

                    self.state_machine.move.linear.x  = self.linear_speed
                    self.state_machine.move.angular.z = self.kp * error
                else:
                    self.state_machine.move.linear.x  = speed
                    self.state_machine.move.angular.z = 0

        self.state_machine.pub_vel.publish(self.state_machine.move)