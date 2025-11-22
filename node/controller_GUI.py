#!/usr/bin/env python3

import os
import cv2
import sys
import rospy
import subprocess

from PyQt5 import QtCore, QtGui, QtWidgets
from python_qt_binding import loadUi

from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import Image
from std_srvs.srv import Empty



class Controller_App(QtWidgets.QMainWindow):

    def __init__(self):
        super(Controller_App, self).__init__()
        loadUi("./Controller_GUI.ui", self)
        
        self.score_tracker_process = None
        self.state_machine_process = None

        self.bridge = CvBridge()
        self.raw_image = None
        self.contour_image = None
        self.clue_detection_image = None
        
        self.sim_time_button.clicked.connect(self.SLOT_sim_time)
        self.reset_robot_position_button.clicked.connect(self.SLOT_reset_robot_position)
        self.view_type_button.currentIndexChanged.connect(self.change_view_type)
        self.launch_score_tracker_button.clicked.connect(self.SLOT_launch_score_tracker_script)
        self.launch_state_machine_button.clicked.connect(self.SLOT_launch_state_machine_script)


        self.sub_raw_view = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.raw_image_cb, queue_size=1)
        self.sub_contour_view = rospy.Subscriber("/processed_img", Image, self.contour_image_cb, queue_size= 1)
        # TODO Add subscriber for clue detection
        #self.sub_clue_view = rospy.Subscriber("/processed_img", Image, self.clue_detection_image_cb, queue_size= 1)

        self.sim_paused = False
        self.pause_icon = QtGui.QIcon("icons/icons8-pause-30.png")
        self.resume_icon = QtGui.QIcon("icons/icons8-resume-button-30.png")
        self.sim_time_button.setIcon(self.pause_icon)


    def raw_image_cb(self, data):
            self.raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.view_type_button.currentText() == "Raw":
                self.update_view(self.raw_image)
    

    def contour_image_cb(self, data):
        self.contour_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        if self.view_type_button.currentText() == "Contour":
                self.update_view(self.contour_image)


    def clue_detection_image_cb(self, data):
        self.clue_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        if self.view_type_button.currentText() == "Clue":
                self.update_view(self.clue)


    def change_view_type(self):
        type = self.view_type_button.currentText()

        if type == "Raw":
            self.update_view(self.raw_image)

        elif type == "Contour":
            self.update_view(self.contour_image)

        elif type == "Clue":
            self.update_view(self.contour_image)


    def update_view(self, img):
        if img is None:
            return
        
        pixmap = self.convert_cv_to_pixmap(img)
        pixmap = pixmap.scaled(self.camera_view.width(),
                    self.camera_view.height(),
                    QtCore.Qt.KeepAspectRatio)
        
        self.camera_view.setPixmap(pixmap)


    ##
    # Converts openCV image to QtGui with correct scaling
    # Source: stackoverflow.com/questions/34232632/
    #
    #  @param self The object pointer
    # @param cv_img Image as a openCV object
    # @return Type QtGui image
    def convert_cv_to_pixmap(self, cv_img):
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        height, width, channel = cv_img.shape
        bytesPerLine = channel * width
        q_img = QtGui.QImage(cv_img.data, width, height, 
                        bytesPerLine, QtGui.QImage.Format_RGB888)
        return QtGui.QPixmap.fromImage(q_img)
  

    def SLOT_sim_time(self):
        
        if self.sim_paused is False:
            rospy.wait_for_service('/gazebo/pause_physics')
            rospy.ServiceProxy('/gazebo/pause_physics', Empty)()
            self.sim_time_button.setIcon(self.resume_icon)
            
            self.sim_paused = True
        else:
            rospy.wait_for_service('/gazebo/unpause_physics')
            rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
            self.sim_time_button.setIcon(self.pause_icon)
            self.sim_paused = False


    def SLOT_reset_robot_position(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        reset_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        #TODO Figure out the reset position
        state = ModelState()
        state.model_name = "B1"
        state.pose.position.x = -8
        state.pose.position.y = -8
        state.pose.position.z = 0
        state.pose.orientation.w = 1
        state.reference_frame = "world"

        try:
            reset_service(state)
            rospy.loginfo("Robot reset!")
        except:
            rospy.loginfo("Failed to reset model")    
    
    
    def SLOT_launch_score_tracker_script(self):
        script = os.path.expanduser("~/ros_ws/src/2025_competition/enph353/enph353_utils/scripts/score_tracker.py")
        self.score_tracker_process = subprocess.Popen(["python3", script], cwd = os.path.dirname(script)) 
    

    def SLOT_launch_state_machine_script(self):
        self.state_machine_process = subprocess.Popen(["./state_machine.py"])


    ## 
    # Closes all the background processes (score_tracker.py and state_machine.py) when controller_GUI.py is terminated 
    # Uses Qt's event-handling system
    #
    def closeEvent(self, event):
        if self.score_tracker_process:
            try:
                self.score_tracker_process.terminate()
                self.score_tracker_process.kill()
            except:
                pass

        if self.state_machine_process:
            try:
                self.state_machine_process.terminate()
                self.state_machine_process.kill()
            except:
                pass

        event.accept()



if __name__ == "__main__":
    rospy.init_node("controller_GUI")
    app = QtWidgets.QApplication(sys.argv)
    my_app = Controller_App()
    my_app.show()
    sys.exit(app.exec_())