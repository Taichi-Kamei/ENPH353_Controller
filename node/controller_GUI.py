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


##
# Backend for the "./Controller_GUI.ui".
# Allows control over the Gazebo simulation world, 
# launching scripts for running the score tracker and the state machine,
# and viewing robot's raw/contour/clue_detection camera view
#
# Requires user to launch the Gazebo simulation beforehand.
# Subscribed to "/B1/rrbot/camera1/image_raw", and custom "/processed_img" ROS nodes
#
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
        self.tape_image = None
        
        self.sim_time_button.clicked.connect(self.SLOT_sim_time)
        self.view_type_button.currentIndexChanged.connect(self.change_view_type)
        self.reset_robot_position_button.clicked.connect(self.SLOT_reset_robot_position)
        self.launch_score_tracker_button.clicked.connect(self.SLOT_launch_score_tracker_script)
        self.launch_state_machine_button.clicked.connect(self.SLOT_launch_state_machine_script)


        self.sub_raw_view = rospy.Subscriber("/B1/rrbot/camera1/image_raw",
                                            Image, self.raw_image_cb, queue_size=1)
        self.sub_contour_view = rospy.Subscriber("/processed_img",
                                            Image, self.contour_image_cb, queue_size= 1)
        self.sub_tape_view = rospy.Subscriber("/tape_img",
                                            Image, self.tape_image_cb, queue_size= 1)
        # TODO Add subscriber for clue detection
        #self.sub_clue_view = rospy.Subscriber("/processed_img", Image, self.clue_detection_image_cb, queue_size= 1)

        self.sim_paused = False
        self.pause_icon = QtGui.QIcon("icons/icons8-pause-30.png")
        self.resume_icon = QtGui.QIcon("icons/icons8-resume-button-30.png")
        self.sim_time_button.setIcon(self.pause_icon)


    def tape_image_cb(self, data):
        self.tape_image = self.bridge.imgmsg_to_cv2(data,"bgr8")

        if self.view_type_button.currentText() == "Clue":
            self.update_view(self.tape_image)
    
    ##
    # Callback function for the view_type: Raw
    # 
    # @param self The object pointer
    # @param data The image which is in imgmsg from ROS
    def raw_image_cb(self, data):
            self.raw_image = self.bridge.imgmsg_to_cv2(data,"bgr8")

            if self.view_type_button.currentText() == "Raw":
                self.update_view(self.raw_image)
    
    ##
    # Callback function for the view_type: Contour
    # 
    # @param self The object pointer
    # @param data The image which is in imgmsg from ROS
    def contour_image_cb(self, data):
        self.contour_image = self.bridge.imgmsg_to_cv2(data,"bgr8")

        if self.view_type_button.currentText() == "Contour":
                self.update_view(self.contour_image)


    ##
    # Callback function for the view_type: Clue
    # 
    # @param self The object pointer
    # @param data The image which is in imgmsg from ROS
    def clue_detection_image_cb(self, data):
        self.clue_image = self.bridge.imgmsg_to_cv2(data,"bgr8")

        if self.view_type_button.currentText() == "Clue":
                self.update_view(self.clue)

    ##
    # Changes the type of view by the type chosen in the combobox on GUI
    # 
    # @param self The object pointer
    def change_view_type(self):
        type = self.view_type_button.currentText()

        if type == "Raw":
            self.update_view(self.raw_image)

        elif type == "Contour":
            self.update_view(self.contour_image)

        elif type == "Clue":
            self.update_view(self.tape_image)

    ##
    # Updates the displayed view on the GUI
    # 
    # @param self The object pointer
    # @param img The image that will be shown on the GUI
    def update_view(self, img):
        if img is None:
            return
        
        pixmap = self.convert_cv_to_pixmap(img)
        pixmap = pixmap.scaled(self.camera_view.width(),
                    self.camera_view.height(), QtCore.Qt.KeepAspectRatio)
        
        self.camera_view.setPixmap(pixmap)


    ##
    # Converts openCV image to QtGui with correct scaling
    # Source: stackoverflow.com/questions/34232632/
    #
    # @param self The object pointer
    # @param cv_img Image as a openCV object
    # @return Type QtGui image
    def convert_cv_to_pixmap(self, cv_img):
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        height, width, channel = cv_img.shape
        bytesPerLine = channel * width
        q_img = QtGui.QImage(cv_img.data, width, height, 
                        bytesPerLine, QtGui.QImage.Format_RGB888)
        return QtGui.QPixmap.fromImage(q_img)
  
    ##
    # Pauses or unpauses the Gazebo simulation.
    # 
    # @param self The object pointer
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

    ##
    # Resets the robot position to the initial position
    # 
    # @param self The object pointer
    def SLOT_reset_robot_position(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        reset_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        
        state = ModelState()
        state.model_name = "B1"
        state.pose.position.x = 5.5
        state.pose.position.y = 2.5
        state.pose.position.z = 0.1

        # Chat did the math for me to make the robot face North (Maybe it's not north idk)
        state.pose.orientation.x = 0
        state.pose.orientation.y = 0
        state.pose.orientation.z = 0.707 
        state.pose.orientation.w = -0.707
        state.reference_frame = "world"

        try:
            reset_service(state)
            rospy.loginfo("Robot reset!")
        except:
            rospy.loginfo("Failed to reset model")    
    

    ##
    # Launches the ./score_tracker.py in the '~/ros_ws/2025_competion' directory when the button on GUI is clicked. 
    #
    # When the script is already running and the user clicks the launch button, 
    # the running script will be terminated before relaunching another script.
    # 
    # @param self The object pointer
    def SLOT_launch_score_tracker_script(self):
        
        script = os.path.expanduser(
                "~/ros_ws/src/2025_competition/enph353/enph353_utils/scripts/score_tracker.py")
        
        if not self.score_tracker_process:
            
            self.score_tracker_process = subprocess.Popen(["python3", script],
                                                        cwd = os.path.dirname(script)) 
        else:
            self.score_tracker_process.terminate()
            self.score_tracker_process.kill()
            self.score_tracker_process = subprocess.Popen(["python3", script],
                                                        cwd = os.path.dirname(script))
            rospy.loginfo("Reopening the Score Tracker Window")
    
    ##
    # Launches the ./state_machine.py when the button on GUI is clicked. 
    # When the script is already running and the user clicks the launch button, 
    # the running script will be terminated before relaunching another script.
    # 
    # @param self The object pointer
    def SLOT_launch_state_machine_script(self):

        if not self.state_machine_process:
            self.state_machine_process = subprocess.Popen(["./state_machine.py"])
        else:
            self.state_machine_process.terminate()
            self.state_machine_process.kill()
            self.state_machine_process = subprocess.Popen(["./state_machine.py"])
            rospy.loginfo("Restarting State Machine")


    ## 
    # Closes all the background processes (score_tracker.py and state_machine.py) when controller_GUI.py is terminated 
    # Uses Qt's event-handling system
    # 
    # @param self The object pointer
    # @param event Qt's event handler
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