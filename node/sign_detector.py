#!/usr/bin/env python
import rospy
import cv2
import sys
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QLabel, QPushButton, QSlider, QSpinBox, 
                             QDoubleSpinBox)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.uic import loadUi

class SignDetector(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.current_cv_image = None
        
        # Initialize ROS
        rospy.init_node('sign_detector', anonymous=True)
        
        # Load UI file
        self.load_ui()
        
        # Initialize parameters
        self.init_parameters()
        
        # Setup dynamic controls
        self.setup_dynamic_controls()
        
        # Connect signals
        self.connect_signals()
        
        # Setup ROS subscriber
        self.image_sub = rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, self.image_callback)
        
        # Update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.process_and_update)
        self.timer.start(33)  # 30Hz
        
    def load_ui(self):
        # Get the directory where this script is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        ui_file = os.path.join(script_dir, 'sign_detector.ui')
        
        if not os.path.exists(ui_file):
            rospy.logerr(f"UI file not found: {ui_file}")
            sys.exit(1)
            
        loadUi(ui_file, self)
        rospy.loginfo("UI loaded successfully")
        
    def init_parameters(self):
        # Color parameters with default values (HSV format)
        self.blue_params = {
            'H_min': 100, 'H_max': 140,
            'S_min': 150, 'S_max': 255,
            'V_min': 50, 'V_max': 255
        }
        
        self.gray_params = {
            'H_min': 0, 'H_max': 180,
            'S_min': 0, 'S_max': 50,
            'V_min': 50, 'V_max': 200
        }
        
        # Processing parameters
        self.MIN_SIGN_AREA = 5000
        self.contour_epsilon = 0.02
        
    def setup_dynamic_controls(self):
        # Create slider pairs for blue controls
        self.blue_hue_min_slider, self.blue_hue_max_slider, self.blue_hue_min_label, self.blue_hue_max_label = self.create_slider_pair(
            self.blueHueLayout, 'blue', 'H', 0, 179, self.blue_params['H_min'], self.blue_params['H_max'])
        
        self.blue_sat_min_slider, self.blue_sat_max_slider, self.blue_sat_min_label, self.blue_sat_max_label = self.create_slider_pair(
            self.blueSatLayout, 'blue', 'S', 0, 255, self.blue_params['S_min'], self.blue_params['S_max'])
        
        self.blue_val_min_slider, self.blue_val_max_slider, self.blue_val_min_label, self.blue_val_max_label = self.create_slider_pair(
            self.blueValLayout, 'blue', 'V', 0, 255, self.blue_params['V_min'], self.blue_params['V_max'])
        
        # Create slider pairs for gray controls
        self.gray_hue_min_slider, self.gray_hue_max_slider, self.gray_hue_min_label, self.gray_hue_max_label = self.create_slider_pair(
            self.grayHueLayout, 'gray', 'H', 0, 179, self.gray_params['H_min'], self.gray_params['H_max'])
        
        self.gray_sat_min_slider, self.gray_sat_max_slider, self.gray_sat_min_label, self.gray_sat_max_label = self.create_slider_pair(
            self.graySatLayout, 'gray', 'S', 0, 255, self.gray_params['S_min'], self.gray_params['S_max'])
        
        self.gray_val_min_slider, self.gray_val_max_slider, self.gray_val_min_label, self.gray_val_max_label = self.create_slider_pair(
            self.grayValLayout, 'gray', 'V', 0, 255, self.gray_params['V_min'], self.gray_params['V_max'])
        
    def create_slider_pair(self, layout, color_type, channel, min_val, max_val, default_min, default_max):
        # Min slider
        min_slider = QSlider(Qt.Horizontal)
        min_slider.setRange(min_val, max_val)
        min_slider.setValue(default_min)
        layout.addWidget(min_slider)
        
        # Max slider
        max_slider = QSlider(Qt.Horizontal)
        max_slider.setRange(min_val, max_val)
        max_slider.setValue(default_max)
        layout.addWidget(max_slider)
        
        # Value labels
        min_label = QLabel(str(default_min))
        min_label.setMinimumWidth(30)
        max_label = QLabel(str(default_max))
        max_label.setMinimumWidth(30)
        
        layout.addWidget(min_label)
        layout.addWidget(QLabel("-"))
        layout.addWidget(max_label)
        
        return min_slider, max_slider, min_label, max_label
        
    def connect_signals(self):
        # Connect blue slider signals
        self.blue_hue_min_slider.valueChanged.connect(lambda v: self.update_param('blue', 'H_min', v, self.blue_hue_min_label))
        self.blue_hue_max_slider.valueChanged.connect(lambda v: self.update_param('blue', 'H_max', v, self.blue_hue_max_label))
        self.blue_sat_min_slider.valueChanged.connect(lambda v: self.update_param('blue', 'S_min', v, self.blue_sat_min_label))
        self.blue_sat_max_slider.valueChanged.connect(lambda v: self.update_param('blue', 'S_max', v, self.blue_sat_max_label))
        self.blue_val_min_slider.valueChanged.connect(lambda v: self.update_param('blue', 'V_min', v, self.blue_val_min_label))
        self.blue_val_max_slider.valueChanged.connect(lambda v: self.update_param('blue', 'V_max', v, self.blue_val_max_label))
        
        # Connect gray slider signals
        self.gray_hue_min_slider.valueChanged.connect(lambda v: self.update_param('gray', 'H_min', v, self.gray_hue_min_label))
        self.gray_hue_max_slider.valueChanged.connect(lambda v: self.update_param('gray', 'H_max', v, self.gray_hue_max_label))
        self.gray_sat_min_slider.valueChanged.connect(lambda v: self.update_param('gray', 'S_min', v, self.gray_sat_min_label))
        self.gray_sat_max_slider.valueChanged.connect(lambda v: self.update_param('gray', 'S_max', v, self.gray_sat_max_label))
        self.gray_val_min_slider.valueChanged.connect(lambda v: self.update_param('gray', 'V_min', v, self.gray_val_min_label))
        self.gray_val_max_slider.valueChanged.connect(lambda v: self.update_param('gray', 'V_max', v, self.gray_val_max_label))
        
        # Connect parameter controls
        self.areaSpin.valueChanged.connect(self.update_min_area)
        self.epsilonSpin.valueChanged.connect(self.update_epsilon)
        
        # Connect buttons
        self.saveButton.clicked.connect(self.save_parameters)
        self.resetButton.clicked.connect(self.reset_parameters)
    
    def update_param(self, color_type, param, value, label_widget=None):
        getattr(self, f'{color_type}_params')[param] = value
        if label_widget:
            label_widget.setText(str(value))
    
    def update_min_area(self, value):
        self.MIN_SIGN_AREA = value
    
    def update_epsilon(self, value):
        self.contour_epsilon = value
    
    def image_callback(self, msg):
        try:
            self.current_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Image conversion error: {str(e)}")
    
    def process_and_update(self):
        if self.current_cv_image is None:
            return
        
        # Process the image
        results = self.process_image(self.current_cv_image.copy())
        self.update_displays(*results)
    
    def process_image(self, image):
        # Make copies for different processing steps
        original = image.copy()
        contour_display = image.copy()
        
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create blue mask
        blue_lower = np.array([self.blue_params['H_min'], self.blue_params['S_min'], self.blue_params['V_min']])
        blue_upper = np.array([self.blue_params['H_max'], self.blue_params['S_max'], self.blue_params['V_max']])
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
        
        # Clean up blue mask
        kernel = np.ones((5,5), np.uint8)
        blue_mask_clean = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask_clean = cv2.morphologyEx(blue_mask_clean, cv2.MORPH_CLOSE, kernel)
        
        # Find contours in blue mask
        blue_contours, _ = cv2.findContours(blue_mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        sign_detected = False
        sign_image = None
        plate_image = None
        gray_mask_display = None
        
        largest_area = 0
        if blue_contours:
            largest_blue = max(blue_contours, key=cv2.contourArea)
            largest_area = cv2.contourArea(largest_blue)
            
            # Draw all blue contours
            cv2.drawContours(contour_display, blue_contours, -1, (0, 255, 0), 2)
            
            if largest_area > self.MIN_SIGN_AREA:
                sign_detected = True
                
                # Draw the largest contour in red
                cv2.drawContours(contour_display, [largest_blue], -1, (0, 0, 255), 3)
                
                # Extract sign
                sign_image = self.extract_perspective(original, largest_blue)
                
                if sign_image is not None:
                    # Process the extracted sign for gray plate
                    hsv_sign = cv2.cvtColor(sign_image, cv2.COLOR_BGR2HSV)
                    
                    # Create gray mask on the sign
                    gray_lower = np.array([self.gray_params['H_min'], self.gray_params['S_min'], self.gray_params['V_min']])
                    gray_upper = np.array([self.gray_params['H_max'], self.gray_params['S_max'], self.gray_params['V_max']])
                    gray_mask = cv2.inRange(hsv_sign, gray_lower, gray_upper)
                    
                    # Clean gray mask
                    gray_mask_clean = cv2.morphologyEx(gray_mask, cv2.MORPH_OPEN, kernel)
                    gray_mask_display = cv2.cvtColor(gray_mask_clean, cv2.COLOR_GRAY2BGR)
                    
                    # Find gray contours
                    gray_contours, _ = cv2.findContours(gray_mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    
                    if gray_contours:
                        largest_gray = max(gray_contours, key=cv2.contourArea)
                        
                        # Extract plate from sign
                        plate_image = self.extract_perspective(sign_image, largest_gray, output_size=(300, 200))
        
        # Create blue mask display (colorized)
        blue_mask_display = cv2.cvtColor(blue_mask_clean, cv2.COLOR_GRAY2BGR)
        
        # Update status
        status_info = f"Blue contours: {len(blue_contours)}, "
        status_info += f"Largest area: {largest_area:.0f}, "
        status_info += f"Threshold: {self.MIN_SIGN_AREA}"
        
        return original, blue_mask_display, contour_display, sign_image, gray_mask_display, plate_image, status_info, sign_detected
    
    def extract_perspective(self, image, contour, output_size=(300, 400)):
        epsilon = self.contour_epsilon * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        if len(approx) != 4:
            return None
            
        # Order points
        points = self.order_points(approx.reshape(4, 2))
        width, height = output_size
        
        dst_points = np.array([[0, 0], [width-1, 0], [width-1, height-1], [0, height-1]], dtype='float32')
        
        H = cv2.findHomography(points, dst_points)[0]
        warped = cv2.warpPerspective(image, H, (width, height))
        return warped
    
    def order_points(self, pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        
        return rect
    
    def update_displays(self, original, blue_mask, contours, sign_image, gray_mask, plate_image, status_info, sign_detected):
        # Update all image displays
        self.originalLabel.setPixmap(self.cv2_to_pixmap(original).scaled(400, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.blueMaskLabel.setPixmap(self.cv2_to_pixmap(blue_mask).scaled(400, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.contourLabel.setPixmap(self.cv2_to_pixmap(contours).scaled(400, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        
        if sign_image is not None:
            self.signLabel.setPixmap(self.cv2_to_pixmap(sign_image).scaled(400, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            self.signLabel.setText("No sign detected\nor invalid contour")
        
        if gray_mask is not None:
            self.grayMaskLabel.setPixmap(self.cv2_to_pixmap(gray_mask).scaled(400, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            self.grayMaskLabel.setText("No gray plate found")
        
        if plate_image is not None:
            self.plateLabel.setPixmap(self.cv2_to_pixmap(plate_image).scaled(400, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            self.plateLabel.setText("No plate extracted")
        
        # Update status
        status_text = f"Status: {'SIGN DETECTED' if sign_detected else 'Searching...'}"
        self.statusLabel.setText(status_text)
        self.infoLabel.setText(status_info)
        
        # Color code status
        if sign_detected:
            self.statusLabel.setStyleSheet("QLabel { padding: 10px; border: 2px solid green; background-color: #90EE90; }")
        else:
            self.statusLabel.setStyleSheet("QLabel { padding: 10px; border: 1px solid gray; background-color: white; }")
    
    def cv2_to_pixmap(self, cv_image):
        h, w = cv_image.shape[:2]
        if len(cv_image.shape) == 2:  # Grayscale
            bytes_per_line = w
            qt_format = QImage.Format_Grayscale8
        else:  # Color
            bytes_per_line = w * 3
            qt_format = QImage.Format_RGB888
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        q_img = QImage(cv_image.data, w, h, bytes_per_line, qt_format)
        return QPixmap.fromImage(q_img)
    
    def save_parameters(self):
        # Save current parameters to file
        params = {
            'blue_params': self.blue_params,
            'gray_params': self.gray_params,
            'MIN_SIGN_AREA': self.MIN_SIGN_AREA,
            'contour_epsilon': self.contour_epsilon
        }
        rospy.loginfo("Parameters saved (implement file saving)")
        rospy.loginfo(f"Blue params: {self.blue_params}")
        rospy.loginfo(f"Gray params: {self.gray_params}")
    
    def reset_parameters(self):
        # Reset to default values
        self.blue_params = {'H_min': 100, 'H_max': 140, 'S_min': 150, 'S_max': 255, 'V_min': 50, 'V_max': 255}
        self.gray_params = {'H_min': 0, 'H_max': 180, 'S_min': 0, 'S_max': 50, 'V_min': 50, 'V_max': 200}
        self.MIN_SIGN_AREA = 5000
        self.contour_epsilon = 0.02
        
        # Update UI elements
        self.areaSpin.setValue(self.MIN_SIGN_AREA)
        self.epsilonSpin.setValue(self.contour_epsilon)
        
        # Update sliders
        self.blue_hue_min_slider.setValue(self.blue_params['H_min'])
        self.blue_hue_max_slider.setValue(self.blue_params['H_max'])
        self.blue_sat_min_slider.setValue(self.blue_params['S_min'])
        self.blue_sat_max_slider.setValue(self.blue_params['S_max'])
        self.blue_val_min_slider.setValue(self.blue_params['V_min'])
        self.blue_val_max_slider.setValue(self.blue_params['V_max'])
        
        self.gray_hue_min_slider.setValue(self.gray_params['H_min'])
        self.gray_hue_max_slider.setValue(self.gray_params['H_max'])
        self.gray_sat_min_slider.setValue(self.gray_params['S_min'])
        self.gray_sat_max_slider.setValue(self.gray_params['S_max'])
        self.gray_val_min_slider.setValue(self.gray_params['V_min'])
        self.gray_val_max_slider.setValue(self.gray_params['V_max'])
        
        rospy.loginfo("Parameters reset to defaults")
    
    def closeEvent(self, event):
        self.timer.stop()
        rospy.signal_shutdown("GUI closed")
        event.accept()

def main():
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    gui = SignDetector()
    gui.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()