#!/usr/bin/env python3
import rospy
import sys
import os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QMessageBox, 
                             QSlider, QLabel, QWidget, QFileDialog)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.uic import loadUi

import numpy as np
import cv2

from controller.node.deprecated.image_processor import ImageProcessor
from parameter_manager import ParameterManager

class SignDebugGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Initialize components
        self.parameter_manager = ParameterManager()
        self.image_processor = ImageProcessor(self.parameter_manager)
        
        # Load UI
        self.load_ui()
        
        # Setup UI components
        self.setup_ui_components()
        
        # Connect signals
        self.connect_signals()
        
        # Setup camera
        self.setup_camera()
        
        # Start update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(33)  # 30Hz
        
    def load_ui(self):
        """Load the UI file"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        ui_file = os.path.join(script_dir, 'sign_debug_gui.ui')
        
        if not os.path.exists(ui_file):
            rospy.logerr(f"UI file not found: {ui_file}")
            sys.exit(1)
            
        loadUi(ui_file, self)
        rospy.loginfo("UI loaded successfully")
        
    def setup_ui_components(self):
        """Setup UI controls with current parameter values"""
        # Set initial values from parameters
        params = self.parameter_manager.get_all_parameters()
        
        # Processing parameters
        self.areaSpin.setValue(params['processing']['MIN_SIGN_AREA'])
        self.epsilonSpin.setValue(params['processing']['contour_epsilon'])
        
        # Letter parameters
        self.minLetterAreaSpin.setValue(params['letters']['min_letter_area'])
        # self.cnnSizeSpin.setValue(params['letters']['cnn_input_size'])
        
        # Set letter color
        color_map = {'blue': 'Blue Letters', 'black': 'Black Letters', 'white': 'White Letters'}
        self.letterColorCombo.setCurrentText(color_map.get(params['letters']['letter_color_mode'], 'Blue Letters'))
        
        # Setup sliders for color parameters
        self.setup_color_sliders()
        
    def setup_color_sliders(self):
        """Setup color range sliders"""
        params = self.parameter_manager.get_all_parameters()
        
        # Blue mask sliders
        blue_params = params['colors']['blue']
        self.setup_slider_pair(self.blueHueLayout, 'blue', 'H', blue_params['H_min'], blue_params['H_max'])
        self.setup_slider_pair(self.blueSatLayout, 'blue', 'S', blue_params['S_min'], blue_params['S_max'])
        self.setup_slider_pair(self.blueValLayout, 'blue', 'V', blue_params['V_min'], blue_params['V_max'])
        
        # Gray mask sliders
        gray_params = params['colors']['gray']
        self.setup_slider_pair(self.grayHueLayout, 'gray', 'H', gray_params['H_min'], gray_params['H_max'])
        self.setup_slider_pair(self.graySatLayout, 'gray', 'S', gray_params['S_min'], gray_params['S_max'])
        self.setup_slider_pair(self.grayValLayout, 'gray', 'V', gray_params['V_min'], gray_params['V_max'])
        
    def setup_slider_pair(self, layout, color_type, channel, min_val, max_val):
        """Create a min-max slider pair for a color channel"""
        # Clear existing layout
        for i in reversed(range(layout.count())):
            layout.itemAt(i).widget().setParent(None)
            
        # Min slider
        min_slider = QSlider(Qt.Horizontal)
        min_slider.setRange(0, 255 if channel in ['S', 'V'] else 179)
        min_slider.setValue(min_val)
        min_slider.valueChanged.connect(lambda v: self.on_color_parameter_changed(color_type, f'{channel}_min', v))
        layout.addWidget(min_slider)
        
        # Max slider
        max_slider = QSlider(Qt.Horizontal)
        max_slider.setRange(0, 255 if channel in ['S', 'V'] else 179)
        max_slider.setValue(max_val)
        max_slider.valueChanged.connect(lambda v: self.on_color_parameter_changed(color_type, f'{channel}_max', v))
        layout.addWidget(max_slider)
        
        # Labels
        min_label = QLabel(str(min_val))
        min_label.setMinimumWidth(30)
        max_label = QLabel(str(max_val))
        max_label.setMinimumWidth(30)
        
        min_slider.valueChanged.connect(lambda v: min_label.setText(str(v)))
        max_slider.valueChanged.connect(lambda v: max_label.setText(str(v)))
        
        layout.addWidget(min_label)
        layout.addWidget(QLabel("-"))
        layout.addWidget(max_label)
        
    def connect_signals(self):
        """Connect UI signals to handlers"""
        # Parameter controls
        self.areaSpin.valueChanged.connect(lambda v: self.parameter_manager.set_parameter('processing', 'MIN_SIGN_AREA', v))
        self.epsilonSpin.valueChanged.connect(lambda v: self.parameter_manager.set_parameter('processing', 'contour_epsilon', v))
        
        # Letter controls
        self.minLetterAreaSpin.valueChanged.connect(lambda v: self.parameter_manager.set_parameter('letters', 'min_letter_area', v))
        # self.cnnSizeSpin.valueChanged.connect(lambda v: self.parameter_manager.set_parameter('letters', 'cnn_input_size', v))
        self.letterColorCombo.currentTextChanged.connect(self.on_letter_color_changed)
        
        # Buttons
        self.saveButton.clicked.connect(self.on_save_parameters)
        self.loadButton.clicked.connect(self.on_load_parameters)
        self.resetButton.clicked.connect(self.on_reset_parameters)
        self.saveLettersButton.clicked.connect(self.on_save_letters)
        
    def on_color_parameter_changed(self, color_type, parameter, value):
        """Handle color parameter changes from sliders"""
        self.parameter_manager.set_parameter('colors', color_type, {parameter: value})
        
    def on_letter_color_changed(self, text):
        """Handle letter color selection change"""
        color_map = {'Blue Letters': 'blue', 'Black Letters': 'black', 'White Letters': 'white'}
        self.parameter_manager.set_parameter('letters', 'letter_color_mode', color_map.get(text, 'blue'))
        
    def on_save_parameters(self):
        """Save current parameters to file"""
        try:
            self.parameter_manager.save_parameters()
            self.statusLabel.setText("Status: Parameters saved")
        except Exception as e:
            rospy.logerr(f"Error saving parameters: {e}")
            self.statusLabel.setText("Status: Save failed")
            
    def on_load_parameters(self):
        """Load parameters from file"""
        try:
            self.parameter_manager.load_parameters()
            self.setup_ui_components()  # Refresh UI with loaded values
            self.statusLabel.setText("Status: Parameters loaded")
        except Exception as e:
            rospy.logerr(f"Error loading parameters: {e}")
            self.statusLabel.setText("Status: Load failed")
            
    def on_reset_parameters(self):
        """Reset parameters to defaults"""
        self.parameter_manager.reset_to_defaults()
        self.setup_ui_components()  # Refresh UI with default values
        self.statusLabel.setText("Status: Parameters reset")
        
    def on_save_letters(self):
        """Save letters with plate reference"""
        try:
            results = self.image_processor.process_current_image()
            if not results or not results[5]:
                QMessageBox.warning(self, "No Letters", "No letters detected.")
                return
                
            individual_letters = results[5]
            plate_image = results[3]  # The plate with bounding boxes
            
            save_dir = QFileDialog.getExistingDirectory(self, "Save Location", os.path.expanduser("~/Desktop"))
            if not save_dir:
                return
            
            # Save plate reference
            if plate_image is not None:
                cv2.imwrite(os.path.join(save_dir, "plate_with_boxes.jpg"), plate_image)
            
            # Save all letters
            for i, letter_data in enumerate(individual_letters):
                if 'original' in letter_data:
                    cv2.imwrite(os.path.join(save_dir, f"letter_{i:02d}.jpg"), letter_data['original'])
                if 'cnn_ready' in letter_data:
                    img = letter_data['cnn_ready']
                    img_uint8 = (img * 255).astype(np.uint8)
                    cv2.imwrite(f"{save_dir}/letter_{i:02d}_cnn.png", img_uint8)
            
            # Show message
            QMessageBox.information(self, "Saved", 
                                f"Saved {len(individual_letters)} letters + plate reference to:\n{save_dir}")
            self.statusLabel.setText(f"Saved {len(individual_letters)} letters")
            
        except Exception as e:
            rospy.logerr(f"Save error: {e}")
            QMessageBox.critical(self, "Error", f"Save failed: {str(e)}")
        
    def setup_camera(self):
        """Setup ROS camera subscriber"""
        self.image_processor.setup_camera()
        
    def update_display(self):
        """Main update loop - process image and update displays"""
        if not self.image_processor.has_image():
            self.show_no_camera_display()
            return
            
        # Process the current image
        results = self.image_processor.process_current_image()
        
        if results:
            self.update_image_displays(results)
            self.update_status_info(results)
            
    def show_no_camera_display(self):
        """Show placeholder when no camera feed is available"""
        placeholder = self.create_placeholder_image("NO CAMERA FEED")
        
        # Update all image displays
        displays = [
            self.originalLabel, self.blueMaskLabel, self.signLabel,
            self.plateLabel, self.letterLabel
        ]
        
        for display in displays:
            if hasattr(display, 'setPixmap'):
                display.setPixmap(placeholder)
            else:
                display.setText("No camera")
                
        self.clear_letters_display()
        self.statusLabel.setText("Status: Waiting for camera")
        
    def create_placeholder_image(self, text):
        """Create a placeholder image with text"""
        
        img = np.zeros((160, 280, 3), dtype=np.uint8)
        cv2.putText(img, text, (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        return self.cv2_to_pixmap(img)
        
    def update_image_displays(self, results):
        """Update all image displays with processed results"""
        original, blue_mask, sign_image, plate_image, letter_boxes, individual_letters = results
        
        # Update main displays
        self.update_single_display(self.originalLabel, original)
        self.update_single_display(self.blueMaskLabel, blue_mask)
        self.update_single_display(self.signLabel, sign_image, "No sign")
        self.update_single_display(self.plateLabel, plate_image, "No plate")
        self.update_single_display(self.letterLabel, letter_boxes, "No letters")
        
        # Update individual letters
        self.update_individual_letters(individual_letters)
        
    def update_single_display(self, label, image, placeholder_text=None):
        """Update a single image display with proper scaling"""
        if image is not None:
            # Get the label's current size
            label_width = label.width()
            label_height = label.height()
            
            # Scale image to fit label while maintaining aspect ratio
            pixmap = self.cv2_to_pixmap(image)
            scaled_pixmap = pixmap.scaled(label_width, label_height, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            label.setPixmap(scaled_pixmap)
        elif placeholder_text:
            label.setText(placeholder_text)

    def update_individual_letters(self, individual_letters):
        """Simple grid layout for individual letters"""
        self.clear_letters_display()
        
        # Sort by row, then by position
        individual_letters.sort(key=lambda l: (l.get('row', 0), l.get('bbox', (0,))[0]))
        
        current_row = -1
        
        for letter_data in individual_letters:
            row = letter_data.get('row', 0)
            
            # Add row separator if new row
            if row != current_row:
                current_row = row
                row_label = QLabel(f"--- Row {row} ---")
                row_label.setStyleSheet("QLabel { color: red; font-weight: bold; margin: 10px; }")
                self.lettersLayout.addWidget(row_label)
            
            # Get the display image
            if 'display_image' in letter_data:
                display_img = letter_data['display_image']
            else:
                display_img = letter_data['original']
            
            # Convert and scale
            pixmap = self.cv2_to_pixmap(display_img)
            scaled_pixmap = pixmap.scaled(60, 60, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            
            # Create label
            letter_label = QLabel()
            letter_label.setPixmap(scaled_pixmap)
            
            # Simple styling
            border_color = "red" if letter_data.get('split', False) else "blue"
            letter_label.setStyleSheet(f"QLabel {{ border: 1px solid {border_color}; margin: 1px; }}")
            
            # Tooltip
            word = letter_data.get('word', 0)
            letter_label.setToolTip(f"Row {row}, Word {word}")
            
            self.lettersLayout.addWidget(letter_label)

    def clear_letters_display(self):
        """Clear the individual letters display"""
        for i in reversed(range(self.lettersLayout.count())):
            widget = self.lettersLayout.itemAt(i).widget()
            if widget:
                widget.deleteLater()
                
    def update_status_info(self, results):
        """Update status information"""
        original, blue_mask, sign_image, plate_image, letter_boxes, individual_letters = results
        
        sign_detected = sign_image is not None
        status_text = "SIGN DETECTED" if sign_detected else "Searching..."
        self.statusLabel.setText(f"Status: {status_text}")
        
        # Update info labels
        contours_info = f"Contours: {len(self.image_processor.last_contours) if hasattr(self.image_processor, 'last_contours') else 0}"
        self.infoLabel.setText(contours_info)
        self.letterInfoLabel.setText(f"Letters: {len(individual_letters)}")
        
        # Color status
        if sign_detected:
            self.statusLabel.setStyleSheet("QLabel { border: 2px solid green; background-color: #90EE90; }")
        else:
            self.statusLabel.setStyleSheet("QLabel { border: 1px solid gray; background-color: white; }")
            
    def cv2_to_pixmap(self, cv_image):
        """Convert OpenCV image to QPixmap"""
        
        if len(cv_image.shape) == 2:  # Grayscale
            h, w = cv_image.shape
            bytes_per_line = w
            q_img = QImage(cv_image.data, w, h, bytes_per_line, QImage.Format_Grayscale8)
        else:  # Color
            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            q_img = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
        return QPixmap.fromImage(q_img)
        
    def closeEvent(self, event):
        """Cleanup on window close"""
        self.timer.stop()
        self.image_processor.cleanup()
        rospy.signal_shutdown("GUI closed")
        event.accept()

def main():
    import sys
    from PyQt5.QtWidgets import QApplication
    
    rospy.init_node('sign_detection_gui', anonymous=True)
    
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    gui = SignDebugGUI()
    gui.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()