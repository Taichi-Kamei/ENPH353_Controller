#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageProcessor:
    def __init__(self, parameter_manager):
        self.parameter_manager = parameter_manager
        self.bridge = CvBridge()
        self.current_image = None
        self.camera_topic = None
        self.image_sub = None
        self.last_contours = []
        
    def setup_camera(self):
        """Setup ROS camera subscriber"""
        topics = rospy.get_published_topics()
        image_topics = [topic[0] for topic in topics if 'image' in topic[0] and 'compressed' not in topic[0]]
        
        # Try common camera topics
        common_topics = [
            '/camera/rgb/image_raw',
            '/camera/image_raw', 
            '/usb_cam/image_raw',
            '/cv_camera/image_raw',
            '/image_raw',
        ]
        
        for topic in common_topics:
            if topic in image_topics:
                self.camera_topic = topic
                break
        else:
            if image_topics:
                self.camera_topic = image_topics[0]
            else:
                self.camera_topic = '/camera/rgb/image_raw'
                
        rospy.loginfo(f"Using camera topic: {self.camera_topic}")
        
        try:
            self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        except Exception as e:
            rospy.logerr(f"Failed to subscribe to camera: {e}")
            
    def image_callback(self, msg):
        """ROS image callback"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
            
    def has_image(self):
        """Check if we have a current image"""
        return self.current_image is not None
        
    def process_current_image(self):
        """Process the current image and return results"""
        if self.current_image is None:
            return None
            
        return self.process_image(self.current_image.copy())
        
    def process_image(self, image):
        """Main image processing pipeline"""
        # Get current parameters
        params = self.parameter_manager.get_all_parameters()
        blue_params = params['colors']['blue']
        gray_params = params['colors']['gray']
        letter_params = params['letters']
        processing_params = params['processing']
        
        # Step 1: Original image
        original = image.copy()
        
        # Step 2: Blue mask with contours
        blue_mask_display, blue_contours = self.create_blue_mask(original, blue_params)
        self.last_contours = blue_contours
        
        # Step 3: Sign extraction
        sign_image = self.extract_sign(original, blue_contours, processing_params)
        
        # Step 4: Plate and letter extraction
        plate_image = None
        letter_boxes_display = None
        individual_letters = []
        
        if sign_image is not None:
            plate_image = self.extract_plate(sign_image, gray_params, processing_params)
            
            if plate_image is not None:
                letter_boxes_display, individual_letters = self.extract_letters(
                    plate_image, letter_params)
        
        return (original, blue_mask_display, sign_image, plate_image, 
                letter_boxes_display, individual_letters)
                
    def create_blue_mask(self, image, blue_params):
        """Create blue mask and find contours"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create blue mask
        blue_lower = np.array([blue_params['H_min'], blue_params['S_min'], blue_params['V_min']])
        blue_upper = np.array([blue_params['H_max'], blue_params['S_max'], blue_params['V_max']])
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
        
        # Clean mask
        kernel = np.ones((5,5), np.uint8)
        blue_mask_clean = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask_clean = cv2.morphologyEx(blue_mask_clean, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(blue_mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create display image with contours
        mask_display = cv2.cvtColor(blue_mask_clean, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(mask_display, contours, -1, (0, 255, 0), 2)
        
        return mask_display, contours
        
    def extract_sign(self, image, contours, processing_params):
        """Extract sign using perspective transformation"""
        if not contours:
            return None
            
        # Find largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < processing_params['MIN_SIGN_AREA']:
            return None
            
        # Apply perspective transformation
        return self.apply_perspective_transform(image, largest_contour, processing_params['contour_epsilon'])
        
    def extract_plate(self, sign_image, gray_params, processing_params):
        """Extract plate from sign"""
        hsv_sign = cv2.cvtColor(sign_image, cv2.COLOR_BGR2HSV)
        
        # Create gray mask
        gray_lower = np.array([gray_params['H_min'], gray_params['S_min'], gray_params['V_min']])
        gray_upper = np.array([gray_params['H_max'], gray_params['S_max'], gray_params['V_max']])
        gray_mask = cv2.inRange(hsv_sign, gray_lower, gray_upper)
        
        # Clean mask
        kernel = np.ones((5,5), np.uint8)
        gray_mask_clean = cv2.morphologyEx(gray_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(gray_mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
            
        largest_contour = max(contours, key=cv2.contourArea)
        return self.apply_perspective_transform(sign_image, largest_contour, processing_params['contour_epsilon'], (300, 200))
        
    def extract_letters(self, plate_image, letter_params):
        """Extract letters with row detection for multiple lines of text"""
        # plate_display = plate_image.copy()
        hsv_plate = cv2.cvtColor(plate_image, cv2.COLOR_BGR2HSV)
        
        # Get letter color range
        letter_color = self.get_letter_color_range(letter_params['letter_color_mode'])
        letter_mask = cv2.inRange(hsv_plate, letter_color['lower'], letter_color['upper'])
        
        # Clean mask
        kernel = np.ones((2,2), np.uint8)
        # letter_mask_clean = cv2.morphologyEx(letter_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        letter_mask_clean = cv2.morphologyEx(letter_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        plate_image = cv2.cvtColor(letter_mask_clean, cv2.COLOR_GRAY2BGR)
        plate_display = cv2.cvtColor(letter_mask_clean, cv2.COLOR_GRAY2BGR)

        # Find all contours
        letter_contours, _ = cv2.findContours(letter_mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Group contours into rows based on vertical position
        rows = self.group_contours_into_rows(letter_contours, plate_image.shape[0])
        
        # Process each row separately
        individual_letters = []
        row_colors = [(0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255)]  # Different colors for each row
        
        for row_idx, row_contours in enumerate(rows):
            row_color = row_colors[row_idx % len(row_colors)]
            row_letters = self.process_single_row(row_contours, plate_image, plate_display, 
                                                row_color, row_idx, letter_params)
            individual_letters.extend(row_letters)
        
        # Sort by row first, then by x position within row
        individual_letters.sort(key=lambda l: (l['row'], l['bbox'][0]))
        
        return plate_display, individual_letters

    def group_contours_into_rows(self, contours, image_height):
        """
        Group contours into rows based on their vertical positions
        Returns list of rows, each row is a list of contours
        """
        if not contours:
            return []

        # Create list of (contour, y_position) pairs
        contour_ys = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            contour_ys.append((contour, y))
        
        # Sort by y position
        contour_ys.sort(key=lambda item: item[1])
        
        rows = []
        current_row = []
        row_threshold = 30  # Fixed pixel threshold - adjust this!
        
        for contour, y in contour_ys:
            if not current_row:
                current_row.append(contour)
            else:
                # Calculate average Y of current row
                current_row_ys = [cv2.boundingRect(c)[1] for c in current_row]
                current_avg_y = sum(current_row_ys) / len(current_row_ys)
                
                if abs(y - current_avg_y) < row_threshold:
                    current_row.append(contour)
                else:
                    # New row
                    rows.append(current_row)
                    current_row = [contour]
        
        if current_row:
            rows.append(current_row)
        
        # Debug: Print row info
        rospy.loginfo(f"Detected {len(rows)} rows:")
        for i, row in enumerate(rows):
            row_ys = [cv2.boundingRect(c)[1] for c in row]
            rospy.loginfo(f"  Row {i}: {len(row)} letters, Y positions: {min(row_ys)}-{max(row_ys)}")
        
        return rows

    def process_single_row(self, row_contours, plate_image, plate_display, 
                        row_color, row_idx, letter_params):
        """
        Process a single row of text, handling merged letters within the row
        """
        row_letters = []
        
        # Calculate typical letter width for this row
        single_letter_widths = []
        for contour in row_contours:
            area = cv2.contourArea(contour)
            if area > letter_params['min_letter_area']:
                x, y, w, h = cv2.boundingRect(contour)
                if 0.3 <= (w / h) <= 1.5:  # Reasonable aspect ratio for single letters
                    single_letter_widths.append(w)
        
        if single_letter_widths:
            typical_width = np.median(single_letter_widths)
        else:
            typical_width = plate_image.shape[1] * 0.08
        
        rospy.loginfo(f"Row {row_idx}: typical letter width {typical_width:.1f}px")
        
        # Process each contour in this row
        for contour in row_contours:
            area = cv2.contourArea(contour)
            if area > letter_params['min_letter_area']:
                x, y, w, h = cv2.boundingRect(contour)
                
                # Check if this is likely multiple letters in the same row
                if w > typical_width * 1.8:
                    num_letters = max(2, int(round(w / typical_width)))
                    split_boxes = self.split_into_monospace_boxes(x, y, w, h, num_letters)
                    
                    rospy.loginfo(f"Row {row_idx}: splitting {w}px into {num_letters} letters")
                    
                    for bbox in split_boxes:
                        sx, sy, sw, sh = bbox
                        # Draw with row-specific color
                        cv2.rectangle(plate_display, (sx, sy), (sx+sw, sy+sh), row_color, 2)
                        
                        # Add row label
                        cv2.putText(plate_display, f"R{row_idx}", (sx, sy-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, row_color, 1)
                        
                        letter_img = plate_image[sy:sy+sh, sx:sx+sw]
                        cnn_ready = self.prepare_for_cnn(letter_img)
                        display_img = self.create_letter_display_image(cnn_ready)
                        
                        row_letters.append({
                            'bbox': bbox,
                            'original': letter_img,
                            'cnn_ready': cnn_ready,
                            'display_image': display_img,
                            'split': True,
                            'row': row_idx,
                            'word': self.assign_word_within_row(bbox, row_letters, typical_width)
                        })
                else:
                    # Single letter in this row
                    cv2.rectangle(plate_display, (x, y), (x+w, y+h), row_color, 2)
                    cv2.putText(plate_display, f"R{row_idx}", (x, y-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, row_color, 1)
                    
                    letter_img = plate_image[y:y+h, x:x+w]
                    cnn_ready = self.prepare_for_cnn(letter_img)
                    display_img = self.create_letter_display_image(cnn_ready)
                    
                    row_letters.append({
                        'bbox': (x, y, w, h),
                        'original': letter_img,
                        'cnn_ready': cnn_ready,
                        'display_image': display_img,
                        'split': False,
                        'row': row_idx,
                        'word': self.assign_word_within_row((x, y, w, h), row_letters, typical_width)
                    })
        
        # Sort this row's letters left to right
        row_letters.sort(key=lambda l: l['bbox'][0])
        
        return row_letters

    def assign_word_within_row(self, current_bbox, existing_letters, typical_width):
        """
        Assign word index within a row based on horizontal spacing
        """
        if not existing_letters:
            return 0  # First word
        
        current_x = current_bbox[0]
        
        # Find the closest letter to the left
        left_letters = [l for l in existing_letters if l['bbox'][0] < current_x]
        if not left_letters:
            return 0  # First word in row
        
        # Get the rightmost letter to the left
        closest_left = max(left_letters, key=lambda l: l['bbox'][0])
        closest_x = closest_left['bbox'][0] + closest_left['bbox'][2]  # x + width
        
        # If gap is large, it's probably a new word
        word_spacing_threshold = typical_width * 1.5
        if current_x - closest_x > word_spacing_threshold:
            return closest_left.get('word', 0) + 1
        else:
            return closest_left.get('word', 0)

    def split_into_monospace_boxes(self, x, y, w, h, num_letters):
        """
        Split a wide bounding box into equal-width boxes for monospace letters
        """
        split_boxes = []
        letter_width = w // num_letters
        
        for i in range(num_letters):
            letter_x = x + (i * letter_width)
            # Ensure we don't go beyond original bounds
            letter_w = letter_width if i < num_letters - 1 else w - (i * letter_width)
            
            split_boxes.append((letter_x, y, letter_w, h))
        
        return split_boxes
        
    def get_letter_color_range(self, color_mode):
        """Get HSV range for letter color"""
        ranges = {
            'blue': {'lower': np.array([80, 100, 0]), 'upper': np.array([150, 255, 255])},
            'black': {'lower': np.array([0, 0, 0]), 'upper': np.array([180, 255, 50])},
            'white': {'lower': np.array([0, 0, 200]), 'upper': np.array([180, 50, 255])}
        }
        return ranges.get(color_mode, ranges['blue'])
        
    def prepare_for_cnn(self, letter_img, cnn_width=100, cnn_height=150):
        """Prepare letter image for CNN input"""
        if len(letter_img.shape) == 3:
            gray = cv2.cvtColor(letter_img, cv2.COLOR_BGR2GRAY)
        else:
            gray = letter_img
        
        # Always resize
        resized = cv2.resize(gray, (cnn_width, cnn_height))
        normalized = resized.astype(np.float32) / 255.0
        
        return normalized
        
    def create_letter_display_image(self, cnn_img, cnn_width=100, cnn_height=150):
        """Create display image for individual letter"""
        display_img = (cnn_img * 255).astype(np.uint8)
        display_img = cv2.cvtColor(display_img, cv2.COLOR_GRAY2BGR)
        
        # Add border and label
        # cv2.rectangle(display_img, (0, 0), (cnn_width-1, cnn_height-1), (0, 255, 0), 1)
        
        return display_img
        
    def apply_perspective_transform(self, image, contour, epsilon, output_size=(300, 400)):
        """Apply perspective transformation to extract ROI"""
        # Simplify contour
        approx = cv2.approxPolyDP(contour, epsilon * cv2.arcLength(contour, True), True)
        
        if len(approx) != 4:
            return None
            
        # Order points and apply homography
        points = self.order_points(approx.reshape(4, 2))
        width, height = output_size
        
        dst_points = np.array([[0, 0], [width-1, 0], [width-1, height-1], [0, height-1]], dtype='float32')
        
        H = cv2.findHomography(points, dst_points)[0]
        warped = cv2.warpPerspective(image, H, (width, height))
        
        return warped
        
    def order_points(self, pts):
        """Order points as top-left, top-right, bottom-right, bottom-left"""
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        
        return rect
        
    def cleanup(self):
        """Cleanup resources"""
        if self.image_sub:
            self.image_sub.unregister()