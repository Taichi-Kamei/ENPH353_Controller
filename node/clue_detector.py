import cv2
import numpy as np

from rapidfuzz import process, fuzz

from ocr import OCR

class ClueDetector:
    def __init__(self, model_path='conv_model_0.99.tflite'):
        self.latest_board_mask = None
        self.latest_board = None
        self.latest_plate = None
        self.latest_letter_mask = None

        self.latest_id = 0
        self.latest_key = None
        self.latest_value = None

        self.ocr_model = OCR(model_path)

        self.ids = {
            'SIZE': 1,
            'VICTIM': 2,
            'CRIME': 3,
            'TIME': 4,
            'PLACE': 5,
            'MOTIVE': 6,
            'WEAPON': 7,
            'BANDIT': 8
        }
        
        self.params = {
            'contour_epsilon': 0.02,

            'min_board_area' : 12000,
            'min_board_color': np.array([80, 125, 0]),
            'max_board_color': np.array([160, 255, 255]),
            'board_width': 600,
            'board_height': 400,

            'min_plate_color': np.array([0, 0, 25]),
            'max_plate_color': np.array([180, 65, 230]),
            'plate_width': 300,
            'plate_height': 200,

            'row_threshold': 30,

            'min_letter_area': 50,
            'min_letter_color': np.array([100, 100, 0]),
            'max_letter_color': np.array([125, 255, 255]),
            'min_letter_ratio': 0.3,
            'max_letter_ratio': 1.5,
            'cnn_width': 100,
            'cnn_height': 150,
        }
    
    def get_board_mask(self):
        return self.latest_board_mask
    
    def get_board(self):
        return self.latest_board
    
    def get_plate(self):
        return self.latest_plate

    def get_letter_mask(self):
        return self.latest_letter_mask
    
    def get_clue(self):
        return self.latest_id, self.latest_value

    def detect_clue(self, raw_image):
        # Blue mask with contours
        self.latest_board_mask, blue_contours = self.create_blue_mask(raw_image)
        
        # Board extraction
        self.latest_board = self.extract_board(raw_image, blue_contours)
        
        if self.latest_board is not None:
            self.latest_plate = self.extract_plate(self.latest_board)
            
            if self.latest_plate is not None:
                self.latest_letter_mask, self.latest_key, self.latest_value = self.extract_letters(self.latest_plate)
                best_match, score, index = process.extractOne(self.latest_key, self.ids.keys())

                
                self.latest_id = self.ids.get(best_match, self.latest_id + 1)
                
                return self.latest_id, self.latest_value

        return None, None

    def create_blue_mask(self, image):
        """Create blue mask and find contours"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create blue mask
        blue_mask = cv2.inRange(hsv, self.params['min_board_color'], self.params['max_board_color'])
        
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
    
    def extract_board(self, image, contours):
        """Extract sign using perspective transformation"""
        if not contours:
            return None
            
        # Find largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < self.params['min_board_area']:
            return None
            
        # Apply perspective transformation
        return self.apply_perspective_transform(image, largest_contour, self.params['contour_epsilon'], (self.params['board_width'], self.params['board_height']))
    
    def extract_plate(self, board):
        """Extract plate from sign"""
        hsv_board = cv2.cvtColor(board, cv2.COLOR_BGR2HSV)
        
        # Create gray mask
        gray_mask = cv2.inRange(hsv_board, self.params['min_plate_color'], self.params['max_plate_color'])
        
        # Clean mask
        kernel = np.ones((5,5), np.uint8)
        gray_mask_clean = cv2.morphologyEx(gray_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(gray_mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
            
        largest_contour = max(contours, key=cv2.contourArea)
        return self.apply_perspective_transform(board, largest_contour, self.params['contour_epsilon'], (self.params['plate_width'], self.params['plate_height']))
    
    def extract_letters(self, plate_image):
        """Extract letters with row detection for multiple lines of text"""
        # plate_display = plate_image.copy()
        hsv_plate = cv2.cvtColor(plate_image, cv2.COLOR_BGR2HSV)
        
        # Get letter color range
        letter_mask = cv2.inRange(hsv_plate, self.params['min_letter_color'], self.params['max_letter_color'])
        
        # Clean mask
        kernel = np.ones((2,2), np.uint8)
        # letter_mask_clean = cv2.morphologyEx(letter_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        letter_mask_clean = cv2.morphologyEx(letter_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        letters_mask = cv2.cvtColor(letter_mask_clean, cv2.COLOR_GRAY2BGR)
        letters_display = cv2.cvtColor(letter_mask_clean, cv2.COLOR_GRAY2BGR)

        # Find all contours
        letter_contours, _ = cv2.findContours(letter_mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Group contours into rows based on vertical position
        rows = self.group_contours_into_rows(letter_contours)

        key_letters = self.process_single_row(rows[0], letters_mask, letters_display, (255, 0, 0))
        key_word = self.ocr_model.predict_batch(key_letters)

        value_letters = self.process_single_row(rows[1], letters_mask, letters_display, (0, 0, 255))
        value_word = self.ocr_model.predict_batch(value_letters)
        
        return letters_display, key_word, value_word
    
    def group_contours_into_rows(self, contours):
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
        
        for contour, y in contour_ys:
            if not current_row:
                current_row.append(contour)
            else:
                # Calculate average Y of current row
                current_row_ys = [cv2.boundingRect(c)[1] for c in current_row]
                current_avg_y = sum(current_row_ys) / len(current_row_ys)
                
                if abs(y - current_avg_y) < self.params['row_threshold']:
                    current_row.append(contour)
                else:
                    # New row
                    rows.append(current_row)
                    current_row = [contour]
        
        if current_row:
            rows.append(current_row)
        
        return rows

    def process_single_row(self, row_contours, letters_mask, letters_display, row_color):
        """
        Process a single row of text, handling merged letters within the row
        """
        row_letters = []
        
        # Calculate typical letter width for this row
        single_letter_widths = []
        for contour in row_contours:
            area = cv2.contourArea(contour)
            if area > self.params['min_letter_area']:
                x, y, w, h = cv2.boundingRect(contour)
                if self.params['min_letter_ratio'] <= (w / h) <= self.params['max_letter_ratio']:
                    single_letter_widths.append(w)
        
        if single_letter_widths:
            typical_width = np.median(single_letter_widths)
        else:
            typical_width = letters_mask.shape[1] * 0.08

        # Create list of (contour, x_position) pairs
        contour_xs = []
        for contour in row_contours:
            x, y, w, h = cv2.boundingRect(contour)
            contour_xs.append((contour, x))
        
        # Sort by y position
        contour_xs.sort(key=lambda item: item[1])
        
        # Process each contour in this row
        for contour, x in contour_xs:
            area = cv2.contourArea(contour)
            if area > self.params['min_letter_area']:
                x, y, w, h = cv2.boundingRect(contour)
                
                # Check if this is likely multiple letters in the same row
                if w > typical_width * 1.8:
                    num_letters = max(2, int(round(w / typical_width)))
                    split_boxes = self.split_into_monospace_boxes(x, y, w, h, num_letters)
                    
                    for bbox in split_boxes:
                        sx, sy, sw, sh = bbox
                        # Draw with row-specific color
                        cv2.rectangle(letters_display, (sx, sy), (sx+sw, sy+sh), row_color, 2)
                        
                        letter_img = letters_mask[sy:sy+sh, sx:sx+sw]
                        
                        row_letters.append(letter_img)
                else:
                    # Single letter in this row
                    cv2.rectangle(letters_display, (x, y), (x+w, y+h), row_color, 2)
                    
                    letter_img = letters_mask[y:y+h, x:x+w]
                    
                    row_letters.append(letter_img)
        
        return row_letters
    
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
    
    def apply_perspective_transform(self, image, contour, epsilon, output_size):
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