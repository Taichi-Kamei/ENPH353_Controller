#!/usr/bin/env python3
import string
import numpy as np
import tensorflow as tf
import cv2
import os

class OCR:
    def __init__(self, model_path=None):
        self.interpreter = None
        self.input_details = None
        self.output_details = None
        self.class_names = (string.digits + string.ascii_uppercase)
        self.model_path = model_path
        
        if model_path and os.path.exists(model_path):
            self.load_model(model_path)
    
    def load_model(self, model_path):
        """Load the TFLite model"""
        try:
            self.interpreter = tf.lite.Interpreter(model_path=model_path)
            self.interpreter.allocate_tensors()
            
            # Get input/output details
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            
            print(f"Model loaded: {model_path}")
            print(f"Input shape: {self.input_details[0]['shape']}")
            print(f"Output shape: {self.output_details[0]['shape']}")
            
        except Exception as e:
            print(f"Error loading model: {e}")
            self.interpreter = None
    
    def preprocess_letter(self, letter_image, target_size=(32, 32)):
        """Preprocess letter image for OCR model"""
        if letter_image is None:
            return None
        
        # Convert to grayscale if needed
        if len(letter_image.shape) == 3:
            gray = cv2.cvtColor(letter_image, cv2.COLOR_BGR2GRAY)
        else:
            gray = letter_image
        
        # Resize to target size
        resized = cv2.resize(gray, target_size, interpolation=cv2.INTER_AREA)
        
        # Normalize to [0, 1]
        normalized = resized.astype(np.float32) / 255.0
        
        # Add channel dimension if needed
        if self.input_details[0]['shape'][-1] == 1:
            normalized = np.expand_dims(normalized, axis=-1)
        
        # Add batch dimension
        input_data = np.expand_dims(normalized, axis=0)
        
        return input_data
    
    def predict(self, letter_image):
        """Predict character from letter image"""
        if self.interpreter is None:
            return None, 0.0
        
        try:
            # Allocate tensors (necessary to prepare the interpreter for inference)
            self.interpreter.allocate_tensors()

            # Get input and output tensor details
            input_details = self.interpreter.get_input_details()
            output_details = self.interpreter.get_output_details()

            # Set the input tensor
            # Ensure the input has the correct dtype (float32) and shape (add batch dimension)
            input_data = letter_image.reshape(150, 100, 1).astype('float32')
            input_tensor = np.expand_dims(input_data, axis=0).astype(np.float32)
            self.interpreter.set_tensor(input_details[0]['index'], input_tensor)

            # Invoke the interpreter to run inference
            self.interpreter.invoke()

            # Get the output tensor
            output_data = self.interpreter.get_tensor(output_details[0]['index'])
            
            # Get prediction
            prediction = np.argmax(output_data[0])
            confidence = np.max(output_data[0])
            
            # Map to character
            if prediction < len(self.class_names):
                char = self.class_names[prediction]
            else:
                char = '?'
            
            return char, confidence
            
        except Exception as e:
            print(f"Prediction error: {e}")
            return None, 0.0
    
    def predict_batch(self, letter_images):
        """Predict multiple letters"""
        results = []
        for img in letter_images:
            char, confidence = self.predict(img)
            results.append((char, confidence))
        return results