#!/usr/bin/env python
import os
import yaml
import rospy

class ParameterManager:
    def __init__(self, config_file=None):
        if config_file is None:
            self.config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sign_detection_config.yaml')
        else:
            self.config_file = config_file
            
        self.parameters = self.load_parameters() or self.get_default_parameters()
        
    def get_default_parameters(self):
        """Get default parameter values - match the structure GUI expects"""
        return {
            'processing': {  # Changed from 'processing_params'
                'MIN_SIGN_AREA': 5000,
                'contour_epsilon': 0.02
            },
            'colors': {  # Changed from 'color_params'
                'blue': {'H_min': 100, 'H_max': 140, 'S_min': 150, 'S_max': 255, 'V_min': 50, 'V_max': 255},
                'gray': {'H_min': 0, 'H_max': 180, 'S_min': 0, 'S_max': 50, 'V_min': 50, 'V_max': 200}
            },
            'letters': {  # Changed from 'letter_params'
                'min_letter_area': 50,
                'cnn_input_size': 32,
                'letter_color_mode': 'blue'
            }
        }
        
    def get_all_parameters(self):
        """Get all current parameters"""
        return self.parameters.copy()
        
    def get_parameter(self, category, key):
        """Get a specific parameter"""
        if category in self.parameters and key in self.parameters[category]:
            return self.parameters[category][key]
        else:
            rospy.logwarn(f"Parameter not found: {category}.{key}")
            return None
        
    def set_parameter(self, category, key, value):
        """Set a parameter value"""
        if category not in self.parameters:
            rospy.logwarn(f"Category not found: {category}")
            return
            
        if isinstance(value, dict):
            # For nested updates (like color parameters)
            if key in self.parameters[category]:
                self.parameters[category][key].update(value)
            else:
                self.parameters[category][key] = value
        else:
            self.parameters[category][key] = value
            
    def load_parameters(self):
        """Load parameters from YAML file"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    loaded_params = yaml.safe_load(f)
                    if loaded_params:
                        # Ensure all expected categories exist
                        default_params = self.get_default_parameters()
                        for category, default_values in default_params.items():
                            if category not in loaded_params:
                                loaded_params[category] = default_values
                            else:
                                # Ensure all keys exist within each category
                                for key, default_value in default_values.items():
                                    if key not in loaded_params[category]:
                                        loaded_params[category][key] = default_value
                        
                        self.parameters = loaded_params
                        rospy.loginfo(f"Parameters loaded from {self.config_file}")
                        return loaded_params
            rospy.loginfo("No config file found or empty file, using defaults")
            return None
        except Exception as e:
            rospy.logwarn(f"Error loading parameters: {e}")
            return None
            
    def save_parameters(self):
        """Save parameters to YAML file"""
        try:
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(self.config_file), exist_ok=True)
            
            with open(self.config_file, 'w') as f:
                yaml.dump(self.parameters, f, default_flow_style=False)
            rospy.loginfo(f"Parameters saved to {self.config_file}")
            return True
        except Exception as e:
            rospy.logerr(f"Error saving parameters: {e}")
            return False
            
    def reset_to_defaults(self):
        """Reset all parameters to defaults"""
        self.parameters = self.get_default_parameters()
        rospy.loginfo("Parameters reset to defaults")