# from ultralytics import YOLO
# model = YOLO('/home/loki/B3RB_ENIAC/runs/detect/train11/weights/best.pt')
# results = model.export(format='tflite', int8 = True)
# print(results)

import tensorflow as tf
import cv2
import numpy as np
import os

# Load the SavedModel
model = tf.saved_model.load('/home/loki/B3RB_ENIAC/runs/detect/train11/weights/best_saved_model')

validation_data_path = '/home/loki/B3RB_ENIAC/data/combined/valid/images'

# Image input size (modify according to your model's input size, typical sizes for YOLO include 416x416 or 640x640)
IMG_SIZE = (320, 320)

# Function to preprocess the image (resizing, normalization, etc.)
def preprocess_image(image_path):
    # Read the image
    img = cv2.imread(image_path)
    
    # Resize the image to the required size
    img = cv2.resize(img, IMG_SIZE)
    
    # Normalize the image (optional, depending on your model's requirement)
    img = img / 255.0
    
    # Convert image to float32 type
    img = img.astype(np.float32)
    
    return img

def representative_data_gen():
    # List all image files in the validation directory (assuming YOLO format)
    image_paths = [os.path.join(validation_data_path, fname) for fname in os.listdir(validation_data_path) if fname.endswith('.jpg') or fname.endswith('.png')]

    for image_path in image_paths:
        # Preprocess the image
        image = preprocess_image(image_path)
        
        # Add a batch dimension and yield the image
        image = np.expand_dims(image, axis=0)  # Add batch dimension
        
        # Yield the image
        yield [image]

# Set up the converter with INT8 quantization
converter = tf.lite.TFLiteConverter.from_saved_model('/home/loki/B3RB_ENIAC/runs/detect/train11/weights/best_saved_model')
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_data_gen
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.uint8  # or tf.uint8
converter.inference_output_type = tf.uint8  # or tf.uint8

# Convert the model
tflite_model = converter.convert()

# Save the TFLite model
with open('model_uint8.tflite', 'wb') as f:
    f.write(tflite_model)