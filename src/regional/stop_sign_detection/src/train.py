from ultralytics import YOLO
import cv2 as cv

model = YOLO('yolov8n.yaml').load('yolov8n.pt')
results = model.train(data="config/config.yaml", batch = 16, epochs=50, degrees = 15, shear=15, crop_fraction=0.9, lr0=0.00001, freeze = 10, iou = 0.50, conf = 0.5)
# results = model.train(data="config/config.yaml", epochs=50, degrees = 20, shear=20,crop_fraction=0.9, lr0=0.0001, )
results = model.val()