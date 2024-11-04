from ultralytics import YOLO
import cv2 as cv
import sys
sys.path.append("/home/loki/B3RB_ENIAC")

model = YOLO('yolov5s.yaml').load('yolov5s.pt')
# model = YOLO('yolov5n.yaml').load('/home/loki/B3RB_ENIAC/runs/detect/train7/weights/last.pt')
results = model.train(data="config/config.yaml", batch = 128, fliplr=0.0, epochs=50, lr0=0.000001, freeze = 6, 
                       conf = 0.40, optimizer = "AdamW", dropout = 0.5, plots = True, imgsz = 320, cos_lr = True, cls = 1.5)
