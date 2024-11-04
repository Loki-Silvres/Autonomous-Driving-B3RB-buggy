import os
import os.path as osp
import sys
import cv2 as cv

# sys.path.append("/home/loki/B3RB_ENIAC")
# from config.config import *

# /home/loki/DentalObjectDetection/data/YOLO/valid/images/0b2b1550-NAJIBMARDANLO_ZAHRA_2020-07-19121046_jpg.rf.c849cea333b5fe90749beb319b634951.jpg
ds_path = "/home/loki/DentalObjectDetection/data/YOLO/valid"

labels = sorted(os.listdir(osp.join(ds_path, "labels")))
images = sorted(os.listdir(osp.join(ds_path, "images")))

for label in labels:
    img = cv.imread(osp.join(ds_path, "images", images[labels.index(label)]))
    IMG_H, IMG_W = img.shape[:2]
    with open(osp.join(ds_path, "labels", label), 'r') as r:
        lines = r.readlines()
        for line in lines:
            line = line.split()
            cls, x, y, w, h = line
            cls = int(cls)
            x, y, w, h = float(x), float(y), float(w), float(h)
            x, y, w, h = x*IMG_W, y*IMG_H, w*IMG_W, h*IMG_H
            # print(cls, x, y, w, h)
            PALETTE = [(220, 20, 60), (119, 11, 32), (0, 0, 142), 
                      (0, 0, 230), (106, 0, 228), (0, 60, 100), 
                      (0, 80, 100), (0, 0, 70), (0, 0, 192), 
                      (250, 170, 30), (100, 170, 30), (220, 220, 0), 
                      (175, 116, 175), (250, 0, 30)]
            color = PALETTE[cls]
            # if cls == "1": 
            #     color = (255, 100, 0)
            # if cls == "2": 
            #     color = (0, 0, 255)
            # if cls == "3": 
            #     color = (0, 255, 0)
            img = cv.rectangle(img, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), color, 2)
            # offset = 5
            # img = cv.putText(img, labels[cls], (int(x-w/2- offset), int(y-h/2 - offset)), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    cv.imwrite(osp.join(ds_path, "visualized", images[labels.index(label)]), img)