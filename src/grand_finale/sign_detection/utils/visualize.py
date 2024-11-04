import cv2 as cv
import os
import os.path as osp
import numpy as np
import sys
sys.path.append("/home/loki/B3RB_ENIAC")

from config.config import *

def visualize(img_path: str, label_path: str, color: tuple[int] = (0, 255, 0), show_labels = True, save = False, save_dir = None) -> np.ndarray:
    img_extension = "."+img_path.split('.')[-1]
    img = cv.imread(img_path)
    with open(label_path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            cls, x, y, w, h = line.split()
            x, y, w, h = float(x), float(y), float(w), float(h)
            x, y, w, h = x*IMG_W, y*IMG_H, w*IMG_W, h*IMG_H
            img = cv.rectangle(img, (int(x-w/2), int(y-h/2)), (int(x+w/2), int(y+h/2)), color, 2)
            if show_labels:
                offset = 5
                img = cv.putText(img, labels[cls], (int(x-w/2- offset), int(y-h/2 - offset)), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            if save and save_dir is None:
                cv.imwrite(img_path.replace(img_extension, f'_visualized{img_extension}'), img)
            else:
                assert osp.isdir(save_dir) 
                cv.imwrite(osp.join(save_dir, img_path.split('/')[-1].replace(img_extension, f'_visualized{img_extension}')), img)
    return img

if __name__ == "__main__":
    img_path = '/home/loki/B3RB_ENIAC/data/train_val/dataset/images/turn_right_46_jpg.rf.d3589c4eaeb52e3ea2dbc506b58452c0.jpg'
    label_path = '/home/loki/B3RB_ENIAC/data/train_val/dataset/labels/turn_right_46_jpg.rf.d3589c4eaeb52e3ea2dbc506b58452c0.txt'
    img = visualize(img_path, label_path)
    cv.imshow('image', img)
    cv.waitKey(0)