import os
import os.path as osp
import matplotlib.pyplot as plt
import numpy as np
import sys
import shutil
sys.path.append("/home/loki/B3RB_ENIAC")

from config.config import *

label_folder_path: str = "/home/loki/B3RB_ENIAC/data/train_val/dataset/labels"

classes = ["left", "right", "stop", "image"]

class_paths = {cls: [] for cls in classes}

ann_files: list[str] = os.listdir(label_folder_path)
for ann_file in ann_files:
    ann_file_path = osp.join(label_folder_path, ann_file)
    for cls in classes:
        if cls in ann_file:
            class_paths[cls].append(ann_file)

for cls in classes:
    class_paths[cls] = sorted(class_paths[cls])

print("All files sacnned: ", np.sum([len(i) for i in class_paths.values()]) == len(ann_files))

train_split = {cls: [] for cls in classes}
val_split = {cls: [] for cls in classes}

for cls in classes:
    train_split[cls] = class_paths[cls][:int(len(class_paths[cls]) * TRAIN_SPLIT)]
    val_split[cls] = class_paths[cls][int(len(class_paths[cls]) * TRAIN_SPLIT):] 

train_files = []
val_files = []

for i in train_split.values():
    train_files.extend(i)

for i in val_split.values():
    val_files.extend(i)
    
print(len(train_files))
print(len(val_files))

for file in train_files:
    shutil.copy(osp.join(TRAIN_VAL_DATA_PATH, "labels", file), osp.join(TRAIN_DATA_PATH, "labels", file))
    shutil.copy(osp.join(TRAIN_VAL_DATA_PATH, "images", file.replace(".txt", ".jpg")), osp.join(TRAIN_DATA_PATH, "images", file.replace(".txt", ".jpg")))

for file in val_files:
    shutil.copy(osp.join(TRAIN_VAL_DATA_PATH, "labels", file), osp.join(VAL_DATA_PATH, "labels", file))
    shutil.copy(osp.join(TRAIN_VAL_DATA_PATH, "images", file.replace(".txt", ".jpg")), osp.join(VAL_DATA_PATH, "images", file.replace(".txt", ".jpg")))
