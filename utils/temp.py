import os
import os.path as osp
import sys
import shutil
sys.path.append("/home/loki/B3RB_ENIAC")
from config.config import *

start_path = '/home/loki/B3RB_ENIAC/data/temp'

files = sorted(os.listdir(osp.join(start_path, "images")))
labels = sorted(os.listdir(osp.join(start_path, "labels")))

train_files = files[:int(len(files) * TRAIN_SPLIT)]
val_files = files[int(len(files) * TRAIN_SPLIT):]

for file in train_files:
    shutil.copy(osp.join(start_path, "images", file), osp.join(TRAIN_DATA_PATH, "images", file))
    shutil.copy(osp.join(start_path, "labels", file.replace(".jpg", ".txt")), osp.join(TRAIN_DATA_PATH, "labels", file.replace(".jpg", ".txt")))

for file in val_files:
    shutil.copy(osp.join(start_path, "images", file), osp.join(VAL_DATA_PATH, "images", file))
    shutil.copy(osp.join(start_path, "labels", file.replace(".jpg", ".txt")), osp.join(VAL_DATA_PATH, "labels", file.replace(".jpg", ".txt")))