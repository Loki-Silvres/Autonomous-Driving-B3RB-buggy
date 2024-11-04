import os
import os.path as osp
import numpy as np

# rng = default_rng(27)

image_path = '/home/loki/B3RB_ENIAC/data/train/images'
images = sorted(os.listdir(image_path))

label_path = '/home/loki/B3RB_ENIAC/data/train/labels'
labels = sorted(os.listdir(label_path))

rand_prefixes = np.random.permutation(len(images))

for i in range(len(images)):

    os.rename(osp.join(image_path, images[i]), osp.join(image_path, str(rand_prefixes[i]) + "_" + images[i]))
    os.rename(osp.join(label_path, labels[i]), osp.join(label_path, str(rand_prefixes[i]) + "_" + labels[i]))