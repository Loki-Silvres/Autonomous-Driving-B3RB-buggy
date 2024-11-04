import os
import os.path as osp
import matplotlib.pyplot as plt
import numpy.random as rand

rng = rand.default_rng(27)

label_folder_path: str = "/home/loki/B3RB_ENIAC/data/train_val/dataset/labels"

IMG_H, IMG_W = 240, 320

labels: dict = {'0':'left', '1':'right', '2':'stop', '3':'straight'}
labels_count: dict = {label: 0 for label in labels.values()}
objects_count: dict = {}
object_sizes: list[str] = ['tiny', 'small', 'medium', 'large'] # 16**2, 32**2, 96**2, inf
object_size_count: dict = {object_size: 0 for object_size in object_sizes}

ann_files: list[str] = os.listdir(label_folder_path)
for ann_file in ann_files:
    ann_file_path = osp.join(label_folder_path, ann_file)
    with open(ann_file_path, 'r') as f:
        lines = f.readlines()
        line_count = len(lines)
        
        if line_count not in objects_count:
            objects_count[line_count] = 0
        objects_count[line_count] += 1
        for line in lines:
            cls, x, y, w, h = line.split()
            labels_count[labels[cls]] += 1
            w = float(w)
            h = float(h)
            area = w * h * IMG_H * IMG_W
            if area < 10**2:
                object_size_count['tiny'] += 1
            elif area < 32**2:
                object_size_count['small'] += 1
            elif area < 96**2:
                object_size_count['medium'] += 1
            else:
                object_size_count['large'] += 1

print(f"Total number of images: {len(ann_files)}")

for count in sorted(objects_count.keys()): 
    print(f"Num of images with {count} objects: {objects_count[count]}")

for object_size in object_sizes:
    print(f"Num of images with {object_size} objects: {object_size_count[object_size]}")
print(f"count of images across classes", labels_count)

plt.bar(["object_count: " + str(count) for count in objects_count.keys()], objects_count.values(), color = [[rng.random(), rng.random(), rng.random()] for _ in range(len(objects_count.keys()))])
plt.xlabel('Number of Objects')
plt.ylabel('Number of Images')
plt.title(label='Number of objects per image',
          fontweight=10,
          pad='2.0')
plt.show()
plt.bar(object_size_count.keys(), object_size_count.values(), color = [[rng.random(), rng.random(), rng.random()] for _ in range(len(objects_count.keys()))])
plt.xlabel('Object Size')
plt.ylabel('Number of Images')
plt.title(label='Object size per bbox',
          fontweight=10,
          pad='2.0')
plt.show()
plt.bar(labels_count.keys(), labels_count.values(), color = [[rng.random(), rng.random(), rng.random()] for _ in range(len(objects_count.keys()))])
plt.xlabel('Labels')
plt.ylabel('Number of Images')
plt.title(label='Total Images in each class',
          fontweight=10,
          pad='2.0')
plt.show()






