import os 
import os.path as osp
import shutil 

img_folder_path = '/home/loki/B3RB_ENIAC/data/test/aim_images/non_sign_images'
destination_path = '/home/loki/B3RB_ENIAC/data/temp'
check_existence_at = "/home/loki/B3RB_ENIAC/runs/detect/predict5/labels"
label = None
for file_name in os.listdir(img_folder_path):
    label_name = file_name.replace('.jpg', '.txt')
    if osp.exists(osp.join(check_existence_at, label_name)):
        continue

    with open(osp.join(destination_path, "labels", label_name), 'w') as f:
        pass
    f.close()
    shutil.copy(osp.join(img_folder_path, file_name), osp.join(destination_path, "images", file_name))