from ultralytics import YOLO
import os 
import json

paths = os.listdir('/home/loki/B3RB_ENIAC/data/test/aim_images/images_StopSign/images')
paths = [ os.path.join('/home/loki/B3RB_ENIAC/data/test/aim_images/images_StopSign/images', path) for path in paths]

model = YOLO("/home/loki/B3RB_ENIAC/runs/detect/train/weights/best.pt")

source = "/home/loki/B3RB_ENIAC/data/test/aim_images/non_sign_images/image_1.jpg"
source = paths

results = model.predict(source, stream=True, show=True, save = True)  # generator of Results objects
results = model.benchmark(data = source, imgsz=320)
print(results)

json.dump(results, open('/home/loki/B3RB_ENIAC/runs/detect/predict/results.json', 'w'))