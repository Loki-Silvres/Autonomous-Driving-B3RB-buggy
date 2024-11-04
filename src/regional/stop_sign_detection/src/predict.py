from ultralytics import YOLO

# Load a pretrained YOLOv8n model
model = YOLO("/home/loki/stop_sign_detection/runs/detect/train16/weights/best.pt")

# Define path to video file
source = "/home/loki/stop_sign_detection/2nd_half_track.mp4"

# Run inference on the source
results = model(source, stream=True, show=True, save = True)  # generator of Results objects