from ultralytics import YOLO

# Load a model
model = YOLO("yolov8n-obb.yaml")  # build a new model from YAML
model = YOLO("yolov8n-obb.pt")  # load a pretrained model (recommended for training)
model = YOLO("yolov8n-obb.yaml").load("yolov8n.pt")  # build from YAML and transfer weights

# Train the model
results = model.train(data="/data/reddy/dataset_blender/data.yaml", epochs=100, imgsz=640)