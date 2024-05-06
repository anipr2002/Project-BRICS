from ultralytics import YOLO


# Load a model antonio/Project-BRICS/src/Bricks_detector/data
# /home/reddy/BRICS/antonio/Project-BRICS/src/Bricks_detector/data
model = YOLO("yolov8n.yaml")  # build a new model from scratch
#model = YOLO('yolov8n.pt')  # load a pretrained model (recommended for training)
#model = YOLO('yolov8n.yaml').load('yolov8n.pt')  # build from YAML and transfer weights
# Use the model
#model.train(data="antonio/Project-BRICS/src/Bricks_detector/config.yaml", epochs=150)  # train the model


model.train(data="/home/reddy/BRICS/data/as_data/data3/data.yaml", epochs=150)  # train the model with Roboflow dataset
