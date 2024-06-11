import os
from PIL import Image
from ultralytics import YOLO
import cv2



model_path = "/home/fsociety/Code/Projects/Project-BRICS/src/best.pt"

# Load a model
model = YOLO(model_path)  # load a custom model

results = model.predict(source="1", show=True)