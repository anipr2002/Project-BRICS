import os
from PIL import Image
from ultralytics import YOLO
import cv2


root_path = "/data/reddy/BRICS"
for model in ["control", "canny", "active_canny", "HED1", "HED2", "anime_style", "contour_style", "opensketch_style"]:
    model_path = f"{root_path}/trains/single_brick/InitialPass_{model}/weights/best.pt"
    
    # Load model
    model = YOLO(model_path)
    print(f"Model {model_path} loaded successfully")

