# import os

# path = "data/single_brick_images"
# images = os.listdir(path)

# for image in images:
#     if "rot" in image:
#         os.remove(os.path.join(path, image))
# # num = 0
# # for image in images:
# #     num += 1
# #     print(os.path.join(path, f"manual_image_{num}.jpg"))
# #     os.rename(os.path.join(path, image), os.path.join(path, f"manual_image_{num}.jpg"))

import os
from PIL import Image
from ultralytics import YOLO
import cv2



model_path = "/home/fsociety/Code/Projects/Project-BRICS/best.pt"

# Load a model
model = YOLO(model_path)  # load a custom model

results = model.predict(source="0", show=True, conf=0.7)
