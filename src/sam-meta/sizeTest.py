import torch
import torchvision
import cv2
import os
import numpy as np
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor
import supervision as sv
from PIL import Image


MODEL_CHECKPOINT_VIT_B = '/home/reddy/BRICS/data/models/sam_vit_b_01ec64.pth'
MODEL_TYPE = 'vit_b'
DEVICE = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')


sam = sam_model_registry[MODEL_TYPE](checkpoint=MODEL_CHECKPOINT_VIT_B).to(device=DEVICE)
mask_generator = SamAutomaticMaskGenerator(sam)

IMAGE_PATH = '/data/reddy/first_pass/test/images/manual_image_26_jpg.rf.d99a6612585907048684078e25be845b.jpg'

image_bgr = cv2.imread(IMAGE_PATH)
image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

sam_result = mask_generator.generate(image_rgb)

# if mask area is less than 1000, then ignore the mask

print(sam_result[0]["segmentation"])

# Draw a bbox around the mask in openCV
bbox = sam_result[0]["bbox"]
cv2.rectangle(image_bgr, (bbox[0], bbox[1]), (bbox[2]+bbox[0], bbox[3]+bbox[1]), (0, 255, 0), 2)
