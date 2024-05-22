import torch
import torchvision
import cv2
import os
import numpy as np
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor
import supervision as sv
from PIL import Image
import time

# Print PyTorch and Torchvision versions, check CUDA availability
print("PyTorch Version: ", torch.__version__)
print("Torchvision Version: ", torchvision.__version__)
print("CUDA Available: ", torch.cuda.is_available())

brick_image = os.path.join(os.path.dirname(__file__), '/home/reddy/BRICS/data/single_brick_images/manual_image_24.jpg')
CHECKPOINT_VIT_L = os.path.join(os.path.dirname(__file__), './model/sam_vit_l_0b3195.pth')
CHECKPOINT_VIT_H = os.path.join(os.path.dirname(__file__), './model/sam_vit_h_4b8939.pth')
CHECKPOINT_VIT_B = os.path.join(os.path.dirname(__file__), './model/sam_vit_b_01ec64.pth')

# Define model checkpoints and device
DEVICE = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
MODEL_TYPE = ["vit_l","vit_h", "vit_b"]


for model_type in MODEL_TYPE:
    CHECKPOINT = CHECKPOINT_VIT_L if model_type == "vit_l" else CHECKPOINT_VIT_H if model_type == "vit_h" else CHECKPOINT_VIT_B
    print(f"Benchmarking {model_type}...")
    start_time = time.time()
    sam = sam_model_registry[model_type](checkpoint=CHECKPOINT).to(device=DEVICE)
    mask_generator = SamAutomaticMaskGenerator(sam)

    image_bgr = cv2.imread(brick_image)
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

    sam_result = mask_generator.generate(image_rgb)

    mask_annotator = sv.MaskAnnotator(color_lookup=sv.ColorLookup.INDEX)
    detections = sv.Detections.from_sam(sam_result=sam_result)
    annotated_image = mask_annotator.annotate(scene=image_bgr.copy(), detections=detections)

    result = Image.fromarray(annotated_image)
    comparison = np.hstack((image_rgb, annotated_image))
    comparison = Image.fromarray(comparison)

    comparison.save(os.path.join(os.path.dirname(__file__), f'./results/result_{model_type}.jpg'))
    print(f"Time taken for {model_type}: {time.time() - start_time:.2f} seconds")

print("Benchmarking complete!")