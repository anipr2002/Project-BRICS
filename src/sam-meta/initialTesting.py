import torch
import torchvision
import cv2
import os
import numpy as np
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor
import supervision as sv
from PIL import Image

# print the version of PyTorch and TorchVision and check if CUDA is available
print("PyTorch Version: ",torch.__version__)
print("Torchvision Version: ",torchvision.__version__)
print("CUDA Available: ",torch.cuda.is_available())

brick_image = os.path.join(os.path.dirname(__file__), './images/image.jpg') # vscode not recognizing file path shata
CHECKPOINT_VIT_L = os.path.join(os.path.dirname(__file__), './model/sam_vit_l_0b3195.pth')
CHECKPOINT_VIT_H = os.path.join(os.path.dirname(__file__), './model/sam_vit_h_4b8939.pth')
CHECKPOINT_VIT_B = os.path.join(os.path.dirname(__file__), './model/sam_vit_b_01ec64.pth')

DEVICE = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
MODEL_TYPE = ["vit_l","vit_h","vit_b"]

sam = sam_model_registry[MODEL_TYPE[0]](checkpoint=CHECKPOINT_VIT_L).to(device=DEVICE)
mask_generator = SamAutomaticMaskGenerator(sam)

image_bgr = cv2.imread(brick_image)
image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

sam_result = mask_generator.generate(image_rgb)
# ['segmentation', 'area', 'bbox', 'predicted_iou', 'point_coords', 'stability_score', 'crop_box']


mask_annotator = sv.MaskAnnotator(color_lookup=sv.ColorLookup.INDEX)
detections = sv.Detections.from_sam(sam_result=sam_result,)
annotated_image = mask_annotator.annotate(scene=image_bgr.copy(), detections=detections)

# convert ndarray to image
result = Image.fromarray(annotated_image)

comparions = np.hstack((image_rgb, annotated_image))
comparions = Image.fromarray(comparions)


comparions.save(os.path.join(os.path.dirname(__file__), './results/result.jpg'))