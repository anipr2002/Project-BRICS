import numpy as np
from segment_anything import SamPredictor, sam_model_registry,SamAutomaticMaskGenerator
import torch 
import os
import cv2
import supervision as sv
from PIL import Image

DEVICE = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
MODEL_TYPE = "vit_b"
CHECKPOINT_VIT_B = "/home/reddy/BRICS/data/models/sam_vit_b_01ec64.pth"

sam = sam_model_registry[MODEL_TYPE](checkpoint=CHECKPOINT_VIT_B).to(device=DEVICE)
mask_predictor = SamPredictor(sam)

IMAGE_PATH = os.path.join(os.path.dirname(__file__), '/home/reddy/BRICS/data/single_brick_images/manual_image_24.jpg')



def get_mask_details(image_path):  
    mask_generator = SamAutomaticMaskGenerator(sam)
    image_bgr = cv2.imread(image_path)
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    sam_result = mask_generator.generate(image_rgb)
    return sam_result[1]
    # only if the area of the mask is 57533 return the mask result
    



def get_mask_image(mask, image_path):
    image_bgr = cv2.imread(image_path)
    mask_annotator = sv.MaskAnnotator(color_lookup=sv.ColorLookup.INDEX)
    detections = sv.Detections.from_sam(sam_result=mask)
    annotated_image = mask_annotator.annotate(scene=image_bgr.copy(), detections=detections)

    result = Image.fromarray(annotated_image)
    return result

# Save the mask image

mask = get_mask_details(IMAGE_PATH)
mask_image = get_mask_image(mask, IMAGE_PATH)
mask_image.save(os.path.join(os.path.dirname(__file__), '/home/reddy/BRICS/anirudh/Project-BRICS/src/sam-meta/results/mask_image.jpg'))


# Get the bounding box from the mask details dict_keys(['segmentation', 'area', 'bbox', 'predicted_iou', 'point_coords', 'stability_score', 'crop_box'])
# def get_bounding_boxes(mask : list[dict[str,any]]):
#     # iterate over the mask and get the bounding box
#     bounding_boxes = []
#     for mask_details in mask:
#         if(mask_details["area"] > 5500 and mask_details["area"] < 10000):
#             bounding_boxes.append(mask_details['bbox'])
#     return bounding_boxes


# # Draw the bounding box on the image
# def draw_bounding_boxes(image_path, bounding_boxes):
#     image = cv2.imread(image_path)
#     for box in bounding_boxes:
#         cv2.rectangle(image, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
#     return image
    

