import torch
import torchvision
import cv2
import os
import numpy as np
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor
import supervision as sv
from PIL import Image
import time
import matplotlib.pyplot as plt

# Print PyTorch and Torchvision versions, check CUDA availability
print("PyTorch Version: ", torch.__version__)
print("Torchvision Version: ", torchvision.__version__)
print("CUDA Available: ", torch.cuda.is_available())

brick_image = os.path.join(os.path.dirname(__file__), '/data/reddy/first_pass/test/images/manual_image_197_rot225_jpg.rf.4d5e5da92f5cd123d9499c13b7f74a90.jpg')
CHECKPOINT_VIT_L = os.path.join(os.path.dirname(__file__), './model/sam_vit_l_0b3195.pth')
CHECKPOINT_VIT_H = os.path.join(os.path.dirname(__file__), './model/sam_vit_h_4b8939.pth')
CHECKPOINT_VIT_B = os.path.join(os.path.dirname(__file__), './model/sam_vit_b_01ec64.pth')

# Define model checkpoints and device
DEVICE = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')


CHECKPOINT = CHECKPOINT_VIT_B
start_time = time.time()
sam = sam_model_registry["vit_b"](checkpoint=CHECKPOINT).to(device=DEVICE)
mask_generator = SamAutomaticMaskGenerator(sam)

image_bgr = cv2.imread(brick_image)
image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

sam_result = mask_generator.generate(image_rgb)
for i, j in enumerate(sam_result):
    if j["area"] >= 50000 and j["area"] <= 80000:
        break
sam_result = [sam_result[i]]
# print(sam_result)
mask_annotator = sv.MaskAnnotator(color_lookup=sv.ColorLookup.INDEX)
detections = sv.Detections.from_sam(sam_result=sam_result)
annotated_image = mask_annotator.annotate(scene=image_rgb.copy(), detections=detections)

seg_arr = [[int(value) for value in row] for row in sam_result[0]['segmentation']]
seg_arr = np.array(seg_arr)

contours, _ = cv2.findContours(sam_result[0]['segmentation'].astype(np.uint8), 1, 1) # not copying here will throw an error
rect = cv2.minAreaRect(contours[0]) # basically you can feed this rect into your classifier
(x,y),(w,h), a = rect # a - anglecv2.imwrite(os.path.join(os.path.dirname(__file__), f'./results/result_{"vit_b"}.jpg'), rect1)

box = cv2.boxPoints(rect)
box = np.int0(box) #turn into ints
rect2 = cv2.drawContours(image_bgr.copy(),[box],0,(0,0,255),3)

cv2.imwrite(os.path.join(os.path.dirname(__file__), f'./results/result_{"vit_b"}.jpg'), rect2)

# for row_num, row in enumerate(seg_arr):
#     positive = [(i, row_num) for i,j in enumerate(row) if j == True]

#     if len(positive) > 0:
#         break

    
# result = Image.fromarray(annotated_image)
# bbox = sam_result[0]['bbox']
# result = cv2.circle(image_bgr, (bbox[0], bbox[1]), 5, (0, 255, 0), -1)
# result = cv2.circle(result, (bbox[0]+bbox[2], bbox[1]), 5, (0, 255, 0), -1)
# result = cv2.circle(result, (bbox[0], bbox[1]+bbox[3]), 5, (0, 255, 0), -1)
# result = cv2.circle(result, (bbox[0]+bbox[2], bbox[1]+bbox[3]), 5, (0, 255, 0), -1)
# result = cv2.rectangle(image_bgr, (bbox[0]-10, bbox[1]-10), (bbox[0]+bbox[2]+10, bbox[1]+bbox[3]+10), (0, 255, 0), 2)
# result = cv2.circle(result, (positive[-1][0], positive[-1][1]), 5, (0, 255, 0), -1)
# result = cv2.rectangle(image_bgr, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
# cv2.imwrite(os.path.join(os.path.dirname(__file__), f'./results/result_{"vit_b"}.jpg'), result)
# comparison = np.hstack((image_rgb, annotated_image))
# comparison = Image.fromarray(comparison)

# comparison.save(os.path.join(os.path.dirname(__file__), f'./results/result_{"vit_b"}.jpg'))
print(f"Time taken for vit_h: {time.time() - start_time:.2f} seconds")

print("Benchmarking complete!")