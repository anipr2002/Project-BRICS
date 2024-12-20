import json
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
from PIL import Image, ImageFont, ImageDraw
import edge_detections

def resize_pad_image(image, mask = False, new_shape=(640, 640)):
    # Resize image to fit into new_shape maintaining aspect ratio
    h, w = image.shape[:2]
    scale = min(new_shape[1] / w, new_shape[0] / h)
    nw, nh = int(w * scale), int(h * scale)

    # Resize image with the scaling factor
    resized_image = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)

    if mask == True:
        new_image = np.full((new_shape[0], new_shape[1]), False, dtype=bool)
    else:
        # Create a new image with padding
        new_image = np.full((new_shape[0], new_shape[1], 3), 128, dtype=np.uint8)

    # Calculate padding
    top = (new_shape[0] - nh) // 2
    left = (new_shape[1] - nw) // 2

    # Place the resized image in the new image with padding
    new_image[top:top + nh, left:left + nw] = resized_image

    return new_image, scale, top, left

def resize_pad_mask(mask, new_shape=(640, 640)):
    # Get the original dimensions
    original_height, original_width = mask.shape
    
    # Calculate the scale to maintain aspect ratio
    scale = min(new_shape[1] / original_width, new_shape[0] / original_height)
    
    # Calculate the new width and height while maintaining aspect ratio
    new_width = int(original_width * scale)
    new_height = int(original_height * scale)
    
    # Resize the mask
    resized_mask = cv2.resize(mask.astype(np.uint8), (new_width, new_height), interpolation=cv2.INTER_NEAREST)
    
    # Create a new blank mask with the target dimensions
    new_mask = np.full(new_shape, False, dtype=bool)
    
    # Calculate padding
    pad_top = (new_shape[0] - new_height) // 2
    pad_left = (new_shape[1] - new_width) // 2
    
    # Place the resized mask into the new mask with padding
    new_mask[pad_top:pad_top + new_height, pad_left:pad_left + new_width] = resized_mask.astype(bool)
    
    return new_mask, scale, pad_top, pad_left

def rle_to_binary_mask(rle):
    """Converts a COCOs run-length encoding (RLE) to binary mask.
    :param rle: Mask in RLE format
    :return: a 2D binary numpy array where '1's represent the object
    """
    binary_array = np.zeros(np.prod(rle.get('size')), dtype=bool)
    counts = rle.get('counts')

    start = 0
    for i in range(len(counts) - 1):
        start += counts[i]
        end = start + counts[i + 1]
        binary_array[start:end] = (i + 1) % 2

    binary_mask = binary_array.reshape(*rle.get('size'), order='F')

    return binary_mask

def resize_bounding_box(bbox, scale, pad_top, pad_left, margin=0.075):
    """
    Resizes a rotated bounding box with added margins while preserving its orientation.
    
    Args:
        bbox (list): List of (x, y) coordinates of the bounding box.
        scale (float): Scale factor used for resizing.
        pad_top (int): Top padding added during image resizing.
        pad_left (int): Left padding added during image resizing.
        margin (float): Fraction of the bounding box dimensions to add as margin.
        
    Returns:
        np.ndarray: Resized and adjusted bounding box coordinates with margins.
    """
    # Convert the bounding box to a NumPy array
    bbox_array = np.array(bbox, dtype=np.float32)
    
    # Find the center of the box
    rect = cv2.minAreaRect(bbox_array)
    center, size, angle = rect  # center (x, y), size (width, height), and rotation angle
    
    # Add margin to the width and height
    width, height = size
    width += margin * width
    height += margin * height

    # Scale the center position
    center_x = center[0] * scale + pad_left
    center_y = center[1] * scale + pad_top
    scaled_center = (center_x, center_y)

    # Create the new rotated rectangle with the updated size
    new_rect = (scaled_center, (width * scale, height * scale), angle)

    # Get the four corners of the new bounding box
    resized_bbox = cv2.boxPoints(new_rect)

    return resized_bbox


paths = ["/data/reddy/BRICS/single_brick"]
dataset_path = "/data/reddy/BRICS/datasets/single_brick"

os.makedirs(dataset_path + "/annotated_images", exist_ok=True)
os.makedirs(dataset_path + "/masked_images", exist_ok=True)

for i in ["control", "canny", "active_canny", "anime_style", "contour_style", "opensketch_style"]:
    os.makedirs(dataset_path + f"/{i}/train/images", exist_ok=True)
    os.makedirs(dataset_path + f"/{i}/test/images", exist_ok=True)
    os.makedirs(dataset_path + f"/{i}/val/images", exist_ok=True)
    os.makedirs(dataset_path + f"/{i}/train/labels", exist_ok=True)
    os.makedirs(dataset_path + f"/{i}/test/labels", exist_ok=True)
    os.makedirs(dataset_path + f"/{i}/val/labels", exist_ok=True)

    os.makedirs(dataset_path + f"/annotated_images/{i}", exist_ok=True)

for i in range(1, 6):
    os.makedirs(dataset_path + "/HED/" + str(i) + "/train/images", exist_ok=True)
    os.makedirs(dataset_path + "/HED/" + str(i) + "/test/images", exist_ok=True)
    os.makedirs(dataset_path + "/HED/" + str(i) + "/val/images", exist_ok=True)
    os.makedirs(dataset_path + "/HED/" + str(i) + "/train/labels", exist_ok=True)
    os.makedirs(dataset_path + "/HED/" + str(i) + "/test/labels", exist_ok=True)
    os.makedirs(dataset_path + "/HED/" + str(i) + "/val/labels", exist_ok=True)

    os.makedirs(dataset_path + "/annotated_images/HED/" + str(i), exist_ok=True)


existing_images = []
if os.path.exists(dataset_path + "/annotated_images/control"):
    existing_images = ["_".join(i.split("_")[2:])for i in os.listdir(dataset_path + "/annotated_images/control")]

print("Existing images:", existing_images)

classes = {"unusable": 0, "usable": 1}

for path in paths:
    head, tail = os.path.split(path)
    path = os.path.join(head, tail.replace(" ", "_"))
    if not os.path.exists(path):
        os.rename(os.path.join(head, tail), path)

    if not os.path.exists(path + "/coco_annotations.json"):
        continue

    print("Annotating images in", path)

    with open(path + "/coco_annotations.json") as f:
        data = json.load(f)
        annotations = data["annotations"]
        images = data["images"]
        categories = {category["id"]: category["name"] for category in data["categories"]}
        
    
    # print(categories)
    train_num = int(round(len(images) * 0.7))
    test_num = int(round(len(images) * 0.2))
    val_num = len(images) - train_num - test_num
    split_list = ["train" for i in range(train_num)] + ["test" for i in range(test_num)] + ["val" for i in range(val_num)]
    np.random.shuffle(split_list)


    annotations_grouped = {}
    for annotation in annotations:
        image_id = annotation["image_id"]
        if image_id not in annotations_grouped:
            annotations_grouped[image_id] = []
        annotations_grouped[image_id].append(annotation)
    
    
    for image in annotations_grouped:

        img = cv2.imread(f"{path}/{images[image-len(images)]['file_name']}")
        image_type = split_list.pop()
        resized_image, scale, pad_top, pad_left = resize_pad_image(img)
        
        annotated_image = np.copy(resized_image)
        mask_image = Image.fromarray(cv2.cvtColor(np.copy(resized_image), cv2.COLOR_BGR2RGB))

        annotations_text = ""
        for annotation in annotations_grouped[image]:
            mask = rle_to_binary_mask(annotation["segmentation"]).astype(np.uint8)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            largest_contour = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(largest_contour)
            (x,y),(w,h), a = rect
            
            box = cv2.boxPoints(rect)
            box = np.intp(box) #turn into ints
            resized_box = resize_bounding_box(box, scale, pad_top, pad_left)
            resized_box = np.intp(resized_box)

            # Class = annotation["category_id"]
            Class = classes[categories[annotation["category_id"]].split("_")[-1]]
            
            annotations_text += f"{Class} {' '.join([str((coord/640)) for coord in resized_box.flatten()])}\n"

            color = (0, 0, 0)
            annotated_image = cv2.drawContours(annotated_image, [resized_box], 0, color, 1)

            mask = resize_pad_mask(mask)[0]
            mask = mask.astype(np.uint8) * 255
            mask_image.putalpha(255)
            mask = Image.fromarray(mask, mode="L")
            overlay = Image.new('RGBA', mask_image.size)
            draw_ov = ImageDraw.Draw(overlay)
            draw_ov.bitmap((0, 0), mask, fill=(color[2]+255, color[1], color[0]+255, 64))
            mask_image = Image.alpha_composite(mask_image, overlay)

        brick_name = categories[annotation["category_id"]]

        if f"{brick_name}_{image}.jpg" in existing_images:
            print(f"Image {brick_name}_{image} already exists in the dataset. Skipping...")
            continue

        for i in ["control", "canny", "active_canny", "anime_style", "contour_style", "opensketch_style"]:
            with open(f"{dataset_path}/{i}/{image_type}/labels/image_{brick_name}_{image}.txt", "w") as f:
                f.write(annotations_text)
        for i in range(1, 6):
            with open(f"{dataset_path}/HED/{i}/{image_type}/labels/image_{brick_name}_{image}.txt", "w") as f:
                f.write(annotations_text)    
        
        mask_image.save(f"{dataset_path}/masked_images/masked_image_{brick_name}_{image}.png")
        cv2.imwrite(f"{dataset_path}/annotated_images/control/annotated_image_{brick_name}_{image}.jpg", annotated_image)

        cv2.imwrite(f"{dataset_path}/control/{image_type}/images/image_{brick_name}_{image}.jpg", resized_image)

        edge_detections.canny_edge(resized_image, f"{dataset_path}/canny/{image_type}/images/image_{brick_name}_{image}.jpg", annotation=[dataset_path, resized_box])
        edge_detections.active_canny(resized_image, f"{dataset_path}/active_canny/{image_type}/images/image_{brick_name}_{image}.jpg", annotation=[dataset_path, resized_box])
        edge_detections.hed_edge(resized_image, f"{dataset_path}/HED/PlAcEhOlDeR/{image_type}/images/image_{brick_name}_{image}.jpg", annotation=[dataset_path, resized_box])
        edge_detections.info_drawing(resized_image, "anime_style", f"{dataset_path}/anime_style/{image_type}/images/image_{brick_name}_{image}.jpg", annotation=[dataset_path, resized_box])
        edge_detections.info_drawing(resized_image, "contour_style", f"{dataset_path}/contour_style/{image_type}/images/image_{brick_name}_{image}.jpg", annotation=[dataset_path, resized_box])
        edge_detections.info_drawing(resized_image, "opensketch_style", f"{dataset_path}/opensketch_style/{image_type}/images/image_{brick_name}_{image}.jpg", annotation=[dataset_path, resized_box])
        print(f"Done image {brick_name}_{image}")

# for style in ["anime_style", "contour_style", "opensketch_style"]:
#     for split in ["train", "test", "val"]:
#         informative_drawing.process_images(f"{dataset_path}/control/{split}/images", f"{dataset_path}", style)
