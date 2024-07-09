from ultralytics import YOLO
import cv2
import numpy as np

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

model = YOLO('/home/reddy/BRICS/chirag/Project-BRICS/runs/obb/train2/weights/best.pt')

image_path = '/home/reddy/BRICS/data/images/captured_image_59.jpg'
img = cv2.imread(image_path)
# img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img, scale, top, left = resize_pad_image(img, mask=False, new_shape=(640, 640))
#rotate image 90
# img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

cv2.imwrite(image_path, img)
results = model.predict(image_path, save = True, conf=0.5)

