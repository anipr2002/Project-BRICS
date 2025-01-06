import os
from PIL import Image
from ultralytics import YOLO
import cv2
import numpy as np
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

dataset = "InitialPass"
trains_path = f"/home/vam_c/Documents/BRICS/personal_ws/trains"

models = {
          f"{dataset}_control" : {"model" : "",  "preprocess" : None, "args" : None}, 
          f"{dataset}_canny" : {"model" : "", "preprocess" : edge_detections.canny_edge, "args" : {"image" : ""}}, 
          f"{dataset}_active_canny" : {"model" : "", "preprocess" : edge_detections.active_canny, "args" : {"image" : ""}},
          f"{dataset}_HED1" : {"model" : "", "preprocess" : edge_detections.hed_edge, "args" : {"image" : "", "layer" : 1}},
          f"{dataset}_HED2" : {"model" : "", "preprocess" : edge_detections.hed_edge, "args" : {"image" : "", "layer" : 2}},
          f"{dataset}_anime_style" : {"model" : "", "preprocess" : edge_detections.info_drawing, "args" : {"image" : "", "model_name" : "anime_style"}},
          f"{dataset}_contour_style" : {"model" : "", "preprocess" : edge_detections.info_drawing, "args" : {"image" : "", "model_name" : "contour_style"}},
          f"{dataset}_opensketch_style" : {"model" : "", "preprocess" : edge_detections.info_drawing, "args" : {"image" : "", "model_name" : "opensketch_style"}},
          }

def preprocess(model_name, model):
    if model["preprocess"]:
        img = model["preprocess"](**model["args"])
        if isinstance(img, list):
            img = img[int(model_name.split("HED")[-1])-1]
    return img

# Load the trained model
for model_name in models:
    models[model_name]["model"] = YOLO(f'{trains_path}/{dataset}/{model_name}/weights/best.pt')
    print(f"Loaded model: {model_name}")



def image_predictions(image_dir, output_dir):
    grid_size = int(len(os.listdir(image_dir))**0.5) + 1


    for model_name in models:
        image_results = []
        os.makedirs(os.path.join(output_dir, model_name), exist_ok=True)
        # Load images in their original size and predict
        for filename in sorted(os.listdir(image_dir)):
            if filename.endswith('.jpg') or filename.endswith('.png'):
                # Load image in its original size
                img_path = os.path.join(image_dir, filename)
                img = cv2.imread(img_path)
                img = resize_pad_image(img)[0]

                if models[model_name]["args"] is not None:
                    models[model_name]["args"]["image"] = img
                    
                    img = preprocess(model_name, models[model_name])

                results = models[model_name]["model"].predict(img, conf=0.9)
                cv2.imshow("Prediction", results[0].plot())
                # Save the visualized result and add it to the list for stitching
                result_image = results[0].plot()  # Visualize prediction
                result_image_path = os.path.join(output_dir, model_name, filename)
                cv2.imwrite(result_image_path, result_image)
                image_results.append(result_image)

        # Calculate the number of images required for the grid
        required_images = grid_size * grid_size

        # Add blank images if we have fewer than required images
        if len(image_results) < required_images:
            # Get the shape of the first image as a reference for creating blank images
            ref_height, ref_width = image_results[0].shape[:2]
            blank_image = np.zeros((ref_height, ref_width, 3), dtype=np.uint8)  # Black image placeholder
            image_results.extend([blank_image] * (required_images - len(image_results)))

        # Create the grid with filled blank images if necessary
        rows = [np.hstack(image_results[i*grid_size:(i+1)*grid_size]) for i in range(grid_size)]
        stitched_image = np.vstack(rows)  # Stack the rows vertically to form the grid

        # Save the stitched image
        stitched_image_path = os.path.join(output_dir, f'{model_name}/{model_name}_{image_dir.split("/")[-1]}_stitched_results_{grid_size}x{grid_size}.jpg')
        cv2.imwrite(stitched_image_path, stitched_image)

        print(f"Stitched {grid_size}x{grid_size} image saved at: {stitched_image_path}")

def video_predictions(source, model):
    cap = cv2.VideoCapture(source)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        img = resize_pad_image(frame)[0]
        if models[model]["args"] is not None:
            models[model]["args"]["image"] = img
            img = preprocess(model, models[model])
        results = models[model]["model"].predict(img, conf=0.8)
        
        cv2.imshow("Prediction", results[0].plot())

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
# Directory containing the images to test
image_dir = '/home/reddy/BRICS/chirag/Project-BRICS/src/yolo/captured'
output_dir = f'/home/reddy/BRICS/chirag/Project-BRICS/src/yolo/predictions'
# os.makedirs(output_dir, exist_ok=True)

# image_predictions(image_dir, output_dir)
video_predictions(6, f"{dataset}_contour_style")