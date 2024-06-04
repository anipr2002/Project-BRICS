from ultralytics import YOLO

model = YOLO('/home/reddy/BRICS/anirudh/Project-BRICS/src/yolo/obb/train/weights/best.pt')

results = model.predict('/home/reddy/BRICS/anirudh/Project-BRICS/src/sam-meta/images/image.jpg', save = True)

