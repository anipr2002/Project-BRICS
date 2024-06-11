from ultralytics import YOLO

model = YOLO('/home/fsociety/Code/Projects/Project-BRICS/best.pt')

results = model.predict('/home/fsociety/Code/Projects/Project-BRICS/src/yolo/WhatsApp Image 2024-06-11 at 13.11.29.jpeg', save = True, conf=0.5)

