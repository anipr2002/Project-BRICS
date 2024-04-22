import cv2
import time
import os

#RGB camera. Use port for your laptop
cap = cv2.VideoCapture(1)
## Ir camera of realsense
cap_ir = cv2.VideoCapture(2)

#Auto exposure for cameras that don't have it
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.3)


def capture(mode = 1, num_images = 10, path = os.getcwd()+"/src/capture_images/" + "images/"):
    if not os.path.exists(path):
            os.makedirs(path)

    if mode == 1: #Manual capturing
        num = 0
        while True:
            ret, frame = cap.read()
            cv2.imshow('frame', frame)
            
            if cv2.waitKey(1) & 0xFF == ord(' '):
                num += 1
                cv2.imwrite(f'{path}/manual_image_{num}.jpg', frame)
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    elif mode == 2:
        #Auto capturing for seconds
        delay = 0.1
        for i in range(num_images):
        
            print(f"Taking picture {i+1} in {delay} seconds...")
            time.sleep(delay)

            ret, frame = cap.read()
        
            image_path = os.path.join(path, f"captured_image_{i+1}.jpg")
            print(image_path)
            cv2.imwrite(image_path, frame)
            print(f"Image {i+1} captured successfully!")
    
        cap.release()
        cv2.destroyAllWindows()

capture(mode = 2, num_images= 20)