import cv2
import time
import os




def capture(port_rgb = 1, port_ir = 2, mode = 1, num_images = 10, path = os.getcwd()+"/src/capture_images/" + "images/"):

    #RGB camera. Use port for your laptop
    cap = cv2.VideoCapture(port_rgb)
    ## Ir camera of realsense
    cap_ir = cv2.VideoCapture(port_ir)

    #Auto exposure for cameras that don't have it
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.3)

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
        cap_images = 0
        while cap_images < num_images:    
            cap_images += 1    
            print(f"Taking picture {cap_images} in {delay} seconds...")
            time.sleep(delay)

            ret, frame = cap.read()

            cv2.imshow('frame', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            image_path = os.path.join(path, f"captured_image_{cap_images}.jpg")
            print(image_path)
            cv2.imwrite(image_path, frame)
            print(f"Image {cap_images} captured successfully!")
    
        cap.release()
        cv2.destroyAllWindows()

capture(port_rgb = 0, mode = 2, num_images= 250)