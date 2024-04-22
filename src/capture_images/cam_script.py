import cv2

cap = cv2.VideoCapture(4)
cap2 = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.3)

def capture(mode = 1, seconds = 10):
    if mode == 1: #Manual capturing
        num = 0
        while True:
            ret, frame = cap.read()
            cv2.imshow('frame', frame)
            
            if cv2.waitKey(1) & 0xFF == ord(' '):
                num += 1
                cv2.imwrite(f'image_{num}.jpg', frame)
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    elif mode == 2:
        #Auto capturing for seconds
        pass