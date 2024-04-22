import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.3)

num = 0
while True:
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    
    if cv2.waitKey(1) & 0xFF == ord(' '):
        num += 1
        cv2.imwrite(f'image_{num}.jpg', frame)
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break