import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

while True:
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    
    if cv2.waitKey(1) & 0xFF == ord(' '):
        cv2.imwrite('image.jpg', frame)
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break