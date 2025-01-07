import cv2
import numpy as np

# Load the image
cap = cv2.VideoCapture(4)
while True:
    ret, frame = cap.read()
    
    # image = cv2.imread('/home/fsociety/Downloads/4x4_1000-4.png')

    # Convert the image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()

    # Create the ArUco detector
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    # Detect the markers
    corners, ids, rejected = detector.detectMarkers(gray)
    
    
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow('Detected Markers', frame)
    
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break