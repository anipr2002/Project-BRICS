import os
import cv2 
import numpy as np
import sys
import time
import imutils

path = "data/single_brick_images"

images = os.listdir(path)

def rotate(image, angle):
    height, width = image.shape[:2]
    centerX, centerY = (width // 2, height // 2)
    
    M = cv2.getRotationMatrix2D((centerX, centerY), angle, 0.5)
    rotated = cv2.warpAffine(image, M, (width, height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    
    return rotated
print(os.getcwd())
for image_path in images:
    print(image_path)
    image = cv2.imread(os.path.join(path, image_path))

    image_rot45_path = os.path.join(path, image_path[:-4]+"_rot45.jpg")
    image_rot90_path = os.path.join(path, image_path[:-4]+"_rot90.jpg")
    image_rot135_path = os.path.join(path, image_path[:-4]+"_rot135.jpg")
    image_rot180_path = os.path.join(path, image_path[:-4]+"_rot180.jpg")
    image_rot225_path = os.path.join(path, image_path[:-4]+"_rot225.jpg")
    image_rot270_path = os.path.join(path, image_path[:-4]+"_rot270.jpg")
    image_rot315_path = os.path.join(path, image_path[:-4]+"_rot315.jpg")
    
    image_rot45 = imutils.rotate(image, 45)
    image_rot90 = imutils.rotate(image, 90)
    image_rot135 = imutils.rotate(image, 135)
    image_rot180 = imutils.rotate(image, 180)
    image_rot225 = imutils.rotate(image, 225)
    image_rot270 = imutils.rotate(image, 270)
    image_rot315 = imutils.rotate(image, 315)
    
    cv2.imwrite(image_rot45_path, image_rot45)
    cv2.imwrite(image_rot90_path, image_rot90)
    cv2.imwrite(image_rot135_path, image_rot135)
    cv2.imwrite(image_rot180_path, image_rot180)
    cv2.imwrite(image_rot225_path, image_rot225)
    cv2.imwrite(image_rot270_path, image_rot270)
    cv2.imwrite(image_rot315_path, image_rot315)
