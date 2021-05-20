import cv2
import numpy as np
import pdb

img = cv2.imread("img/street.png")
height, width, _ = img.shape

# convert RGB blue to BGR blue
RGB_blue = [58,99,173]
BGR_blue = np.uint8([[[RGB_blue[2], RGB_blue[1], RGB_blue[0]]]])

# get HSV blue and a range 
HSV_blue = cv2.cvtColor(BGR_blue, cv2.COLOR_BGR2HSV)[0][0]
range_val = 25
lower_blue = np.array([HSV_blue[0]-range_val, HSV_blue[1]-range_val, HSV_blue[2]-range_val])
upper_blue = np.array([HSV_blue[0]+range_val, HSV_blue[1]+range_val, HSV_blue[2]+range_val])

# convert from RGB to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# apply a mask
mask = cv2.inRange(hsv, lower_blue, upper_blue)

#cv2.imshow("mask1", mask)

mask = np.ones((50,50), np.uint8)
height, width = mask.shape
grid_size = 2
for i in range(height):
    for j in range(width):
        if i % grid_size == 0 and j % grid_size == 0:
            pass
        else:
            mask[i,j] = 0


cv2.imshow("mask2", mask)

#kernel = np.array(([1,0,-1], [0,0,0], [-1,0,1]), np.uint8)
#mask = cv2.filter2D(mask, -1, kernel)
#cv2.imshow("mask3", mask)

cv2.waitKey(0)