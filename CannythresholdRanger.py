import cv2
import numpy as np
import imutils

def nothing(x):
    pass

vid = cv2.VideoCapture(1, cv2.CAP_DSHOW)

# Create a window
cv2.namedWindow('image')

# Create trackbars for color change
cv2.createTrackbar('Threshold1', 'image', 0, 500, nothing)
cv2.createTrackbar('Threshold2', 'image', 0, 500, nothing)

# Set default value for Threshold2 trackbars
cv2.setTrackbarPos('Threshold2', 'image', 500)

# Initialize Thresholds values
Threshold2 = Threshold1 = 0
pThreshold2 = pThreshold1 = 0

# Load image
while True:
    _, img = vid.read()

    height, width, channels = img.shape

    start_y = int(height*2/3)

    cropped = img[start_y:height, 0:width]

    # Get current positions of all trackbars
    t1 = cv2.getTrackbarPos('Threshold1', 'image')
    t2 = cv2.getTrackbarPos('Threshold2', 'image')

    gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

    # given input image, kernel width =5 height = 5, Gaussian kernel standard deviation
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    # Find the edges in the image using canny detector
    edged = cv2.Canny(blurred, t1, t2)

    # Print if there is a change in HSV value
    if((pThreshold1 != Threshold1) | (pThreshold2 != Threshold2)):
        #print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
        pThreshold1 = Threshold1
        pThreshold2 = Threshold2
        
    # Display result image
    cv2.imshow("img",img)
    cv2.imshow('image', edged)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()

