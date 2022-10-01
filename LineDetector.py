import cv2
import numpy as np
import math

def region_of_interest(img):
    height, width = img.shape[0], img.shape[1]
    
    polygon1 = np.array([[ (60,height), (200,0), (460,0), (460,height) ]])
    mask1 = np.zeros_like(img)
    cv2.fillPoly(mask1,polygon1,(255,0,0))
    masked_img = cv2.bitwise_and(mask1,img)

    polygon2 = np.array([[ (0, int(height/2)), (int(width/2),int(height/2)), (int(width/2),height), (0,height) ]])
    mask2 = np.zeros_like(img)
    cv2.fillPoly(mask2,polygon2,(255,0,0))
    masked_img = cv2.bitwise_and(mask2,masked_img)

    return masked_img

def detectLine(image):
    
    threshold1 = 100
    threshold2 = 200
    theta = 0
    #start_x = 0
    #start_y = 0
    minLineLength = 50
    maxLineGap = 10
    k_width = 7
    k_height = 7
    max_slider = 10
    threshold=5
    desired_angle = 155
    avr = -1

    # 0(straight), 1(left), 2(right)

    direction = -1

    #height, width, channels = image.shape

    #start_y = int(height*2/3)

    #cv2.rectangle(cropped,(start_x,start_y),(width,height),(0,0,0))

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # given input image, kernel width =5 height = 5, Gaussian kernel standard deviation
    blurred = cv2.GaussianBlur(gray, (k_width, k_height), 0)

    # Find the edges in the image using canny detector
    edged = cv2.Canny(blurred, threshold1, threshold2)

    cropped = region_of_interest(edged)

    # it will return line coordinates,it will return 3darray.
    lines = cv2.HoughLinesP(cropped,rho=1,theta=np.pi/180,threshold=max_slider,minLineLength=minLineLength,maxLineGap=maxLineGap)

    if lines is not None:
        for x in range(0, len(lines)):
            l = lines[x][0]
            x1,y1,x2,y2 = l[0],l[1],l[2],l[3]
        #for x1,y1,x2,y2 in lines[x]:
            # draw line in image using cv2.line function.
            cv2.line(cropped,(x1,y1),(x2,y2),(255,0,0),3)
            theta = theta + math.atan2((y2-y1),(x2-x1))
        avr = (theta/len(lines)) * 180/np.pi
        print(avr)
        #print(len(lines))
        #cv2.line(cropped,(lines[0][0][0],lines[0][0][1]),(lines[0][0][2],lines[0][0][3]),(255,0,0),3)

    #cv2.line(image, (150,height),(240,int(height/2)),(255,0,0),3)
        
    if(abs(avr) > 75 and abs(avr) < 105):
        direction = 0
    else:
        direction = 2
    #if(theta>threshold):
    #    direction = 2
    #if(theta<-threshold):
    #    direction = 1
    #if(abs(theta)<threshold):
    #    direction = 0


    cv2.imshow("edged",edged)
    cv2.imshow("cropped",cropped)

    return direction

def main():
    
    vid = cv2.VideoCapture(0)
    while True:
        ret, img = vid.read()
        #removeGlare(img)
        direction = detectLine(img)
        if direction == 0:
            print("Go straight")
        elif direction == 1:
            print("Go left")
        elif direction == 2:
            print("Go right")
        else :
            print("Unidentify")

        cv2.imshow("test", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vid.release()
    cv2.destroyAllWindows()

    #img = cv2.imread("TAG_AR\Road9.png")
    #removeGlare(img)
    #direction = detectLine(img)
    #if direction == 0:
    #    print("go straight")
    #elif direction == 1:
    #    print("go left")
    #elif direction == 2:
    #    print("go right")
    #else :
    #    print("unidentify")

    #cv2.imshow("test", img)
    #cv2.waitKey(0)

if __name__ == "__main__":
    main()