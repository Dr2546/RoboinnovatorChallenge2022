import cv2
import numpy as np
import sys
import imutils

def rgb_hsv(r,g,b):
    h,s,v = cv2.cvtColor(np.uint8([[[b,g,r]]]),cv2.COLOR_BGR2HSV)[0][0]
    return h,s,v

def hsv_rgb(h,s,v):
    r,g,b = cv2.cvtColor(np.uint8([[[h,s,v]]]),cv2.COLOR_HSV2BGR)[0][0]
    return r,g,b

def isIn(a1,b1,c1,d1,a2,b2,c2,d2):
    if a1 >= a2 and b1 >= b2 and a1+c1 <= a2+c2 and b1+d1 <= b2+d2:
        return True
    else: 
        return False

def detectCone(img, middle_box_x, middle_box_y, middle_box_length, middle_box_height):

    isNear = False

    height, width, channels = img.shape

    #middle_box_x, middle_box_y, middle_box_length, middle_box_height = 200,160,344,100 #some number... 
    area_that_near = 2500 #some number...

    cv2.rectangle(img,(middle_box_x,middle_box_y),(middle_box_x + middle_box_length,middle_box_y + middle_box_height),(0,0,0),2)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #Yellow BGR = (0,255,255)
    lower_bound = np.array([16,137,216])
    upper_bound = np.array([31,255,255])
    mask = cv2.inRange(hsv, lower_bound, upper_bound )
    
    #define kernel size  
    kernel = np.ones((7,7),np.uint8)
    # Remove unnecessary noise from mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours from the mask
    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        x,y,w,h = cv2.boundingRect(c)
        #cv2.rectangle(img, (x, y), (x + w, y + h), (0,0,255), 2)
        #Give 1 mm = 3.8 pixel

        area = int(w*h)

        if area >= area_that_near and isIn(x,y,w,h,middle_box_x ,middle_box_y, middle_box_length, middle_box_height) : # and some condition that cone is in middle of camera)
           isNear = True
           #message = 'Object is near area = ' + str(area)
           #img = cv2.putText(img,message,(int(x+w/2),int(y+h/2)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.LINE_AA)

        #output = cv2.drawContours(img, contours, -1, (0, 0, 255), 3)
    return isNear


def main():
    vid = cv2.VideoCapture(0)
    while True:
        ret, img = vid.read()
        if detectCone(img, 150,200,344,100) or detectCone(img, 0,300,140,180) or detectCone(img, 500,300,140,180):
            print("Found")
        else:
            print("Not Found")
        
        cv2.imshow("test", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vid.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()