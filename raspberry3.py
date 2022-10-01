from smbus import SMBus
import time
import RPi.GPIO as GPIO

import cv2
import numpy as np
import math

addr = 0x8
bus = SMBus(1)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(16, GPIO.IN)
GPIO.setup(12, GPIO.IN)

first = False

def isIn(a1,b1,c1,d1,a2,b2,c2,d2):
    if a1 >= a2 and b1 >= b2 and a1+c1 <= a2+c2 and b1+d1 <= b2+d2:
        return True
    else: 
        return False

def detectCone(img, middle_box_x, middle_box_y, middle_box_length, middle_box_height):

    isNear = False

    height, width, channels = img.shape

    #middle_box_x, middle_box_y, middle_box_length, middle_box_height = 150,200,344,100 #some number... 
    area_that_near = 2500 #some number...

    #cv2.rectangle(img,(middle_box_x,middle_box_y),(middle_box_x + middle_box_length,middle_box_y + middle_box_height),(0,0,0),2)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #Red BGR = (0,255,255)
    #lower_bound = np.array([165,35,0])
    #upper_bound = np.array([179,255,255])
    
    #Orange
    lower_bound = np.array([3,110,170])
    upper_bound = np.array([18,255,255])
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
           #$message = 'Object is near area = ' + str(area)
           #img = cv2.putText(img,message,(int(x+w/2),int(y+h/2)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.LINE_AA)

        #output = cv2.drawContours(img, contours, -1, (0, 0, 255), 3)
    return isNear

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
            #cv2.line(cropped,(x1,y1),(x2,y2),(255,0,0),3)
            alpha = math.atan2((y2-y1),(x2-x1))
            if abs(alpha) > 15:
                theta = theta + alpha
        avr = (theta/len(lines)) * 180/np.pi
        print(avr)
        
    if(abs(avr) > 75 and abs(avr) < 105):
        direction = 0
    elif(avr > -75):
        direction = 2
    elif(avr < 75):
        direction = 1

    #cv2.imshow("edged",edged)
    #cv2.imshow("cropped",cropped)

    return direction

def dodge(video):
    _, img = video.read()
    start = time.time()
    while detectCone(img, 500,300,140,180):
        _, img = video.read()
        bus.write_byte(addr, 0xA0)

    end = time.time()-start

    bus.write_byte(addr, 0x88)
    time.sleep(3)

    bus.write_byte(addr, 0xC0)
    time.sleep(end+3)

    _, img = video.read()
    while detectCone(img, 0,300,140,180):
        _, img = video.read()
        bus.write_byte(addr, 0xC0)

    bus.write_byte(addr, 0x90)
    time.sleep(3)

    #bus.write_byte(addr, 0x88)
    #time.sleep(3)
    #bus.write_byte(addr, 0xC0)
    #time.sleep(3)
    #bus.write_byte(addr, 0x90)
    #time.sleep(3)

def transport(video):
    _, img = video.read()
    if GPIO.input(21) == 1:
        start = time.time()
        while detectCone(img, 500,300,140,180):
            _, img = video.read()
            bus.write_byte(addr, 0xA0)

        end = time.time()-start

        bus.write_byte(addr, 0x88)
        time.sleep(3)

        bus.write_byte(addr, 0xC0)
        time.sleep(end)

        GPIO.output(20,GPIO.HIGH)
        while GPIO.input(21) == 1 :
            pass

        _, img = video.read()
        while detectCone(img, 0,300,140,180):
            _, img = video.read()
            bus.write_byte(addr, 0xC0)

        bus.write_byte(addr, 0x90)
        time.sleep(3)
        #GPIO.output(20,GPIO.LOW)

def start():
    bus.write_byte(addr, 0xC0)
    time.sleep(3)

def stop():
    bus.write_byte(addr, 0xC0)
    time.sleep(3)
    bus.write_byte(addr, 0x84)
    time.sleep(3)
    bus.write_byte(addr, 0xC0)
    time.sleep(3)
    bus.write_byte(addr, 0x81)


def main():
    vid = cv2.VideoCapture(0)
    while True:
        if GPIO.input(16) == 0 and first == False:
            start()
            first = True

        if GPIO.input(12) == 1:
            first = False

        if GPIO.input(21) == 1:
            transport()
            GPIO.output(20, GPIO.LOW)

        #track lines and cone
        _, img = vid.read()

        #if cone found
        if detectCone(img, 150,200,344,100):
            #dodge
            dodge(vid)

        #track line
        direction = detectLine(img)
        if direction == 0:
            #Straight
            bus.write_byte(addr, 0xC0)
        elif direction == 1:
            #Go left
            bus.write_byte(addr, 0x84)
        elif direction == 2:
            #Go right
            bus.write_byte(addr, 0x82)

        if GPIO.input(16) == 1:
            stop()

        #cv2.imshow("img",img)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break
        

        #if GPIO.input(21) == 0:
        #    #doing
        #    '''bus.write_byte(addr, 0xC0)#L
        #    while True:
        #        _, img = vid.read()
        #        if detectCone(img):
        #            bus.write_byte(addr,0xC8)
        #            time.sleep(4)
        #            bus.write_byte(addr,0xD0)
        #            time.sleep(4)
                
        #        direction = detectLine(img)
        #        if direction == 0:
        #            bus.write_byte(addr,0xC0)
        #        elif direction == 2:
        #            bus.write_byte(addr,0x82)
        #        if cv2.waitKey(1) & 0xFF == ord('q'):
        #            break                
                
        #    bus.write_byte(addr, 0x81)
        #    vid.release()
        #    cv2.destroyAllWindows()'''
        #    #bus.write_byte(addr, 0xC0)#B
        #    #time.sleep(5)
        #    #bus.write_byte(addr, 0x90)#B
        #    #time.sleep(5)
        #    #time.sleep(100)
        #    """bus.write_byte(addr, 0xA0)#B
        #    time.sleep(ti)
        #    bus.write_byte(addr, 0x90)#L
        #    time.sleep(ti)
        #    bus.write_byte(addr, 0x88)#R
        #    time.sleep(ti)
        #    bus.write_byte(addr, 0xD0)#FL
        #    time.sleep(ti)
        #    bus.write_byte(addr, 0xC8)#FR
        #    time.sleep(ti)
        #    bus.write_byte(addr, 0xB0) #BL
        #    time.sleep(ti)
        #    bus.write_byte(addr, 0xA2) #BR
        #    time.sleep(ti)
        #    bus.write_byte(addr, 0x81) #stop
        #    time.sleep(ti)"""
        #else:
        #    time.sleep(ti)
    
    vid.release()
    cv2.destroyAllWindows()
if __name__ == "__main__":
    main()
