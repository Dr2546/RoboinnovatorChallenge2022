
from smbus import SMBus
import time
import RPi.GPIO as GPIO
import ApriltagDetector
import numpy as np
import cv2
import cv2.aruco as aruco
import pigpio

addrn = 0x10 # bus address
bus = SMBus(1) # indicates /dev/ic2-1
 
servo = 18

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(20, GPIO.IN)
GPIO.setup(27, GPIO.IN)

pwm = pigpio.pi()
pwm.set_mode(servo, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo, 50)

pwm.set_servo_pulsewidth(servo, 1200)
time.sleep(0.5)

numb = 1
num = 1

def TagDetector(image):

    id = []

    aruco_dict = aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)
    parameters = aruco.DetectorParameters_create()

    imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        x,y,w,h = cv2.boundingRect(cnt)
        cropped = image[y:y+h, x:x+w]
        corners, ids, rejectedImgPoints = aruco.detectMarkers(cropped, aruco_dict, parameters=parameters)
        aruco.drawDetectedMarkers(cropped, corners, ids)
        if ids is not None:
            for id_num in ids:
                if id_num[0] not in id:
                    id.append(id_num[0])

    return id



def main():
    vid = cv2.VideoCapture(0)
    supply = []
    key_tag = 576
    final_tag = 20 #not this ,change in future
    on = False
    finish = False

    while on == False:
        _,img = vid.read()
        k = TagDetector(img)
        if len(k)>0 and k[0] == key_tag:
            on = True

    pwm.set_servo_pulsewidth(servo, 600)
    time.sleep(0.5)

    bus.write_byte(addrn, 0x80)

    #start scanning supply sequence
    while len(supply) != 5:
        _, supply_tag = vid.read()
        tag_list = TagDetector(supply_tag)
        if len(tag_list)>0 :
            tag = tag_list[0]
            if tag not in supply and tag != key_tag:
                supply.append(tag)

    pwm.set_servo_pulsewidth(servo, 1200)
    time.sleep(0.5)

    while not finish:
        #start moving
        
        GPIO.output(16, GPIO.LOW)
        GPIO.output(12, GPIO.LOW)

        #bus.write_byte(addr, 0xC0)
        #time.sleep(5)

        while True:

            #reading tag all the time
            _, tag = vid.read()
            element = TagDetector(tag)

            #found the tower
            if len(element)>0 and element[0] in supply:
                index = supply.index(element[0])+1
                
                #tell pi3 to start transport
                GPIO.output(21, GPIO.HIGH)

                #wait for transporting
                while GPIO.input(20) == 1:
                    pass

                bus.write_byte(addrn, index)

                while GPIO.input(27) == 1:
                    pass
                GPIO.output(21, GPIO.LOW)

                #finish transport
                supply[index-1] = -1

            #if somehow still survive till the end
            if len(element)>0 and element[0] == final_tag:
                #go to start point and stop
                #bus.write_byte(addr, 0xC0)
                #time.sleep(3)
                #bus.write_byte(addr, 0x84)
                #time.sleep(5)
                #bus.write_byte(addr, 0xC0)
                #time.sleep(3)
                GPIO.output(16, GPIO.HIGH)
                finish = True

            #if want to rerun
            if len(element)>0 and element[0] == key_tag:
                GPIO.output(12, GPIO.HIGH)
                break



        #if(num == 1):
        #    GPIO.output(21, GPIO.HIGH)
        #    bus.write_byte(addr, 0x01)
        #    num = 0
        #else:
        #    GPIO.output(21, GPIO.LOW)
        #    num = 1

        
        #if status != status_now:
        #    status = status_now
        #    if(status == -1):
        #        GPIO.output(21, GPIO.HIGH)
        #        bus.write_byte(addr, 0x01)
        #    elif(status == 1):
        #        GPIO.output(21, GPIO.HIGH)
        #        bus.write_byte(addr, 0x0)
        #        #walk
        
        #_, img = vid.read()
        #id = TagDetector(img)
        #if(12 in id):
        #    status_now = status * -1
        #    time.sleep(2)
            
        
        '''
        cv2.imshow("img",img)
        print(status)
        print(id)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break'''
        #time.sleep(10)
        #	t = bus.read_byte(addr)
        
    vid.release
    #cv2.destroyAllWindow()
    
if __name__ == "__main__":
    main()