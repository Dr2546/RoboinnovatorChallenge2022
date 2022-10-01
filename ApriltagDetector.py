import numpy as np
import cv2
import cv2.aruco as aruco

def main():

    vid = cv2.VideoCapture(1, cv2.CAP_DSHOW)

    while True: 
        ret, image = vid.read()
        id = TagDetector(image)
        print(id)
        

    #    imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #    ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    #    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #    for cnt in contours:
    #        x,y,w,h = cv2.boundingRect(cnt)
    #        cropped = image[y:y+h, x:x+w]
    #        corners, ids, rejectedImgPoints = aruco.detectMarkers(cropped, aruco_dict, parameters=parameters)
    #        aruco.drawDetectedMarkers(cropped, corners, ids)

    #    cv2.imshow("test", image)
        cv2.imshow("img",image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    vid.release()
    cv2.destroyAllWindows()

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

if __name__ == "__main__":
    main()