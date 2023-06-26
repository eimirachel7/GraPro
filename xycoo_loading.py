import cv2
import numpy as np 
from numpy import linalg
import cmath
from math import *
import cvzone as cvz

global mat
mat = np.matrix

global d1, a2, a3 # so do cm
d1 = 7 
a2 = 12
a3 = 12

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

#480p resolution
cap.set(3, 640)
cap.set(4, 480)
#cap.set(10,50) #brightness
#cap.set(11,50) #contrast
#cap.set(12,100) #saturation

#-----------------chuyen he truc toa do-----------------------------------------------------
CM_TO_PIXEL = 32 / 640 #32 la so do thuc te cua frame

#def matrix_move ():

rot_angle = 180
rot_angle = np.deg2rad(rot_angle)
rot_mat_0_c = np.array([[1, 0, 0],
                            [0, np.cos(rot_angle), -np.sin(rot_angle)],
                            [0, np.sin(rot_angle), np.cos(rot_angle)]])

disp_vec_0_c = np.array([[-1.8],[24.4],[0.0]]) #khoang cach x,y,z giua 2 goc toa do

extra_row_homgen = np.array([[0,0,0,1]])

homgen_0_c = np.concatenate((rot_mat_0_c,disp_vec_0_c), axis=1)
homgen_0_c = np.concatenate((homgen_0_c, extra_row_homgen), axis=0)

coord_base_frame = np.array([[0.0],
                                [0.0],
                                [0.0],
                                [1]])
    #return homgen_0_c, coord_base_frame
#------------------------------------------------------------------------------------------

#-------------------dong hoc nguoc---------------------------------------------------------
def inv_Kine(xE, yE, zE):
    theta1 = atan2(xE,yE)
    c1 = cos(theta1)
    s1 = sin(theta1)

    c3 = (pow((xE/c1),2)+pow((zE-d1),2)-pow(a3,2)-pow(a2,2))/(2*a3*a2)
    if (c3 <=1 and c3 >=-1):
        s3 = -sqrt(1-pow(c3,2))
        theta3 = atan2(s3,c3)

        c2 = (a2*(xE/c1)+a3*(c3*(xE/c1)+s3*(zE-d1)))/(pow((xE/c1),2)+pow((zE-d1),2))
        s2 = (a2*(zE-d1)+a3*(c3*(zE-d1)-s3*(xE/c1)))/(pow((xE/c1),2)+pow((zE-d1),2))
        theta2 = atan2(s2,c2)

        theta = np.array([theta1, theta2, theta3])

        return theta
    else:
        pass
#-------------------------------------------------------------------------------------------

while True:
    _, frame = cap.read()
    frame = cv2. flip(frame, 1)
    roi = frame[0:480, 140:480]
    
    #object_detector = cv2.createBackgroundSubtractorMOG2(history=100,varThreshold=40)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #mask = object_detector.apply(roi)
    
    _, threshold = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    
    kernel = np.ones((5,5),np.uint8)
    cv2.dilate(threshold, kernel, iterations=1)
    threshold = cv2.GaussianBlur(threshold, (15,15), 0)
    contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area > 1000:
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(frame, (x,y), (x+w,y+h), (250,0,100), 2)

            x2 = x + int(w/2)
            y2 = y + int(h/2)
            
            cv2.circle(roi, (x2,y2), 4, (0,255,255), 1)
            
            x2_cm = x2*CM_TO_PIXEL
            y2_cm = y2*CM_TO_PIXEL
            cam_ref_coord = np.array([[x2_cm],
                                [y2_cm],
                                [0.0],
                                [1]])
            #MT = np.arrmatrix_move()
            coord_base_frame = homgen_0_c @ cam_ref_coord
            #MT[2] = MT[1] @ cam_ref_coord

            text1 = "x: " + str(x2_cm) + "cm, y: " + str(y2_cm) + "cm"
            text2 = "x: " + str(coord_base_frame[0][0]) + ", y: " + str(coord_base_frame[1][0])
            cv2.putText(frame, text2, (x2-10, y2-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            tg = inv_Kine(coord_base_frame[0][0], coord_base_frame[1][0], 3)
            print(tg)
            



    cv2.imshow("frame", frame)
    #cv2.imshow("RoI", roi)
    #cv2.imshow("threshold", threshold)
    #cv2.imshow("Gray", gray)
    if cv2.waitKey(1) == 27:
        break
cap.release()
cv2.destroyAllWindows()