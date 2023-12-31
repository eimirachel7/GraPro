import numpy as np
import cv2
from math import *
from decimal import *

global mat 
mat = np.matrix

CM_TO_PIXEL = 32 / 640 #32 la so do thuc te cua frame

global d1, a2, a3
d1 = 7
a2 = 12
a3 = 12

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
#-----------------chuyen he truc toa do-----------------------------------------------------
def coor_move():
    global coord_base_frame, homgen_0_c

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
    return coord_base_frame, homgen_0_c

#-------------------------------------------------------------------------------------------
#-----------------chia xung-----------------------------------------------------------------
def pulse_convert(t1,t2,t3):
    t11 = np.rad2deg(t1)
    t21 = np.rad2deg(t2)
    t31 = np.rad2deg(t3)
    p1 = t11 * (500/360)
    p2 = t21 * (500/360)
    p3 = t31 * (500/360)

    p1 = abs(p1)
    p2 = abs(p2)
    p3 = abs(p3)
    
    pulse = np.array([p1, p2, p3])
    return pulse

#-------------------------------------------------------------------------------------------

lower = {'red':([166, 84, 141]), 'blue':([97, 100, 117]),'yellow':([23, 59, 119]), 'orange':([0, 50, 80]), 'purple':([130, 80, 80])} #assign new item lower['blue'] = (93, 10, 0)
upper = {'red':([186,255,255]),  'blue':([117,255,255]), 'yellow':([54,255,255]), 'orange':([20,255,255]), 'purple':([150, 255, 255])}

colors = {'red':(0,0,255), 'blue':(255,0,0), 'yellow':(0, 255, 217), 'orange':(0,140,255), 'purple':(211,0,148)}

cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

cap.set(10,250) #brightness
cap.set(11,50) #contrast
cap.set(12,100) #saturation

while True:
    _, frame = cap.read()
    frame = cv2.flip(frame, 1)
    roi = frame[0:480, 140:480]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(roi, (11,11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mlist = []
    clist = []
    ks = []

    for (key, value) in upper.items():
        kernel = np.ones((2,2),np.uint8)
        mask = cv2.inRange(hsv,np.array(lower[key]),np.array(upper[key]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.dilate(mask, kernel, iterations=1)
        mlist.append(mask)
        cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(cnts)>=1:
            clist.append(cnts[-1])
            ks.append(key)

    #print(ks)
    
    for i,cnt in enumerate(clist):
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
        x = approx.ravel()[0]
        y = approx.ravel()[1]

        if area > 1000:
            #cv2.drawContours(frame, [approx], 0, (0,0,0), 2)
            x,y,w,h = cv2.boundingRect(cnt)
            # cv2.rectangle(roi, (x,y), (x+w, y+h), (0,255,50), 2)
            
            x2 = x + int(w/2)
            y2 = y + int(h/2)

            x2_cm = x2*CM_TO_PIXEL
            y2_cm = y2*CM_TO_PIXEL
            cam_ref_coord = np.array([[x2_cm],
                                [y2_cm],
                                [0.0],
                                [1]])
            coor_move()
            coord_base_frame = homgen_0_c @ cam_ref_coord
            #time.sleep(0.1)
            text1 = "x: " + str(x2_cm) + "cm, y: " + str(y2_cm) + "cm"
            text2 = "x: " + str(coord_base_frame[0][0]) + "cm" + ", y: " + str(coord_base_frame[1][0]) + "cm"

            #cv2.circle(frame, (x2,y2), 4, (0,255,255), 1)
            #cv2.putText(frame, text2, (x2-10,y2-10),cv2.FONT_ITALIC,0.5, (255,50,100),2)
        
            if len(approx) == 3:
                cv2.rectangle(roi, (x,y), (x+w, y+h), (0,255,50), 2)
                cv2.putText(roi, ks[i] + " Triangle", (x-10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 1, colors[ks[i]], 2)
                cv2.circle(roi, (x2,y2), 4, (0,255,255), 1)
                cv2.putText(roi, text2, (x2-10,y2-10),cv2.FONT_ITALIC,0.5, (255,50,100),2)
                
            elif len(approx) >= 4 and len(approx) <= 6:
                cv2.rectangle(roi, (x,y), (x+w, y+h), (0,255,50), 2)
                cv2.putText(roi, ks[i] + " Rectangle", (x-10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 1, colors[ks[i]], 2)
                cv2.circle(roi, (x2,y2), 4, (0,255,255), 1)
                cv2.putText(roi, text2, (x2-10,y2-10),cv2.FONT_ITALIC,0.5, (255,50,100),2)
            
            elif len(approx) >= 10 and len(approx) <= 20:
                cv2.rectangle(roi, (x,y), (x+w, y+h), (0,255,50), 2)
                cv2.putText(roi, ks[i] + " Circle", (x-10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 1, colors[ks[i]], 2)
                cv2.circle(roi, (x2,y2), 4, (0,255,255), 1)
                cv2.putText(roi, text2, (x2-10,y2-10),cv2.FONT_ITALIC,0.5, (255,50,100),2)
        
            tg = inv_Kine(coord_base_frame[0][0], coord_base_frame[1][0], 3)
            print(tg)

            #ps = pulse_convert(tg[0], tg[1], tg[2])
            #print(ps)

    cv2.imshow("frame", frame)
    cv2.imshow("hsv", hsv)
    cv2.imshow("RoI", roi)
    if cv2.waitKey(1) == 27:
        break
cap.release()
cv2.destroyAllWindows()