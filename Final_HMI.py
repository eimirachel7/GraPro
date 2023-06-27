from tkinter import *
#import tkinter.filedialog as tkf
import cv2
import numpy as np
#from numpy import linalg
#import cmath
import math
from math import *

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

hmi = Tk()
hmi.title("PROJECT: SORTING OBJECT USING ROBOT ARM")
hmi.geometry("700x500")

label = Label(hmi, text="MÀN HÌNH ĐIỀU KHIỂN", font=("Montserrat", 20))
label.pack()


def phan_loai_hinh():
    cap = cv2.VideoCapture(0)

    while (cap.isOpened()): 
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        roi = frame[0:480, 140:480]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        processed_image = cv2.medianBlur(gray, 3)
        anh_tach_bien = cv2.Canny(processed_image, 30, 140, L2gradient=False)
        kernel = np.ones((2,2), np.uint8)
        dilation = cv2.dilate(anh_tach_bien, kernel, iterations=1)
        im_floodfill = dilation.copy()
        h, w = dilation.shape[:2]
        mask = np.zeros((h+2, w+2), np.uint8)
        cv2.floodFill(im_floodfill, mask, (0,0), 255)
        im_floodFill_inv = cv2.bitwise_not(im_floodfill)
        im_out = dilation|im_floodFill_inv 
        thresh = im_out
        contours, _  = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv2.contourArea(c)
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04*peri, True)
            num_approx = len(approx)

            if area > 5000 and area < 6000:
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(roi, (x,y), (x+w, y+h), (0,255,50), 2)
                if (num_approx == 4):
                    approx1 = approx[0,0,:]
                    approx2 = approx[1,0,:]
                    approx3 = approx[2,0,:]
                    approx4 = approx[3,0,:]

                    canh1 = math.sqrt(math.pow(approx2[0]-approx1[0],2)+math.pow(approx2[1]-approx1[1],2))
                    canh2 = math.sqrt(math.pow(approx3[0]-approx2[0],2)+math.pow(approx3[1]-approx2[1],2))
                    dientichtinh = canh1*canh2
                    dientichreal = cv2.contourArea(c)
                    tilecanh = canh1/canh2

                    if (dientichtinh >= dientichreal*0.95 and dientichtinh <= dientichreal*1.05):
                        if (tilecanh >= 0.95 and tilecanh <= 1.05):
                            cv2.putText(roi, "square", (x,y), 
                                        cv2.FONT_HERSHEY_COMPLEX, 0.7, 
                                        (255, 0, 255), 2 )
                        else: 
                            cv2.putText(roi, "rectangle", (x,y),
                                        cv2.FONT_HERSHEY_COMPLEX,0.7,
                                        (255, 50, 200), 2)
                    else: 
                        cv2.putText(roi,"error", (x,y), 
                                    cv2.FONT_HERSHEY_COMPLEX, 0.7, 
                                    (200, 110, 255), 2)
                        continue
                        
                elif (num_approx == 3):
                    approx1 = approx[0,0,:]
                    approx2 = approx[1,0,:]
                    approx3 = approx[2,0,:]

                    canh1 = math.sqrt(math.pow(approx2[0]-approx1[0],2)+math.pow(approx2[1]-approx1[1],2))
                    canh2 = math.sqrt(math.pow(approx3[0]-approx2[0],2)+math.pow(approx3[1]-approx2[1],2))
                    canh3 = math.sqrt(math.pow(approx1[0]-approx3[0],2)+math.pow(approx1[1]-approx3[1],2))
                    p = (canh1+canh2+canh3)/2
                    dientichtamgiac = math.sqrt(p*(p-canh1)*(p-canh2)*(p-canh3))
                    dientichreal = cv2.contourArea(c)

                    if (dientichtamgiac > dientichreal*0.95 and dientichtamgiac <= dientichreal*1.05):
                        cv2.putText(roi, "triangle", (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.7,(0, 255, 0), 2)
                        
                    else: 
                        cv2.putText(roi,"error", (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.7, (200, 110, 255), 2)
                        continue
                
                elif (10 <= num_approx <= 20 and area > 100000):
                    cv2.putText(roi, "circle", (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 255, 255), 2)
                else: 
                    cv2.putText(roi,"error", (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.7, (200, 110, 255), 2)
                    continue
                        
        cv2.imshow("frame", frame)
        cv2.imshow("RoI", roi)
        if cv2.waitKey(1) == 27:
            break
    cap.release()
    cv2.destroyAllWindows()

def phan_loai_hinh2():

    #calculate angle
    def angle(pt1,pt2,pt0):
        dx1 = pt1[0][0] - pt0[0][0]
        dy1 = pt1[0][1] - pt0[0][1]
        dx2 = pt2[0][0] - pt0[0][0]
        dy2 = pt2[0][1] - pt0[0][1]
        return float((dx1*dx2 + dy1*dy2))/math.sqrt(float((dx1*dx1 + dy1*dy1))*(dx2*dx2 + dy2*dy2) + 1e-10)

    cap = cv2.VideoCapture(0)
    #dictionary of all contours
    contours = {}
    #array of edges of polygon
    approx = []
    #scale of the text
    scale = 2

    while (cap.isOpened()): 
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        roi = frame[0:480, 140:480]
        if ret==True:
            #grayscale
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            #Canny
            canny = cv2.Canny(roi,80,240,3)

            #contours
            contours, hierarchy = cv2.findContours(canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for i in range(0,len(contours)):
                #approximate the contour with accuracy proportional to
                #the contour perimeter
                approx = cv2.approxPolyDP(contours[i],cv2.arcLength(contours[i],True)*0.02,True)

                #Skip small or non-convex objects
                if(abs(cv2.contourArea(contours[i]))<500 or not(cv2.isContourConvex(approx))):
                    continue

                #triangle
                if(len(approx) == 3):
                    x,y,w,h = cv2.boundingRect(contours[i])
                    cv2.rectangle(roi, (x,y), (x+w, y+h), (0,255,50), 2)
                    cv2.putText(roi,'TRIANGLE',(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,(0,255,255),2,cv2.LINE_AA)
                elif(len(approx)>=4 and len(approx)<=6):
                    #nb vertices of a polygonal curve
                    vtc = len(approx)
                    #get cos of all corners
                    cos = []
                    for j in range(2,vtc+1):
                        cos.append(angle(approx[j%vtc],approx[j-2],approx[j-1]))
                    #sort ascending cos
                    cos.sort()
                    #get lowest and highest
                    mincos = cos[0]
                    maxcos = cos[-1]

                    #Use the degrees obtained above and the number of vertices
                    #to determine the shape of the contour
                    x,y,w,h = cv2.boundingRect(contours[i])
                    cv2.rectangle(roi, (x,y), (x+w, y+h), (0,255,50), 2)
                    if(vtc>=4 and vtc<=6):
                        cv2.putText(roi,'RECTANGLE',(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,(0,255,255),2,cv2.LINE_AA)
                else:
                    #detect and label circle
                    area = cv2.contourArea(contours[i])
                    x,y,w,h = cv2.boundingRect(contours[i])
                    cv2.rectangle(roi, (x,y), (x+w, y+h), (0,255,50), 2)
                    radius = w/2
                    if(abs(1 - (float(w)/h))<=2 and abs(1-(area/(math.pi*radius*radius)))<=0.2):
                        cv2.putText(roi,'CIRCLE',(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,(0,255,255),2,cv2.LINE_AA)

        #Display the resulting frame
        cv2.imshow('frame',frame)
        cv2.imshow('canny',canny)
        cv2.imshow("RoI", roi)
        if cv2.waitKey(1) == 27:
            break
    cap.release()
    cv2.destroyAllWindows()

def phan_loai_hinh3():

    cap = cv2. VideoCapture(0, cv2.CAP_DSHOW)
    
    #480p resolution
    cap.set(3,640) 
    cap.set(4,480)
    cap.set(10,0)
    cap.set(11,50)
    cap.set(12,80)

    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        roi = frame[0:480, 140:480]
        img_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, threshold = cv2.threshold(img_gray, 150, 255, 0)
        kernel = np.ones((5,5), np.uint8)
        cv2.dilate(threshold, kernel, iterations=1)
        threshold = cv2.GaussianBlur(threshold, (15,15), 0)
        contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            epsilon = 0.03*cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, epsilon, True)
            #for pt in approx:
            #    cv2.circle(frame, (pt[0][0], pt[0][1]), 5, (255, 0, 0), -1)
            area = cv2.contourArea(c)
            
            if area > 1000:
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(roi, (x,y), (x+w,y+h), (0,255,20), 2)
                x2 = x + int(w/2)
                y2 = y + int(h/2)
                cv2.circle(roi, (x2,y2), 2, (0,0,255), 2)

                x2_cm = x2*CM_TO_PIXEL
                y2_cm = y2*CM_TO_PIXEL
                cam_ref_coord = np.array([[x2_cm],
                                [y2_cm],
                                [0.0],
                                [1]])
                coor_move()
                coord_base_frame = homgen_0_c @ cam_ref_coord

                shape = "undefined"
                if len(approx) == 3:
                    shape = "triangle"
                elif len(approx)>=4 and len(approx)<=6:
                    shape = "Rectangle"
                elif len(approx)>=10 and len(approx)<=30:
                    shape = "Circle"
                else:
                    shape = "Error"
                

                text1 = "x: " + str(x2_cm) + "cm, y: " + str(y2_cm) + "cm"
                text2 = "x: " + str(coord_base_frame[0][0]) + "cm" + ", y: " + str(coord_base_frame[1][0]) + "cm"
                cv2.putText(roi, text2, (x2-10, y2-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.putText(roi, shape, (x, y-5), cv2.FONT_ITALIC, 1, (100, 0, 255), 1, cv2.LINE_AA)

                tg = inv_Kine(coord_base_frame[0][0], coord_base_frame[1][0], 3)
                print(tg)

        cv2.imshow("Frame", frame)
        cv2.imshow("RoI", roi)
        cv2.imshow("threshold", threshold)
        if cv2.waitKey(1)==27:
            break
    cap.release()
    cv2.destroyAllWindows()

def phan_loai_mau():
    
    
    #label.configure(text="PHÂN LOẠI MÀU SẮC", font=("Montserrat", 20))
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(3,640)
    cap.set(4,480)
    cap.set(10,250) #brightness
    cap.set(11,100) #contrast
    cap.set(12,100) #saturation

    img_count = 1
    
    while True:

        _, frame = cap.read()
        frame = cv2.flip(frame, 1)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        height, width, _ = frame.shape
        cx = int(width/2)
        cy = int(height/2)

        pixel_center = hsv_frame[cx,cy]
        hue_value = pixel_center[0]
        sat_value = pixel_center[1]
        lum_value = pixel_center[2]

        color = "Undefined"
        #color = "Undefined"
        if (hue_value < 20 or hue_value > 230):
            color = "RED"
        elif (hue_value < 50):
            color = "YELLOW"
        elif (hue_value < 131 and hue_value > 80):
            color = "BLUE"
        else:
            color = "NONE"            
            #continue


        pixel_center_bgr = frame[cy,cx]
        b, g, r = int(pixel_center_bgr[0]), int(pixel_center[1]), int(pixel_center[2])

        #cv2.rectangle(frame, (cx-220, 100), (cx + 200, 120), (255, 255, 255), -1)
        cv2.putText(frame, color, (cx - 200, 100), 0, 3, (b, g, r), 5)
        cv2.circle(frame, (cx, cy), 10, (25,25,25), 3)
        
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) == 27:
            break
        elif cv2.waitKey(1) == ord('s'):
            cv2.imwrite(f"img\color{img_count}.png", frame)
            img_count += 1 
    cap.release()
    cv2.destroyAllWindows()


but1 = Button(hmi, text="PHÂN LOẠI HÌNH DÁNG", height=2, width=20, 
              font=("Montserrat", 12), command=phan_loai_hinh3)
but1.place(x=90, y=100)

but2 = Button(hmi, text="PHÂN LOẠI MÀU SẮC", height=2, width=20, 
              font=("Montserrat", 12), command=phan_loai_mau)
but2.place(x=90, y=300)


hmi.mainloop()
