from tkinter import *
import tkinter.filedialog as tkf
import cv2
import numpy as np
import math
import serial
import time
from PIL import Image, ImageTk

DataSerial = serial.Serial('COM3', 115200)

time.sleep(1)

hmi = Tk()
hmi.title("PROJECT: SORTING OBJECT USING ROBOT ARM")
hmi.geometry("1000x500")

label = Label(hmi, text="MÀN HÌNH ĐIỀU KHIỂN", font=("Montserrat", 20))
label.pack()


def phan_loai_hinh():
    cap = cv2.VideoCapture(1)

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


def phan_loai_mau():
    #label.configure(text="PHÂN LOẠI MÀU SẮC", font=("Montserrat", 20))
    cap = cv2.VideoCapture(1)
    
    while True:

        ret, frame = cap.read()
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
        #if (lum_value > 100 and lum_value < 125):
        if (sat_value > 156 and sat_value < 240):
                #color = "Undefined"
            if (hue_value < 20 or hue_value > 230):
                color = "RED"
                DataSerial.write('D\r'.encode())
                time.sleep(1)
            elif ((hue_value) < 50 and (hue_value > 30)):
                color = "YELLOW"
                DataSerial.write('Y\r'.encode())
                time.sleep(1)
            elif (hue_value < 131 and hue_value > 80):
                color = "BLUE"
                DataSerial.write('X\r'.encode())
                time.sleep(1)
            else:
                color = "NONE"
                continue
                

        pixel_center_bgr = frame[cy,cx]
        b, g, r = int(pixel_center_bgr[0]), int(pixel_center[1]), int(pixel_center[2])

        cv2.rectangle(frame, (cx-220, 100), (cx + 200, 120), (255, 255, 255), -1)
        cv2.putText(frame, color, (cx - 200, 100), 0, 3, (b, g, r), 5)
        cv2.circle(frame, (cx, cy), 20, (25,25,25), 3)
        
        cv2.imshow("Frame", frame)
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


but1 = Button(hmi, text="PHÂN LOẠI HÌNH DÁNG", height=2, width=20, font=("Montserrat", 12), command=phan_loai_hinh)
but1.place(x=100, y=100)

but2 = Button(hmi, text="PHÂN LOẠI MÀU SẮC", height=2, width=20, font=("Montserrat", 12), command=phan_loai_mau)
but2.place(x=100, y=300)


hmi.mainloop()
