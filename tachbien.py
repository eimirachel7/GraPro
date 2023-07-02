import cv2
import numpy as np
import imutils

cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()
    # frame = np.empty((800*1024*3), dtype=np.uint8)
    # frame = frame.reshape((800,1024,3))
    # resized = imutils.resize(frame, width=1000)
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    filter = cv2.medianBlur(gray, 3)

    #_, threshold = cv2.threshold(filter, 100, 255, cv2.THRESH_BINARY)
    borders = cv2.Canny(filter, 100, 160, L2gradient=False)
    
    kernel = np.ones((2,2), np.uint8)
    dilation =cv2.dilate(borders, kernel, iterations=1) #lam day bien
    img_floodfill = dilation.copy()
    h,w =dilation.shape[:2]
    mask  = np.zeros((h+2, w+2), np.uint8)
    cv2.floodFill(img_floodfill, mask, (0,0), 255);
    img_floodfill_inverse = cv2.bitwise_not(img_floodfill)
    img_out = dilation|img_floodfill_inverse
    thres = img_out

    contours, hierarchy = cv2.findContours(thres.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #contours = contours[1]

    for c in contours:
        area = cv2.contourArea(c)
        if area > 500:
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(frame, (x,y), (x+w, y+h), (250,150,0),2)

    cv2.imshow("frame", frame)
    cv2.imshow("floodfill", img_out)
    cv2.imshow("dilation", dilation)
    if cv2.waitKey(1) == 27:
        break
cap.release()
cv2.destroyAllWindows()