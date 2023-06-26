import cv2
import numpy as np
from math import *

global mat 
mat = np.matrix

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(3,640)
cap.set(4,480)
cap.set(10,0)
cap.set(11,50)
cap.set(12,80)

while True:
    ret, frame = cap.read()
    frame = cv2.flip(frame,1)




    cv2.imshow("frame", frame)
    if cv2.waitKey(1) == 27:
        break
cap.release()
cv2.destroyAllWindows()