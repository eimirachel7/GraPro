import cv2
import numpy as np

prevX,prevY=-1,-1
def printCoordinate(event, x, y, flags, params):
    global prevX,prevY
    if event==cv2.EVENT_LBUTTONDOWN:
        cv2.circle(img,(x,y),3,(255,255,255),-1)
        strXY='('+str(x)+','+str(y)+')'
        font=cv2.FONT_HERSHEY_PLAIN
        cv2.putText(img,strXY,(x+10,y-10),font,1,(255,255,255))
        if prevX==-1 and prevY==-1:
            prevX,prevY=x,y
        else:
            cv2.line(img,(prevX,prevY),(x,y),(0,0,255),5)
            prevX,prevY=-1,-1
        cv2.imshow("image",img)
img = np.zeros((480,640,4),dtype=np.uint8)
cv2.imshow("image",img)
cv2.setMouseCallback("image",printCoordinate)
cv2.waitKey()
cv2.destroyAllWindows()