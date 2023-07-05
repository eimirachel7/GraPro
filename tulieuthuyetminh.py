import cv2
import matplotlib.pyplot as plt
import numpy as np


# fig = plt.figure(figsize=(16, 9)) # Tạo vùng vẽ tỷ lệ 16:9
# ax1, ax2 = fig.subplots(1, 2) # Tạo 6 vùng vẽ con

# anh1 = cv2.imread('hustlogo.png')
# ax1.imshow(anh1, cmap='gray')
# ax1.set_title("Ảnh máy đọc BGR")

# anh2 = cv2.cvtColor(anh1, cv.COLOR_BGR2RGB)
# ax2.imshow(anh2, cmap='gray')
# ax2.set_title("Ảnh thật RGB")

# plt.show()

img_real = cv2.imread("shapes.jpg", cv2.IMREAD_COLOR)
img = cv2.imread("shapes.jpg", cv2.IMREAD_GRAYSCALE)
gray = cv2.imread("shapes.jpg", cv2.IMREAD_GRAYSCALE)




#_, threshold = cv2.threshold(img, 240, 255, cv2.THRESH_MASK)
borders = cv2.Canny(img, 100, 160, L2gradient=False)
#contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
kernel = np.ones((2,2), np.uint8)
dilation = cv2.dilate(borders, kernel, iterations=1)
img_floodfill = dilation.copy()
h,w =dilation.shape[:2]
mask  = np.zeros((h+2, w+2), np.uint8)
cv2.floodFill(img_floodfill, mask, (0,0), 255);
img_floodfill_inverse = cv2.bitwise_not(img_floodfill)
img_out = dilation|img_floodfill_inverse
thres = img_out

contours, _ = cv2.findContours(thres.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
font = cv2.FONT_HERSHEY_COMPLEX

for c in contours: 
    approx = cv2.approxPolyDP(c, 0.01*cv2.arcLength(c, True), True)
    cv2.drawContours(img, [approx], 0, (0, 255, 255), 5)
    # x = approx.ravel()[0]
    # y = approx.ravel()[1]

    # if len(approx) == 3:
    #     cv2.putText(img, "Triangle", (x+2, y-2), font, 1, (0))
    # elif len(approx) == 4:
    #     cv2.putText(img, "Rectangle", (x+2, y-2), font, 1, (0))
    # elif len(approx) >=15:
    #     cv2.putText(img, "Circle", (x+2, y-2), font, 1, (0))
    # else:
    #     cv2.putText(img, "undifined", (x+5,y-2), font, 1, (0))


cv2.imshow("shape", img)
cv2.imshow("gray", gray)
cv2.imshow("shapereal", img_real)
cv2.imshow("borders", borders)
cv2.imshow("thres", thres)
cv2.waitKey(0)
cv2.destroyAllWindows()
