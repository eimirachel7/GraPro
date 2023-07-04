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

img = cv2.imread("shapes.jpg", cv2.IMREAD_COLOR)


cv2.imshow("shape", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
