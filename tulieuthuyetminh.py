import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np


fig = plt.figure(figsize=(16, 9)) # Tạo vùng vẽ tỷ lệ 16:9
ax1, ax2 = fig.subplots(1, 2) # Tạo 6 vùng vẽ con

dub = cv.imread('hustlogo.png')
ax1.imshow(dub, cmap='gray')
ax1.set_title("Ảnh máy đọc BGR")

dub_real = cv.cvtColor(dub, cv.COLOR_BGR2RGB)
ax2.imshow(dub_real, cmap='gray')
ax2.set_title("Ảnh thật RGB")

plt.show()
