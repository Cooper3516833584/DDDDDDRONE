import cv2 as cv
import numpy as np

#1.以灰度图的方式读取
img= cv.imread('img1.png',0)

#2.opencv 显示图像 调用显示图像后 要调用cv.waitKey()给图像绘制留下时间
cv.imshow('grayImg',img)
cv.waitKey(0)

#matplotlib中显示


#3.保存图像
cv.imwrite('img1.png',img)



