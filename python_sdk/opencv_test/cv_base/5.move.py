import cv2 as cv
import numpy as np

#cv.warpAffine(img,M.dsize)
#img 输入图像
#M 2*3移动矩阵
#dsize 输出图像大小

#1.读取
img=cv.imread('img1.png')

#2.图像平移
rows,cols=img.shape[:2]
#平移矩阵
M=np.float32([[1,0,100],[0,1,50]])
dst=cv.warpAffine(img,M,(cols,rows))

#3.图像显示
cv.imshow('img',dst)
cv.waitKey(0)