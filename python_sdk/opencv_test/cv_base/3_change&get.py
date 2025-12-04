import cv2 as cv
import numpy as np

#创建 512*512 3通道全黑图
img=np.zeros((256,256,3),np.uint8)
#获取像素点全通道的值
point=img[0,0]

#获取像素点0，0处 0通道的值
point=img[0,0,0]

#修改像素点
img[100,100]=[0,0,255]

#获取图像属性
print(img.shape)
print(img.dtype)
print(img.size)

#通道拆分与合并
#通道拆分
b,g,r=cv.split(img)
#通道合并
img=cv.merge((b,g,r))

#色彩空间的改变
#flag 转换类型 &cv.COLOR_BGR2GRAY: BGR-GRAY &cv.COLOR_BGR2HSV: BGR-HSV
img= cv.imread('123.jpg',0)
gray=cv.cvtColor(img,cv.COLOR_BGR2GRAY)

cv.imshow('img',img)
cv.waitKey(0)