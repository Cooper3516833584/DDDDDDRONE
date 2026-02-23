import cv2 as cv
#cv.resize(src,dsize,fx=0,fy=0,interpolation=cv.INTER_LINEAR)
#src 输入图像
#dsize 绝对尺寸
#fx fy 相对尺寸
#interpolation 插值方法

#1.读取图像
img=cv.imread('D:\Python\opencv_test\cv_deal\calender.png')
#2.图像缩放
#2.1绝对尺寸
rows,cols=img.shape[:2]
res=cv.resize(img,(2*cols,2*rows),interpolation=cv.INTER_CUBIC)

#2.2相对尺寸
res1=cv.resize(img,None,fx=0.5,fy=0.5)
cv.imwrite('D:\Python\opencv_test\cv_deal\calender.png',res1)
#3.图像显示
cv.imshow('img',img)
cv.waitKey(0)
cv.imshow('res',res)
cv.waitKey(0)
cv.imshow('res1',res1)
cv.waitKey(0)
