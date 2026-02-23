import cv2 as cv

"""
canny=cv.Canny(img,threshold1,threshold2)
#img
#threshold1 minval 较小的阈值将间断的边缘连接起来
#threshold2 maxval 较大的阈值检测图像中明显的边缘
"""

#读取
img=cv.imread('img1.png',0)
#
res=cv.Canny(img,0,255)
#图像显示
cv.imshow("res",res)
cv.waitKey(0)