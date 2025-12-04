import cv2 as cv
import numpy as np
"""
res=cv.matchTemplate(img,template,method)
#img
#Templat 模板
#method
    #平方差匹配CV_TM_SQDIFFTM
    #相关匹配CV_TM_CCORR
    #相关系数匹配CV_TM_CCOEFF
"""
#读取
img=cv.imread('123.jpg')
template=cv.imread('temp.png')
h,w,l=template.shape

#模板匹配
#1.模板匹配
res=cv.matchTemplate(img,template,cv.TM_CCORR)
#2.返回最佳匹配位置
min_val,max_val,min_loc,max_loc=cv.minMaxLoc(res)
top_left=min_loc
#top_left=max_loc
bottom_right=(top_left[0]+w,top_left[1]+h)
cv.rectangle(img,top_left,bottom_right,(0,255,0),2)

#图像显示
cv.imshow("img",img)
cv.waitKey(0)