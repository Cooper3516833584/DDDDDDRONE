import cv2 as cv
import numpy as np
"""
#2.1霍夫变换原理

#2.2霍夫线检测
cv.HoughLines(img,rho,theta,threshold)
#img
#rho theta ρ和 的精度
#threshold 阈值只有累加器中高于阈值时才被认为是直线

"""

img=cv.imread('blocks.png')
gray=cv.cvtColor(img,cv.COLOR_BGR2GRAY)
edges=cv.Canny(gray,50,150)

cv.imshow("gray",edges)
cv.waitKey(0)

#霍夫直线变换
lines=cv.HoughLines(edges,0.8,np.pi/180,150)
#print(lines)
#将检测的线绘制在图像上 注意是极坐标
for line in lines:
    rho,theta=line[0]
    a=np.cos(theta)
    b=np.sin(theta)
    x0=a*rho
    y0=b*rho
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))
    cv.line(img,(x1,y1),(x2,y2),(0,255,0))
#图像显示
cv.imshow("img",img)
cv.waitKey(0)

"""
#霍夫圆检测
#原理
circles=cv.HoughCircles(img,method,dp,minDist,param1=100,param2=100，minRadius=0,maxRadius=100)
#img
#method 使用霍夫变换的算法 参数是 HOUGH_GRADIENT
#dp 霍夫空间的分辨率 dp=1时表示霍夫空间与输入图像空间的大小一致 dp=2时霍夫空间是输入图像空间的一半。。。
#minDist 为圆心之间的最小距离 如果检测到两个圆心之间的距离小于该值 则认为它们是同一个圆心
#param1 边缘检测时Canny算子的最高阈值 低阈值是高阈值的一半
#param2 检测圆心和确定半径时所共有的阈值
minRadius和maxRadius为所检测到园半径的最小值和最大值
"""
img=cv.imread('circles.png')
gray=cv.cvtColor(img,cv.COLOR_BGR2GRAY)
img=cv.medianBlur(gray,7)
cv.imshow("img",img)
cv.waitKey(0)
circles=cv.HoughCircles(img,cv.HOUGH_GRADIENT,1,200,param1=100,param2=20,minRadius=0,maxRadius=100)
print(circles)
for i in circles[0,:]:
    cv.circle(img,(int(i[0]),int(i[1])),int(i[2]),(0,255,0),2)
    cv.circle(img,(int(i[0]),int(i[1])),int(i[2]), (0, 255, 0), -1)

cv.imshow("img",img)
cv.waitKey(0)