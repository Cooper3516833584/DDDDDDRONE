import cv2 as cv

"""
core 核心的数据模块
highgui 视频与图像的读取、显示、存储
imgproc 图像处理的基础方法
features2d 图像特征以及特征匹配
"""
cap=cv.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)
cap.set(10,1000)

while True:
    success,img=cap.read()
    cv.imshow("Video",img)
    if cv.waitKey(1)&0xFF==ord('q'):
        break
