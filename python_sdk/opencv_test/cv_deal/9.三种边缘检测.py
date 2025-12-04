import cv2 as cv
"""
#Sobel算子
#标注图像中亮度变化明显的点
#比较简单，实际应用中必canny边缘检测效率高 但是边缘不如Canny检测的准
#抗噪声能力强

Sobel_x_or_y=cv.Sobel(src,ddepth,dx,dy,dst,ksize,scale,delta,borderType)
#src
#ddepth 图像的深度
#dx和dy 指求导的阶数 取值为0 1
#ksize Sobel算子的大小，必须为级数1 3 5 7 默认为3 
    #如果ksize=-1 就演变成3*3的Scharr算子
#scale 缩放比例常数
#borderType 图像边界的模式，默认值为cv2.BORDER_DEFAULT

#求导结果可能大于255或小于0 会有截断
#格式转换
Sobel_abs=cv.convertScaleAbs(x)
#图像混合
result=cv.addWerighted(src1,alpha,src2,beta)
----------------------
#Scharr算子

#Laplacian算子
laplacian=cv.Laplacian(src,ddepth[,ksize[,scale[,delta[,borderType]]]]])
#src
#ddepth 图像深度 -1表示与原图像相同的深度 目标图像深度必须大于原图像深度
ksize 算子大小 1 3 5 7
"""
#读取
img=cv.imread('123.jpg',0)
#计算Sobel卷积
x=cv.Sobel(img,cv.CV_64F,1,0)
y=x=cv.Sobel(img,cv.CV_64F,0,1)
#将数据进行转换
Scale_absX=cv.convertScaleAbs(x)
Scale_absY=cv.convertScaleAbs(y)
#结果混合
result=cv.addWeighted(Scale_absX,0.5,Scale_absY,0.5,0)
#图像显示
cv.imshow("Scale_absX",Scale_absX)
cv.imshow("Scale_absY",Scale_absY)
cv.imshow("result",result)
cv.waitKey(0)