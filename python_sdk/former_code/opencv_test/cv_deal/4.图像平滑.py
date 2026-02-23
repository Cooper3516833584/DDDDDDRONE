import cv2 as cv
#2.图像平滑
#去除高频信息

#2.1均值滤波
#卷积框内所有像素点的平均值代替中心值
#优点：算法简单 速度快
#缺点：去除了许多细节部分
cv.blur(src,ksize,anchor,borderType)
#src 输入图像
#ksize 卷积核大小
#anchor 默认值(-1，-1)表示核中心
#borderType 边界类型

#2.2高斯滤波
#二维高斯分布时高斯滤波器的基础
#给9个值不同权重 将加权平均给中心值
cv.GaussianBlur(src,ksize,sigmaX,sigmay,borderType)
#src
#ksize 卷积核大小
#sigmaX X方向标准差
#sigmaY Y方向标准差默认与X方向相同
#broderType 填充边界类型

#2.3中值滤波
cv.medianBlur(src,ksize)
#用领域内的中值代替中心点
#src
#ksize
