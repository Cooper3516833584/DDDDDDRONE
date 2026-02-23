import cv2 as cv
#掩膜：
#提取感兴趣部分 遮挡部分不被处理
#屏蔽作用
#结构特征提取
#特殊形状图像制作

#创建蒙版
mask=np.zeros(img.shape[:2],np,uint8)
mask[400:650,200:500]=255
#掩膜

masked_img=cv.bitwise_and(img,img,mask=mask)

#统计掩膜后的灰度图
cv.calcHist([img],[0],mask,[256],[1,256])
