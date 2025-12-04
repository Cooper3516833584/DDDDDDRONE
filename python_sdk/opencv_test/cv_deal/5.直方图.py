import cv2 as cv
#灰度直方图

#cv.calcHist(img,channels,mask,histSize,ranges[,hist[,accumulate]])
#img 原图像用中括号括起来 [img]
#channels 通道 如果是灰度图，值为0 如果是彩色图，参数可以是0，1，2 [channels]
#mask 掩膜图像 要统计完整直方图就设置为None 若只想统计一部分，就要制作一个掩膜图像
#histSize BIN的数目，而应该用中括号括起来 如[256]
#ranges 像素值范围 通常为[0,256]
