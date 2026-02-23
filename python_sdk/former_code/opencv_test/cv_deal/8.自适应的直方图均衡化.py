import cv2 as cv
#将图像分成很多小块 然后均衡化 组后拼接
cv.createCLAHE(clipLimit,titleGridSize)
#clipLimit 对比度限制 默认值为0
#titleGridSize 分块的大小 默认为8*8
