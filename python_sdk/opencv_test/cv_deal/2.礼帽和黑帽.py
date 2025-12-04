import cv2 as cv

cv.morphologEx(img,op,kernel)
#img
#op 处理方式 礼帽：cv.MORPH_TOPHAT  黑帽：cv.MORPH_BLACKHAT
#Kernel 核结构

#礼貌运算
#原图像与开运算之差

#黑貌
#闭运算与原图像之差
