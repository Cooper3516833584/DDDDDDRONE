import cv2 as cv

cv.morphologEx(img,op,kernel)
#img
#op 处理方式 开：cv.MORPH_OPEN  闭： cv.MORPH_CLOSE
#Kernel 核结构


# #开运算
#先腐蚀后膨胀--消除噪点
#闭运算
#先膨胀后腐蚀--消除孔洞
