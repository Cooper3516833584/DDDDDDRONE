import cv2
import numpy as np

# 打开摄像头
cap = cv2.VideoCapture(0)
while True:
    # 读取摄像头帧
    ret, frame = cap.read()

    # 指定区域的坐标
    x = 100
    y = 100
    width = 50
    height = 50

    # 获取指定区域的颜色属性
    cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), 2)
    roi = frame[y:y + height, x:x + width]

    # 将指定区域转换为HSV颜色空间
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # 计算指定区域的平均颜色
    average_color = cv2.mean(hsv_roi)
    cv2.imshow('Color Detection', frame)

    print("指定区域的颜色属性：")
    print("平均颜色：", average_color)


    # 按下Esc键退出程序
    if cv2.waitKey(1) == 27:
        break

# 释放摄像头和窗口
cap.release()
cv2.destroyAllWindows()

