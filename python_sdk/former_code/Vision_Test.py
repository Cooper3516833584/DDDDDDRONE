from SolutionsNew.Vision_Net import FastestDetOnnx
import cv2,time

def check_yellow():
    cap = cv2.VideoCapture(0)
    deep = FastestDetOnnx(drawOutput=True)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    while True:
        image = cap.read()[1]
        if image is None:
            continue
        cv2.imshow("origin",image) # 调试时使用
        get = deep.detect(image)
        cv2.imshow("result",image) # 调试时使用
        # width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        # height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        if len(get) > 0:#and get[0][0][0] > 540 and get[0][0][0] < 740 and get[0][0][1] > 300 and get[0][0][1] < 420:
            x = get[0][0][0]/width
            y = get[0][0][1]/height
            if x > 0.3 and x < 0.7 and y > 0.1 and y < 0.6:
                time.sleep(0.1)
                cv2.imwrite("Yellow_1.jpg", image)
                time.sleep(0.1)
                cv2.imwrite("Yellow_2.jpg", image)
                time.sleep(0.1)
                cv2.imwrite("Yellow_3.jpg", image)
                cap.release()
                break
        k = cv2.waitKey(1) & 0xFF   
        if k == ord("q"):
            break
check_yellow()            