"""
test_camera_fov.py
测量下视摄像头实际取景长宽
定点起飞至150cm → 悬停 → 拍照 → 定点降落

运行前提：
  - 上位机已自启动 server_ros.py（FC_Server + ROS 建图链路已运行）
  - 下视 USB 摄像头已连接在 /dev/video0
  - 本脚本通过 FC_Client 连接 FC_Server，不直接抢占飞控串口

照片保存在 /home/fc/桌面/DDDDDrone_Cloned 下。
通过照片中地面覆盖区域和已知高度(150cm)可推算摄像头实际视场角(FOV)。
"""
import os
import time
import cv2
from datetime import datetime
from loguru import logger
from FlightController import FC_Client
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Solutions.Navigation import Navigation
from FlightController.Components.RosMapper import RosMapper
from FlightController.Components.RosNode import RosNodeRunner

# ============ 可调参数 ============
CRUISE_HEIGHT = 150          # 拍照高度 cm
CAMERA_INDEX = 0             # 下视 USB 摄像头索引 (/dev/video0)
SAVE_DIR = "/home/fc/桌面/DDDDDrone_Cloned"
CART_TIMEOUT = 30.0          # Cartographer TF 初始化超时 / s
# =================================


def main():
    # ---- 1. 连接飞控 (FC_Client) ----
    # server_ros.py 已启动 FC_Server 并持有飞控串口，
    # 本脚本通过 FC_Client 网络连接，不直接打开串口。
    logger.info("[FOV] Connecting to FC_Server...")
    fc = FC_Client()
    fc.connect()
    time.sleep(0.5)
    logger.info("[FOV] FC_Client connected")

    # ---- 2. 初始化传感器 Python 包装层 (ROS 模式) ----
    # ROS 驱动节点 (ldlidar, realsense, cartographer, tf2_ros) 由
    # server_ros.py 管理；此处仅创建 Python 端订阅者。
    t265 = T265("ros")
    t265.start()

    radar = LD_Radar()
    radar.start("ros")

    # ---- 3. 初始化桥梁层 ----
    mapper = RosMapper()

    # ---- 4. 初始化导航层 ----
    navi = Navigation(
        fc=fc,
        rs=t265,
        radar=radar,
        mapper=mapper,
    )

    # ---- 5. 启动 ROS Python 节点执行器 ----
    RosNodeRunner().add_nodes().run()

    # ---- 6. 启动导航 (fusion-ros 模式) ----
    navi.start()
    navi.switch_navigation_mode("fusion-ros")
    logger.info("[FOV] Navigation started (fusion-ros)")

    # ---- 7. 定点起飞至 150cm ----
    # pointing_takeoff 内部处理: PROGRAM_MODE → 解锁 → 一键起飞 →
    # 等待悬停 → HOLD_POS_MODE → 闭环位置保持 → 爬升至目标高度
    logger.info(f"[FOV] Taking off to {CRUISE_HEIGHT}cm")
    navi.pointing_takeoff((0, 0), CRUISE_HEIGHT)
    navi.set_yaw(0)
    navi.wait_for_yaw()
    time.sleep(0.5)
    logger.info(f"[FOV] Takeoff complete, hovering at {CRUISE_HEIGHT}cm")

    # ---- 8. 等待 Cartographer TF 建立 ----
    logger.info(f"[FOV] Waiting for Cartographer TF (timeout={CART_TIMEOUT}s)...")
    t0 = time.perf_counter()
    while True:
        time.sleep(1)
        pt = navi.current_point
        logger.info(f"[FOV] current_point: ({pt[0]:.1f}, {pt[1]:.1f})")
        if pt[0] != 0 or pt[1] != 0:
            break
        if time.perf_counter() - t0 > CART_TIMEOUT:
            raise RuntimeError(
                f"Cartographer TF not established within {CART_TIMEOUT}s"
            )
    logger.info(f"[FOV] Cartographer TF established ({time.perf_counter() - t0:.1f}s)")

    # ---- 9. 悬停稳定后拍照 ----
    logger.info("[FOV] Stabilizing before capture...")
    time.sleep(3.0)

    logger.info(f"[FOV] Opening camera /dev/video{CAMERA_INDEX}")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"Camera open failed: /dev/video{CAMERA_INDEX}")

    # 获取实际分辨率
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    logger.info(f"[FOV] Camera resolution: {actual_w}x{actual_h}")

    # 预热并抓取一帧
    for _ in range(5):
        cap.read()
        time.sleep(0.05)

    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        raise RuntimeError("Camera read failed")

    # ---- 10. 保存照片 ----
    os.makedirs(SAVE_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"fov_{CRUISE_HEIGHT}cm_{actual_w}x{actual_h}_{timestamp}.jpg"
    save_path = os.path.join(SAVE_DIR, filename)
    cv2.imwrite(save_path, frame)
    logger.info(f"[FOV] Photo saved: {save_path}")
    logger.info(
        f"[FOV] Image: {frame.shape[1]}x{frame.shape[0]} px, "
        f"height={CRUISE_HEIGHT}cm"
    )

    # ---- 11. 定点降落 ----
    logger.info("[FOV] Landing...")
    navi.stop_move()
    navi.set_navigation_state(False)
    navi.set_keep_height_state(False)
    fc.set_flight_mode(fc.PROGRAM_MODE)
    time.sleep(0.1)
    fc.stablize()
    fc.land()
    if not fc.wait_for_lock():
        logger.warning("[FOV] Auto lock timeout, forcing lock")
        fc.lock()
    logger.info("[FOV] Landed and locked")

    # ---- 12. 清理 ----
    fc.close()
    logger.info("[FOV] Done")


if __name__ == "__main__":
    main()
