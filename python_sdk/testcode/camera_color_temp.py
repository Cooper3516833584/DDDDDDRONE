"""
USB 摄像头实时色温调节工具
使用 OpenCV 滑块实时调节白平衡/色温，预览调节效果
"""

import cv2
import numpy as np


def kelvin_to_rgb(kelvin: int) -> tuple:
    """
    将色温 (开尔文) 转换为 RGB 白点值
    参考: Tanner Helland 的算法
    """
    temp = kelvin / 100.0

    # 红色通道
    if temp <= 66:
        red = 255
    else:
        red = temp - 60
        red = 329.698727446 * (red ** -0.1332047592)
        red = max(0, min(255, red))

    # 绿色通道
    if temp <= 66:
        green = temp
        green = 99.4708025861 * np.log(green) - 161.1195681661
        green = max(0, min(255, green))
    else:
        green = temp - 60
        green = 288.1221695283 * (green ** -0.0755148492)
        green = max(0, min(255, green))

    # 蓝色通道
    if temp >= 66:
        blue = 255
    elif temp <= 19:
        blue = 0
    else:
        blue = temp - 10
        blue = 138.5177312231 * np.log(blue) - 305.0447927307
        blue = max(0, min(255, blue))

    return red, green, blue


def apply_white_balance(img: np.ndarray, kelvin: int, tint: int = 0) -> np.ndarray:
    """
    对图像应用白平衡校正
    kelvin: 目标色温 (1000-40000)
    tint: 色调偏移 (-100 到 100, 0=中性, 负=偏绿, 正=偏品红)
    """
    r, g, b = kelvin_to_rgb(kelvin)

    # 计算各通道增益，以绿色为基准
    gain_r = g / r if r > 0 else 1.0
    gain_b = g / b if b > 0 else 1.0
    gain_g = 1.0

    # 色调调整 (tint): 调整绿色通道
    if tint > 0:  # 偏品红 → 减绿
        gain_g = 1.0 - tint / 150.0
    else:  # 偏绿 → 减红和蓝
        gain_r *= 1.0 + tint / 150.0
        gain_b *= 1.0 + tint / 150.0

    # 转为 float32 处理
    img_f = img.astype(np.float32)

    # 确保是 3 通道 BGR
    if img_f.shape[2] == 4:
        img_f = img_f[:, :, :3]  # 去掉 alpha 通道

    # 使用 numpy 直接操作各通道 (比 cv2.split 更可靠)
    # OpenCV 格式: BGR
    r_ch = img_f[:, :, 2] * gain_r   # R 在索引 2
    g_ch = img_f[:, :, 1] * gain_g   # G 在索引 1
    b_ch = img_f[:, :, 0] * gain_b   # B 在索引 0

    # 合并通道 (BGR)
    result = np.dstack([b_ch, g_ch, r_ch])
    result = np.clip(result, 0, 255).astype(np.uint8)

    return result


def draw_info_panel(img: np.ndarray, kelvin: int, tint: int, fps: float) -> np.ndarray:
    """在画面上叠加信息面板"""
    h, w = img.shape[:2]
    overlay = img.copy()

    # 底部半透明面板
    panel_h = 80
    cv2.rectangle(overlay, (0, h - panel_h), (w, h), (0, 0, 0), -1)
    img = cv2.addWeighted(img, 0.3, overlay, 0.7, 0)

    # 色温文字
    cv2.putText(img, f"Color Temp: {kelvin}K", (20, h - 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    # 色调文字
    cv2.putText(img, f"Tint: {tint:+d}", (20, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    # FPS
    cv2.putText(img, f"FPS: {fps:.1f}", (w - 150, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

    return img


def main():
    # 打开 USB 摄像头 (通常 0 是内置摄像头, 1 是 USB 摄像头)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        # 尝试索引 1
        cap = cv2.VideoCapture(1)
        if not cap.isOpened():
            print("错误: 无法打开摄像头! 请检查 USB 摄像头是否已连接。")
            print("尝试的索引: 0, 1")
            return

    # 获取摄像头信息
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"摄像头已打开: {width}x{height}")

    # 创建窗口
    window_name = "Camera Color Temperature Adjuster"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 640)

    # 初始值
    init_kelvin = 6500   # 日光色温 (初始值)
    init_tint = 0        # 中性色调

    # 创建滑块
    # 色温: 1000K - 20000K, 默认 6500K
    cv2.createTrackbar("Temperature (K)", window_name, init_kelvin, 20000, lambda x: None)
    # 色调: -100 到 100, 默认 0
    cv2.createTrackbar("Tint", window_name, init_tint + 100, 200, lambda x: None)

    # 预设按钮说明 (印在窗口标题附近)
    print("\n操作说明:")
    print("  滑块 'Temperature (K)': 调节色温 (1000K-20000K)")
    print("  滑块 'Tint': 调节色调 (-100 绿 ~ +100 品红)")
    print("  按键 'r': 重置为默认值 (6500K, Tint=0)")
    print("  按键 's': 保存当前帧为截图")
    print("  按键 'q' / ESC: 退出")
    print("  预设参考:")
    print("    2800K - 白炽灯 (暖黄)")
    print("    4000K - 暖白荧光灯")
    print("    5500K - 日光/闪光灯")
    print("    6500K - 阴天/标准日光")
    print("    8000K - 浓云遮日")
    print("    10000K+ - 蓝天 (偏冷/蓝)")

    # FPS 计算
    fps = 0.0
    fps_counter = 0
    fps_timer = cv2.getTickCount()
    frame_count = 0

    print("\n运行中... 按 'q' 或 ESC 退出\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("警告: 读取帧失败, 重试中...")
            continue

        frame_count += 1

        # FPS 计算 (每秒更新)
        if frame_count % 10 == 0:
            ticks = cv2.getTickCount()
            fps = cv2.getTickFrequency() / ((ticks - fps_timer) / 10.0)
            fps_timer = ticks

        # 读取滑块值
        kelvin = cv2.getTrackbarPos("Temperature (K)", window_name)
        tint_raw = cv2.getTrackbarPos("Tint", window_name)
        tint = tint_raw - 100  # 映射 0-200 → -100 到 100

        # 确保色温不低于 1000K
        if kelvin < 1000:
            kelvin = 1000
            cv2.setTrackbarPos("Temperature (K)", window_name, kelvin)

        # 应用白平衡
        adjusted = apply_white_balance(frame, kelvin, tint)

        # 显示原始和调整后的对比 (左右并排)
        # 将原始帧缩放到一半宽度
        h, w = frame.shape[:2]
        original_small = cv2.resize(frame, (w // 2, h // 2))

        # 在调整后的画面上添加信息面板
        display = draw_info_panel(adjusted, kelvin, tint, fps)

        cv2.imshow(window_name, display)

        # 键盘处理
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:  # q 或 ESC
            break
        elif key == ord('r'):  # 重置
            cv2.setTrackbarPos("Temperature (K)", window_name, 6500)
            cv2.setTrackbarPos("Tint", window_name, 100)
            print("已重置: 色温=6500K, Tint=0")
        elif key == ord('s'):  # 截图
            filename = f"screenshot_{kelvin}K_tint{tint:+d}.jpg"
            cv2.imwrite(filename, adjusted)
            print(f"截图已保存: {filename}")
        elif key == ord('1') or key == ord('2') or key == ord('3') or key == ord('4') or key == ord('5'):
            # 快捷键预设
            presets = {
                ord('1'): (2800, 0, "白炽灯"),
                ord('2'): (4000, 0, "暖白荧光灯"),
                ord('3'): (5500, 0, "日光"),
                ord('4'): (6500, 0, "阴天日光"),
                ord('5'): (10000, 0, "蓝天冷调"),
            }
            k, t, name = presets[key]
            cv2.setTrackbarPos("Temperature (K)", window_name, k)
            cv2.setTrackbarPos("Tint", window_name, t + 100)
            print(f"预设: {name} ({k}K)")

    # 清理
    cap.release()
    cv2.destroyAllWindows()
    print("已退出。")


if __name__ == "__main__":
    main()
