import os,time  # 导入 os 与 time：os 用于路径/工作目录操作；time 用于 sleep 等延时控制（任务流程节拍）
os.chdir(os.path.dirname(os.path.abspath(__file__)))  # 把当前工作目录切到脚本所在目录：保证相对路径（保存图片等）都落在脚本目录下
import numpy as np  # 导入 numpy：用于数组/向量计算（航点、坐标等）
from FlightController import FC_Controller, FC_Client, FC_Server  # 导入飞控控制相关类：本脚本实际用 FC_Controller（串口直连），Client/Server 备用
from FlightController.Components import LD_Radar  # 导入雷达驱动：LD_Radar 负责雷达数据采集、建图、点云解析、位姿解算等
from FlightController.Solutions.Navigation import Navigation  # 导入导航封装：Navigation 负责把“去航点/定高/绕点”等高级指令变成实时控制闭环
from FlightController.Solutions.PathPlanner import TrajectoryGenerator  # 导入轨迹生成器（本脚本未显式使用，但可能作为规划工具预留）
from loguru import logger  # 导入 loguru 日志：用于打印任务状态、异常等
from SolutionsNew.Vision_Net import FastestDetOnnx  # 导入目标检测器：FastestDetOnnx（用于检测“黄色目标”）
import cv2  # 导入 OpenCV：用于相机读取/图像保存等
from FlightController.Solutions.Vision import *  # 导入视觉工具函数（通配符）：如 change_cam_resolution / set_cam_autowb 等
from FlightController.Solutions.Vision_Net import *  # 导入 Vision_Net 相关（通配符）：可能包含其它网络/推理工具（此脚本主要用 FastestDetOnnx）

BASE_POINT = np.array([0, 0])  # 基地点（任务坐标系原点）：用于定点起飞/基准坐标定义，单位通常为 cm（由系统约定）
LANDING_POINT = np.array([0, 0])  # 降落点（同样为原点）：用于定点降落，通常与 BASE_POINT 一致


class Mission(object):  # 任务封装类：把一整套飞行任务流程组织成可复用对象
    def __init__(self, fc: FC_Controller, radar: LD_Radar):  # 构造函数：注入飞控对象 fc 与雷达对象 radar
        self.fc = fc  # 保存飞控句柄：后续用于模式切换/降落兜底等
        self.radar = radar  # 保存雷达句柄：后续用于注册点云解析函数、读取两根杆坐标等
        self.navi = Navigation(fc, radar)  # 创建导航对象：负责闭环导航（依赖雷达位姿解算/融合等）

    def stop(self):  # 停止任务：主要用于停掉导航线程，避免后台仍持续发实时控制
        self.navi.stop()  # 让 Navigation 停止后台线程/轨迹任务等（具体取决于 Navigation 实现）
        logger.info("[MISSION] Mission stopped")  # 打印日志：提示任务已停止

    def check_yellow(self):  # 黄标检测流程：在飞行途中监控相机，一旦检测到黄色目标则暂停、拍照、再继续原轨迹
        navi = self.navi  # 取一个局部变量引用：减少多次 self.navi 访问，语义更清晰
        cap = cv2.VideoCapture(0)  # 打开默认摄像头（索引 0）：用于持续读取画面做检测
        deep = FastestDetOnnx(drawOutput=False)  # 初始化 ONNX 目标检测器；drawOutput=False 表示不在图上绘制输出
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)  # 读取摄像头输出宽度：用于把检测坐标归一化到 0~1
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)  # 读取摄像头输出高度：用于把检测坐标归一化到 0~1
        while True:  # 无限循环：直到检测到黄标并处理完，或轨迹结束/退出条件触发
            # time.sleep(0.01)  # （调试/限频用）每次循环休眠 10ms：可减少 CPU 占用，但会降低检测频率；当前被注释掉
            image = cap.read()[1]  # 读取一帧图像：cap.read() 返回 (ret, frame)，这里直接取 frame（下标 [1]）
            if image is None:  # 若读取失败或返回空帧：直接进入下一轮循环继续尝试
                continue  # 跳过本次循环：避免后续 detect 对 None 报错
            # cv2.imshow("origin",image) # 调试时使用  # （调试用）显示原始画面窗口：线上飞行一般不启用
            get = deep.detect(image)  # 对当前帧做目标检测：返回检测结果列表（结构取决于 FastestDetOnnx 的实现）
            
            if len(get) > 0 :  # 如果检测到至少一个目标：进入目标位置判断流程（只使用第一个目标）
                x = get[0][0][0]/width  # 取第一个目标的某个 x 坐标并归一化：/width -> 0~1（注意该坐标含义依赖 detect 输出格式）
                y = get[0][0][1]/height  # 取第一个目标的某个 y 坐标并归一化：/height -> 0~1（注意坐标含义依赖 detect 输出格式）
                #logger.info(f"[Yellow] x: {x}, y: {y}")  # （调试用）打印目标归一化位置：便于调阈值
                if x>0.4 and x<0.6 and y<0.6:  # 判断目标是否位于画面中央区域（x 在中间 20% 宽度内，y 在上方 60%）
                    navi.navigation_stop_here()  # 让导航立刻“就地停止”：暂停当前导航/轨迹，避免继续飞过目标
                    logger.info(f"[MISSION] Yellow found!")  # 记录日志：提示已找到黄标并触发停机拍照流程
                    time.sleep(1)  # 暂停 1s：给飞机稳定悬停、画面稳定（避免刚停下模糊）再拍照
                    cv2.imwrite("Yellow_1.jpg", image)  # 保存第 1 张黄标照片：文件名固定写入脚本目录
                    time.sleep(0.1)  # 短暂间隔：避免连续帧完全相同，也给写盘一点时间
                    cv2.imwrite("Yellow_2.jpg", image)  # 保存第 2 张黄标照片：当前代码重复保存同一帧 image（若想不同帧需重新 read）
                    time.sleep(0.1)  # 短暂间隔：同上
                    cv2.imwrite("Yellow_3.jpg", image)  # 保存第 3 张黄标照片：同上（仍是同一帧 image）
                    logger.info("[MISSION] photo save, continue trajectory!")  # 日志：拍照完成，准备恢复飞行轨迹
                    cap.release()  # 释放摄像头：避免资源占用/锁死（后续还会在别处再次打开摄像头拍二维码）
                    navi.navigation_follow_trajectory(navi.traj_list_before_stop, wait=True)  # 用“停止前保存的轨迹点列表”继续飞；wait=True 表示等待轨迹跑完
                    break  # 退出检测循环：黄标任务完成，不再继续检测
            if not navi.traj_running_event.is_set():  # 如果导航轨迹运行事件已清除：说明飞向目标的轨迹已经结束
                logger.info("[MISSION] Trajectory finished, no yellow found")  # 日志：到达终点仍未发现黄标（或未满足阈值）
                cap.release()  # 释放摄像头：确保资源正确回收
                break  # 退出检测循环：不再继续检测（因为飞行段已结束）

    def run(self):  # 主任务流程：把起飞、扫杆、去 A、去 B（巡线）、拍码、绕杆、降落等串成一条完整任务
        fc = self.fc  # 飞控对象局部引用：用于设置日志、事件清理、兜底等
        radar = self.radar  # 雷达对象局部引用：用于注册地图函数、读取两根杆坐标等
        cam = self.cam  # 相机句柄（注意：此脚本中 __init__ 未定义 self.cam，若外部未赋值会抛 AttributeError；但此行不允许改动）
        navi = self.navi  # 导航对象局部引用：后续的飞行控制主要都通过 navi 完成
        ############### 参数 #################  # 分隔注释：参数区（用于集中配置任务速度/高度）
        self.navigation_speed = 25  # 导航速度  # 水平导航速度（cm/s）：影响 navigation_to_waypoint 的闭环期望速度
        self.cruise_height = 105  # 巡航高度  # 巡航高度（cm）：起飞后主要在该高度执行任务（定高/绕点/航点飞行）
        self.vertical_speed = 20  # 垂直速度  # 上升/下降速度（cm/s）：影响 set_height/起降过程的垂直速度限制
        ################ 启动线程 ################  # 分隔注释：启动导航/定位相关线程
        navi.set_navigation_speed(self.navigation_speed)  # 设置导航水平速度：让 Navigation 内部控制器使用该速度作为参考
        navi.set_vertical_speed(self.vertical_speed)  # 设置导航垂直速度：限制升降速度，避免过猛
        navi.start()  # 启动导航线程  # 启动 Navigation 后台线程：通常会周期性读取位姿并发送实时控制指令
        navi.switch_navigation_mode("radar")  # 切换导航模式到 radar：内部会启动雷达位姿解算（start_resolve_pose），为导航提供 (x,y,yaw)
        logger.info("[MISSION] Navigation started")  # 日志：导航系统启动完成
        ################  校准 ################  # 分隔注释：基地点校准（坐标系归零）
        navi.calibrate_basepoint()  # 校准基地点：把当前雷达/融合位姿定义为 (0,0) 原点，后续航点都相对于此点
        ################ 初始化 ################  # 分隔注释：相机/事件等初始化
        fc.set_action_log(False)  # 关闭飞控动作日志输出：减少控制台打印，避免影响性能/可读性
        change_cam_resolution(cam, 640, 480, 60)  # 设置相机分辨率与帧率：640x480@60fps（函数来自 Vision 工具模块）
        set_cam_autowb(cam, True)  # 设置相机自动白平衡：提高颜色稳定性（对黄色识别可能更稳）
        fc.event.key_short.clear()  # 清除“短按键”事件状态：避免旧事件影响本次任务（具体含义取决于飞控事件系统）
        fc.event.key_short.wait_clear()  # 等待按键事件完全清空：确保起飞前输入状态稳定/无残留触发
        fc.set_action_log(True)  # 重新打开飞控动作日志：后续关键动作可记录/输出，便于排错
        ################ 初始化完成 ################  # 分隔注释：初始化结束
        logger.info("[MISSION] Mission Started")  # 日志：任务正式开始
        navi.pointing_takeoff(BASE_POINT, self.cruise_height)  # 定点起飞：从 BASE_POINT 起飞并爬升到巡航高度（cm）
        navi.set_yaw(0)  # 设定偏航角为 0°：统一机头方向/坐标系方向，便于后续“只改 x 偏移”这样的策略成立
        navi.wait_for_yaw()  # 等待偏航角到位：确保 yaw 调整完成再进行后续动作（避免坐标/运动方向误差）
        time.sleep(1)  # 起飞后额外等待 1s：让飞机/传感器/位姿更稳定（减少马上扫杆时的噪声）
        ################ 扫杆 ################  # 分隔注释：使用雷达扫描并定位两根杆（pole）
        R = 77  # 安全偏移半径/距离：用于在杆旁生成安全航点（单位 cm）
        radar.register_map_func(radar.map.find_nearest_with_ext_point_opt, from_=0, to_=90,num=2)   # 注册雷达地图解析函数：在 0~90° 扇区找最近的 2 个目标点（两根杆）
        time.sleep(5)  # 等待 5s：给雷达后台线程时间更新 map_func_results（否则结果可能为空/未刷新）
        logger.info("found two poles")  # 日志：提示已获得两根杆的点（实际上此时只是准备读取结果）
        point_1 = radar.map_func_results[0][0]  # 从 map_func_results 取第 1 个点：默认取 func_id=0 的第 0 个结果点（依赖“这是第一个注册的函数”）
        point_1.distance /= 10  # mm -> cm  # 雷达距离单位为 mm，这里转为 cm（导航坐标系使用 cm）
        xy_point_1 = point_1.to_xy()  # 将（角度+距离）转换为平面坐标 (x,y)：匿名坐标系，x 前方为正，y 左侧为正（由 to_xy 定义）
        point_1_x = xy_point_1[0]  # 杆 1 的 x 坐标（cm）：用于生成航点/绕点等
        point_1_y = xy_point_1[1]  # 杆 1 的 y 坐标（cm）
        point_2 = radar.map_func_results[0][1]  # 从 map_func_results 取第 2 个点：同样来自 func_id=0 的第 1 个结果点
        point_2.distance /= 10  # mm -> cm  # 同样把距离从 mm 转换为 cm
        xy_point_2 = point_2.to_xy()  # 把第 2 根杆点转换成平面坐标 (x,y)
        point_2_x = xy_point_2[0]  # 杆 2 的 x 坐标（cm）
        point_2_y = xy_point_2[1]  # 杆 2 的 y 坐标（cm）
        logger.info(xy_point_1)  # 日志：打印杆 1 的坐标，便于调试/确认方向是否合理
        logger.info(xy_point_2)  # 日志：打印杆 2 的坐标
        now_point = navi.current_point  # 获取当前导航估计位置（x,y）：用于调试或规划（此脚本后面未使用 now_point_x/y 做决策）
        now_point_x = now_point[0]  # 当前 x（cm）：当前估计位置的 x 分量
        now_point_y = now_point[1]  # 当前 y（cm）：当前估计位置的 y 分量
        R = 77  # 再次赋值 R：与前面重复，但保持一致（可能是为了强调后续偏移使用 R）
        WAY_POINT_A: np.ndarray = np.array([point_1_x - R, point_1_y])  # 生成 A 点安全航点：在杆 1 左侧（x-R）处保持距离，避免贴杆撞击
        WAY_POINT_B: np.ndarray = np.array([point_2_x - R, point_2_y])  # 生成 B 点安全航点：在杆 2 左侧（x-R）处保持距离
        ################导航到A杆########  # 分隔注释：飞向 A 杆安全点
        navi.navigation_to_waypoint(WAY_POINT_A)  # 导航到 A 安全航点：阻塞式，直到到点或内部判定到达
        logger.info("[MISSION] Reach A")  # 日志：提示到达 A 点
        ################导航到B杆,过程中巡线########  # 分隔注释：飞向 B 杆安全点，并在途中检测黄标
        navi.set_navigation_speed(speed=10)  # 降低导航速度：让途中检测更从容/更稳定，也降低错过目标概率
        time.sleep(0.5)  # 等待 0.5s：让速度参数切换在控制回路中稳定生效
        navi.navigation_to_waypoint(WAY_POINT_B, wait=False)  # 非阻塞导航到 B：让导航线程后台继续飞行，主线程可以并行做视觉检测
        self.check_yellow()  # 执行黄标检测：循环读取相机，若发现黄标则暂停、拍照、再续航；若轨迹结束则退出
        ################ 在B杆处拍摄二维码 ################  # 分隔注释：到 B 点后拍摄二维码图像（仅采集，不解析）
        logger.info("[MISSION] Reach B")  # 日志：提示到达 B（或至少完成飞向 B 的轨迹段）
        navi.set_navigation_speed(speed=25)  # 恢复导航速度到正常值：后续绕杆/返回等用更快速度
        time.sleep(0.5)  # 等待 0.5s：让速度恢复设置稳定生效
        cap = cv2.VideoCapture(0)  # 再次打开摄像头：用于拍二维码（因为在 check_yellow 中已 release）
        count = 0  # 用于计数保存图片的数量  # 初始化计数器：控制保存图片张数
        while count < 3:  # 控制保存图片的数量为3张  # 循环直到保存 3 张照片
            ret, image = cap.read()  # 读取一帧图像：ret 表示是否读取成功，image 为帧数据
            time.sleep(0.5)  # 每次拍照间隔 0.5s：避免连续帧过于相似，也给姿态稳定时间
            if image is None:  # 若读取失败或空帧：跳过，不计数，继续尝试
                continue  # 继续下一次循环：直到拿到有效图像
            # cv2.imshow("origin", image)  # 调试时使用  # （调试用）显示画面窗口：通常飞行环境不启用
            # 保存图片  # 注释：下面将当前帧写入磁盘文件
            filename = "QR_{}.jpg".format(count + 1)  # 构造文件名：QR_1.jpg、QR_2.jpg、QR_3.jpg
            cv2.imwrite(filename, image)  # 写文件到磁盘：保存二维码照片，供后续离线识别/验收
            logger.info("[MISSION] Saved image {}".format(filename))  # 日志：提示哪一张图片保存成功
            count += 1  # 计数+1：推进到下一张，直到 count==3 退出循环
        cap.release()  # 释放摄像头：拍照结束，释放设备资源
        ################ 绕B杆 ################  # 分隔注释：以 B 杆为圆心绕行半圈
        logger.info("[MISSION] Circle B")  # 日志：提示开始绕 B 杆
        WAY_POINT_B: np.ndarray = np.array([point_2_x, point_2_y])  # 将 B 杆真实坐标作为绕行圆心（不再偏移 R）
        navi.navigation_around_waypoint(WAY_POINT_B, degree=1*np.pi, mode="counterclockwise")  # 围绕 B 点逆时针绕 π 弧度（半圈）
        ################ 导航到A杆 ################  # 分隔注释：转向并导航到 A 杆另一侧安全点
        WAY_POINT_A: np.ndarray = np.array([point_1_x + R, point_1_y])   # 生成 A 的另一侧安全航点：x+R（与之前 x-R 对称），用于从另一侧接近/绕行
        navi.navigation_to_waypoint(WAY_POINT_A,wait=True)  # 导航到该安全点：wait=True 阻塞等待到达，保证后续绕行从正确位置开始
        ################ 绕A杆 ################  # 分隔注释：以 A 杆为圆心绕行半圈
        logger.info("[MISSION] Circle A")  # 日志：提示开始绕 A 杆
        WAY_POINT_A: np.ndarray = np.array([point_1_x, point_1_y])   # 将 A 杆真实坐标作为绕行圆心
        navi.navigation_around_waypoint(WAY_POINT_A, degree=1*np.pi, mode="counterclockwise")  # 围绕 A 点逆时针绕 π 弧度（半圈）
        ################ 定点降落 ################  # 分隔注释：返回降落点并降落
        navi.pointing_landing(LANDING_POINT)  # 定点降落到 LANDING_POINT（通常就是基地点原点），导航层负责闭环回到点并执行降落动作

if __name__ == "__main__":  # Python 脚本入口：直接运行该文件时执行下面的管理/启动/兜底逻辑
    fc = FC_Controller()  # 创建飞控控制器：用于串口直连飞控并发送/接收控制帧
    fc.start_listen_serial(serial_dev="/dev/ttyACM0")  # 启动串口监听：Linux 下常见 /dev/ttyACM0（Windows 需改 COMx）
    fc.wait_for_connection()  # 阻塞等待飞控连接就绪：确保后续发指令不会失败
    radar = LD_Radar()  # 创建雷达对象：负责接收雷达数据并维护 map/位姿等
    radar.start()  # 启动雷达：默认参数启动（可能走串口或飞控转发，依赖 LD_Radar 实现）
    time.sleep(0.5)  # 等待 0.5s：给雷达线程启动与缓冲初始化时间
    navi = Navigation(fc=fc,radar=radar)  # 创建一个导航对象（注意：这里创建了但后面未使用，仅作为预留/调试对象）
    mission = Mission(fc, radar)  # 创建任务对象：把飞控与雷达注入 Mission，后续 run() 负责执行完整任务
    mission.cam = cv2.VideoCapture(0)
    if not mission.cam.isOpened():
        raise RuntimeError("Camera open failed: index 0")

    try:  # 主流程 try：确保异常时仍能进入 finally 做安全兜底
        mission.run()  # 执行任务主流程：起飞→扫杆→A→B(检测黄标)→拍码→绕杆→降落
    except Exception as e:  # 捕获所有异常：防止程序直接崩溃导致无人机失控
        logger.exception(f"[MANAGER] Mission Failed")  # 记录异常堆栈：便于定位问题（logger.exception 会带 traceback）
    finally:  # 无论成功失败都执行：安全收尾（停止导航+自动降落）
        mission.stop()  # 停止导航线程：避免后台仍在发送实时控制指令
        if fc.state.unlock.value:  # 如果飞控仍处于解锁（电机已开）状态：执行自动降落兜底
            logger.warning("[MANAGER] Auto Landing")  # 日志：提示开始自动降落兜底
            fc.set_flight_mode(fc.PROGRAM_MODE)  # 切到程控模式：确保 land/stablize 等指令路径一致可控
            fc.stablize()  # 进入悬停/稳定：中止当前控制，避免继续沿轨迹飞行
            fc.land()  # 触发降落：让飞控执行降落程序
            ret = fc.wait_for_lock()  # 等待自动上锁：降落后飞控应上锁（电机停转）
            if not ret:  # 如果等待超时或失败：用强制 lock 作为最后手段
                fc.lock()  # 强制上锁：确保电机停转（高风险操作，但兜底安全需要）
            try:
                mission.cam.release()
            except Exception:
                pass
    logger.info("[MANAGER] Mission finished")  # 日志：任务结束（无论成功还是异常兜底完成）
    fc.close()  # 关闭飞控串口与后台线程：释放资源，正常退出程序
