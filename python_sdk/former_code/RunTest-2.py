import os, time
os.chdir(os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from loguru import logger

from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation


BASE_POINT = np.array([0.0, 0.0], dtype=float)
LANDING_POINT = np.array([0.0, 0.0], dtype=float)


class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar):
        self.fc = fc
        self.radar = radar
        # ✅ 必须使用关键字参数，否则 Navigation 内部 kwargs["fc"] 会 KeyError
        self.navi = Navigation(fc=fc, radar=radar)

    def stop(self):
        try:
            self.navi.stop()
        except Exception:
            pass
        logger.info("[MISSION] Mission stopped")

    def run(self):
        fc = self.fc
        radar = self.radar
        navi = self.navi

        # ---------------- 参数 ----------------
        navigation_speed = 25   # cm/s
        cruise_height = 105     # cm
        vertical_speed = 20     # cm/s
        R = 77                  # cm 安全偏移距离（离杆的安全距离）

        # ---------------- 启动导航（强制 radar 模式） ----------------
        navi.set_navigation_speed(navigation_speed)
        navi.set_vertical_speed(vertical_speed)

        # ✅ 避免 start() 默认 fusion 导致 RealSense None 报错
        try:
            navi.start(mode="radar")
        except TypeError:
            navi.start("radar")

        logger.info("[MISSION] Navigation started (radar)")
        time.sleep(0.2)

        # ---------------- 校准基地点 ----------------
        navi.calibrate_basepoint()

        # ---------------- 起飞 ----------------
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")

        navi.pointing_takeoff(BASE_POINT, cruise_height)

        # yaw=0 只是“参考方向”，不要求对准世界正北，只要你们一套任务里保持一致即可
        navi.set_yaw(0)
        navi.wait_for_yaw()
        time.sleep(1)

        # ---------------- 扫杆：找两根杆 ----------------
        # 注意：这里扫描的是雷达前方 0~90° 扇区
        radar.register_map_func(
            radar.map.find_nearest_with_ext_point_opt,
            from_=0, to_=90, num=2
        )
        time.sleep(5)

        if (not radar.map_func_results) or (len(radar.map_func_results[0]) < 2):
            raise RuntimeError("Radar did not find 2 poles (map_func_results empty/insufficient).")

        point_1 = radar.map_func_results[0][0]
        point_2 = radar.map_func_results[0][1]

        # 雷达 distance 通常是 mm，这里转成 cm
        point_1.distance /= 10
        point_2.distance /= 10

        xy1 = point_1.to_xy()
        xy2 = point_2.to_xy()

        point_1_x, point_1_y = float(xy1[0]), float(xy1[1])
        point_2_x, point_2_y = float(xy2[0]), float(xy2[1])

        logger.info(f"[MISSION] Pole1 xy(cm): {xy1}")
        logger.info(f"[MISSION] Pole2 xy(cm): {xy2}")

        # ---------------- 生成 A/B 安全点（仅巡线用） ----------------
        # 这里保持原逻辑：x-R 代表在杆的“后方/远离杆”的一侧留出安全距离
        A_safe = np.array([point_1_x - R, point_1_y], dtype=float)
        B_safe = np.array([point_2_x - R, point_2_y], dtype=float)

        # ---------------- 飞到 A_safe ----------------
        logger.info(f"[MISSION] Go A_safe: {A_safe}")
        navi.navigation_to_waypoint(A_safe, wait=True)
        logger.info("[MISSION] Reach A_safe")

        # ---------------- 巡线：A_safe -> B_safe ----------------
        # 你说的“巡线”本质就是沿轨迹从 A_safe 飞到 B_safe
        # 如果想更稳，可以在这段降低速度
        navi.set_navigation_speed(10)
        time.sleep(0.2)

        logger.info(f"[MISSION] Line follow: A_safe -> B_safe, target={B_safe}")
        navi.navigation_to_waypoint(B_safe, wait=True)
        logger.info("[MISSION] Reach B_safe")

        # ---------------- 定点降落 ----------------
        navi.set_navigation_speed(navigation_speed)
        time.sleep(0.2)

        logger.info("[MISSION] Landing")
        navi.pointing_landing(LANDING_POINT)


if __name__ == "__main__":
    fc = FC_Controller()
    fc.start_listen_serial(serial_dev="/dev/ttyACM0")
    fc.wait_for_connection()

    radar = LD_Radar()
    radar.start()
    time.sleep(0.5)

    mission = Mission(fc, radar)

    try:
        mission.run()
    except Exception:
        logger.exception("[MANAGER] Mission Failed")
    finally:
        # 停止导航线程
        mission.stop()

        # 安全兜底：若仍解锁则自动降落/锁定
        try:
            if fc.state.unlock.value:
                logger.warning("[MANAGER] Auto Landing")
                fc.set_flight_mode(fc.PROGRAM_MODE)
                fc.stablize()
                fc.land()
                ret = fc.wait_for_lock()
                if not ret:
                    fc.lock()
        except Exception:
            logger.exception("[MANAGER] Auto Landing Failed")

    logger.info("[MANAGER] Mission finished")
    fc.close()
