from FlightController.Components import LD_Radar
import time

if __name__ == "__main__":
    radar = LD_Radar()
    radar.start()
    radar.start_resolve_pose(rotation_adapt = True)
    time.sleep(1)
    x_init, y_init, _ = radar.rt_pose
    while True:
        time.slepp(1)
        x, y ,yaw = radar.rt_pose
        x -= x_init
        y -= y_init
        print(f"x={x}cm, y={y}cm, yaw={yaw}deg")