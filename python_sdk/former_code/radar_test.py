from FlightController.Components.LDRadar_Driver import LD_Radar
import time,numpy as np 

radar=LD_Radar()
radar.start()
radar.start_resolve_pose()
# radar.register_map_func(radar.map.find_nearest_with_ext_point_opt, from_=0, to_=90)
# time.sleep(3)  # 等待雷达稳定
# while True:

#     time.sleep(2)  
#     point_1 = radar.map_func_results[0][0]
#     point_1.distance /= 10  # mm -> cm
#     center_point_1 = point_1.to_xy()
#     print(f"point_1_x = {center_point_1[0]}")
#     print(f"point_1_y = {center_point_1[1]}")

#     point_2 = radar.map_func_results[0][1]
#     point_2.distance /= 10  # mm -> cm
#     center_point_2 = point_2.to_xy()
#     print(f"point_2_x = {center_point_2[0]}")
#     print(f"point_2_y = {center_point_2[1]}")
# radar.show_radar_map()
while True:
    time.sleep(1)
