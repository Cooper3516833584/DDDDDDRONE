import numpy as np
import matplotlib.pyplot as plt

def navigation_circle(waypoint, waypoint_cur, dt: float = 1, degree: float = 2*np.pi, mode = "counterclockwise"):
    """
    创建圆形轨迹并让无人机进行圆形巡航
    waypoint: (x, y, [z]) 圆心坐标 / cm / 匿名(ROS)坐标系 / 基地原点
    waypoint_cur: (x, y, [z]) 轨迹圆周上的点坐标 / cm / 匿名(ROS)坐标系 / 基地原点
    dt: 轨迹精度 / s
    degree: 转过的角度 / rad
    mode: 转向 / 默认为俯视逆时针
    """
    radius = np.linalg.norm(np.array(waypoint) - np.array(waypoint_cur))
    angle_increment = dt * 35 / radius
    
    traj_list = []
    first_angle = np.arctan2(waypoint_cur[1] - waypoint[1], waypoint_cur[0] - waypoint[0])  # 计算起始角度
    angle = first_angle
    i = 1  # 轨迹点的顺序计数器

    if mode == "counterclockwise":
        angle += degree
        while angle > first_angle:
            x = waypoint[0] + radius * np.cos(angle)
            y = waypoint[1] + radius * np.sin(angle)
            z = 100
            traj_list.append([x, y, z])
            angle -= angle_increment
            i += 1
        traj_list.append([waypoint[0] + radius*np.cos(first_angle), waypoint[1] + radius*np.sin(first_angle), 100])
        traj_list.reverse()
    if mode == "clockwise":
        angle -= degree
        while angle <= first_angle:
            x = waypoint[0] + radius * np.cos(angle)
            y = waypoint[1] + radius * np.sin(angle)
            z = 100
            traj_list.append([x, y, z])
            angle += angle_increment
            i += 1
        traj_list.append([waypoint_cur[0], waypoint_cur[1], 100])
        traj_list.reverse()
        
    
    # 可视化
    traj_array = np.array(traj_list)
    plt.figure(figsize=(8, 8))
    plt.plot(traj_array[:, 0], traj_array[:, 1], label='Circle Trajectory')
    plt.scatter(waypoint[0], waypoint[1], color='red', label='Waypoint Center')
    plt.scatter(waypoint_cur[0], waypoint_cur[1], color='green', label='Waypoint Cur')
    
    # 在每个轨迹点上添加注释
    for j, point in enumerate(traj_array):
        plt.annotate(str(j+1), (point[0], point[1]), textcoords="offset points", xytext=(0,10), ha='center')
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Circular Trajectory')
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()

WAY_POINT = np.array([0, 100])
WAY_POINT_CUR = np.array([0, 30])
navigation_circle(WAY_POINT, WAY_POINT_CUR, degree=1*np.pi, mode = "counterclockwise",dt=0.8)
