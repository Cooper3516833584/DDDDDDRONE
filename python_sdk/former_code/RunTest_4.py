import os,time

os.chdir(os.path.dirname(os.path.abspath(__file__)))

from FlightController import FC_Controller, FC_Client
fc = FC_Client()
fc.connect()
time.sleep(0.5)
fc.set_flight_mode(3)

# 解锁
fc.unlock()         #kadhfahlahiuhadfhdhdfakhahu
time.sleep(2)

# 起飞
fc.take_off(80)
fc.wait_for_takeoff_done(timeout_s=5)
fc.set_flight_mode(2)

# 悬停
fc.set_yaw(yaw=0, speed=0)
time.sleep(1)
fc.stablize()
# fc.wait_for_hovering(timeout_s=1)
time.sleep(5)

# 水平移动
fc.rectangular_move(x=50,y=50,speed=15)
fc.wait_for_last_command_done()
time.sleep(3)

# 下降
fc.set_height(source=1, height=50, speed=20)
time.sleep(2)
fc.set_height(source=1, height=20, speed=15)
time.sleep(3)
fc.set_height(source=1 ,height=0, speed=10)
time.sleep(3)
fc.lock()







