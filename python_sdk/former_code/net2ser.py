import select
import socket
import time

import serial
from loguru import logger

# 远程TCP服务器的IP地址和端口号
# TCP_IP = "FC-PC.lan"
TCP_IP = "orangepizero3.lan"
# TCP_IP = "localhost"
TCP_PORT = 2001

# 本地串口的端口号和波特率
SERIAL_PORT = "COM2"
BAUD_RATE = 500000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((TCP_IP, TCP_PORT))
logger.info(f"Connected to {TCP_IP}:{TCP_PORT}.")

ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
logger.info(f"Serial port {SERIAL_PORT} opened.")

while True:
    # 从TCP套接字读取数据并转发到串口
    r, w, x = select.select([sock], [], [], 0)
    if sock in r:
        data = sock.recv(1024)
        if not data:
            logger.warning(f"Connection has been closed.")
            break
        ser.write(data)
        # logger.debug(f"TCP -> Serial: {data}")  # type: ignore

    # 从串口读取数据并转发到TCP套接字
    if ser.in_waiting:
        data = ser.read(ser.in_waiting)
        sock.sendall(data)
        # logger.debug(f"Serial -> TCP: {data}")  # type: ignore

    time.sleep(0.001)
