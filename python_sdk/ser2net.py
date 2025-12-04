import select
import socket
import time

import serial
from loguru import logger

# 本地串口的端口号和波特率
SERIAL_PORT = "COM2"
BAUD_RATE = 9600

# TCP服务器的IP地址和端口号
TCP_IP = "0.0.0.0"
TCP_PORT = 2000

# 创建TCP服务器套接字并绑定到本地IP地址和端口号
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((TCP_IP, TCP_PORT))
server_socket.listen(1)
logger.info(f"TCP server listening on {TCP_IP}:{TCP_PORT}.")

# 打开本地串口
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
logger.info(f"Serial port {SERIAL_PORT} opened.")

# 等待客户端连接并循环读取TCP套接字和串口数据并转发
while True:
    # 等待客户端连接
    client_socket, address = server_socket.accept()
    logger.info(f"Connection from {address}.")

    # 循环读取TCP套接字和串口数据并转发
    while True:
        # 从TCP套接字读取数据并转发到串口
        r, w, x = select.select([client_socket], [], [], 0)
        if client_socket in r:
            data = client_socket.recv(1024)
            if not data:
                logger.warning(f"Connection has been closed.")
                break
            ser.write(data)
            logger.debug(f"TCP -> Serial: {data}")  # type: ignore

        # 从串口读取数据并转发到TCP套接字
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            client_socket.sendall(data)
            logger.debug(f"Serial -> TCP: {data}")  # type: ignore

        time.sleep(0.001)
