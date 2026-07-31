# 设备联调与排障细节

本文只保留当前仍适用的设备、HC-14、协议和安全信息。历史试验过程、已废弃协议和重复联调记录不再保留。

## 1. 当前设备与端口

### 地面站树莓派

- 地址：`192.168.31.107`
- 用户：`cooper`
- 主机名：`pi`
- HC-14：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 -> /dev/ttyUSB0`
- HC-14 USB 标识：CH340，USB ID `1a86:7523`
- RC927/STM32 屏幕：`/dev/serial/by-id/usb-jixin.pro_CMSIS-DAP_LU_LU_2022_8888-if00 -> /dev/ttyACM0`
- 屏幕当前 USB 标识：CMSIS-DAP_LU，USB ID `c251:f001`

### 机载上位机

- 地址：`192.168.31.176`
- 用户：`fc`
- SSH 别名：`fc`
- 主机名：`fc-ubuntu`
- 飞控串口：`/dev/ttyACM0`
- HC-14 已从飞控 `UT2/USART2` 移到 CH340 测试架，通过 USB 直接连接机载 Linux
- HC-14：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 -> /dev/ttyUSB1`
- HC-14 USB 标识：CH340，USB ID `1a86:7523`
- 雷达 CP2102：`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -> /dev/ttyUSB0`

生产 FleetBus 由机载上位机直接打开 CH340/HC-14，与小车使用相同物理链路和 `BB 33` 封装。它不再调用飞控命令 `0x0D`、无线回调或 `UT2/USART2`。优先使用稳定的 `/dev/serial/by-id/` 路径；若实际 CH340 标识不同，通过 `D_TASK_HC14_PORT` 覆盖，不要猜测易变的 `/dev/ttyUSB*` 编号。地面站仍不得打开 `/dev/ttyACM0`。

本文不保存 SSH 密码、HMAC 密钥或其他凭据。凭据以 `AGENTS.md`、环境变量或用户当前明确提供的信息为准。

## 2. HC-14 当前参数

2026-07-14 初次通过只读 AT 查询确认地面站 CH340/HC-14 为 `9600 8N1`；随后按当前方案发送 `AT+B115200`，并在 115200 下重新查询确认当前配置为：

```text
UART：115200 8N1
无线信道：C28
空中速率：S8
发射功率：+20 dBm
DTR：False
RTS：False
```

机载端和地面站代码默认波特率均为 `115200`。打开 CH340 串口时必须显式关闭 DTR 和 RTS：

```python
ser = serial.Serial()
ser.port = "/dev/ttyUSB0"
ser.baudrate = 115200
ser.rts = False
ser.dtr = False
ser.open()
ser.setRTS(False)
ser.setDTR(False)
```

连续收到 `ORDER ERROR\r\n` 时，优先检查 DTR、RTS、KEY 引脚、串口占用和两端 UART 参数。未经明确授权，不发送修改信道、波特率、空中速率、功率或恢复出厂设置的 AT 指令。

两端 CH340/HC-14 均使用 `115200 8N1`。飞控 USB 继续独立使用 `500000`，不得混用；飞控 `UT2/USART2` 已不在生产 FleetBus 路径中。

## 3. 当前 FleetBus 协议

D 题生产链路使用 FleetBus V1：

```text
D3 91 | version | src | dst | kind | flags | session | seq | payload_len
      | payload | CRC16-CCITT-FALSE | 1D 0F
```

其中 `payload` 上限为 220 字节，完整 FleetBus 帧上限为 239 字节。命令、ACK、普通状态、测绘结果和轨迹批次均使用该协议。会话号、序号、CRC、请求响应匹配、命令去重和超时处理继续保留；FleetBus V1 不使用旧 GroundStationLink V2 的 HMAC 帧。

FleetBus 帧外保留与小车、地面站一致的串口封装：

```text
BB 33 | bridge_len:u8 | FleetBus帧
```

机载端和地面站串口传输层都负责增加和移除该封装。机载端直接读写 CH340，业务协议解析器始终只接收内部 FleetBus 帧。

该外层只负责在透明串口字节流中划分完整 FleetBus 帧，各层设备和波特率仍然独立：

```text
机载上位机 <-> 飞控：Base.py，本地飞控串口，默认 500000
机载上位机 <-> CH340/HC-14：FleetBus，115200 8N1
机载 HC-14 <-> 地面站 HC-14：透明无线链路，两端 CH340 均为 115200 8N1
```

FleetBus 不转发飞控姿态或控制原始帧。飞控 USB 继续使用 `500000`，其 ACK 和 UART2 发送队列不再影响无人机轨迹回传。

## 4. 当前代码位置

机载端：

```text
C:\Users\TZDEZACR\Desktop\DDDDDrone_Cloned\python_sdk\fleet_bus
```

机载上位机部署副本（2026-07-30 只读核验）：

```text
/home/fc/桌面/DDDDrone_Cloned
```

地面站端：

```text
C:\Users\TZDEZACR\Desktop\ground_station\Ground_Station\components
```

修改 FleetBus 协议或 `BB 33` 封装时必须同步两端的 `protocol.py`、串口传输层、解析调用点和纯逻辑测试。禁止只部署其中一端。

## 5. 当前验证状态

2026-07-14 已完成：

- 机载 GroundStationLink 纯逻辑测试：12 项通过；
- 地面站协议、链路和重试纯逻辑测试：38 项通过；
- 双方对固定消息生成的完整帧逐字节一致；
- 机载上位机 WLAN、飞控服务和远程 `FC_Client` 已重新确认在线；
- 已确认飞控无线桥使用 `BB 33 | 长度 | 数据`，地面站传输层已按此格式适配；
- 已确认 HC-14 位于飞控 UT2/USART2，机载 `/dev/ttyUSB0` 不是 HC-14；
- 已将地面站 HC-14 从 9600 修改为 115200，并回读确认 `B115200 / C28 / S8 / +20 dBm`；
- 地面站代码与运行目录已更新为 115200，远端 38 项纯逻辑测试通过；
- 模块互换前，115200 下真实 PING 重传 3 次后 ACK 超时；原始双向探针确认机载 HC-14 当时仍为 9600；
- 两块 HC-14 已互换，原机载模块在地面站 CH340 上回读为 `B9600 / C28 / S8 / +20 dBm`，现已修改并复查为 `B115200 / C28 / S8 / +20 dBm`；当前飞机 UT2 上是此前已确认 115200 的模块，因此两端本地 UART 均已匹配 115200；
- 互换后首次真实 PING 收到 `RECEIVED -> ACCEPTED -> COMPLETED`，最终状态为 `COMPLETED`，重传 0 次；
- 随后连续 5 次真实 PING 均收到相同完整 ACK 序列，最终状态均为 `COMPLETED`，重传均为 0 次，近距离低频 HC-14 双向通信验证通过。

2026-08-01 用户已将机载 HC-14 移到 CH340 测试架并通过 USB 连接上位机。SSH 枚举已确认 CH340 稳定路径映射到 `/dev/ttyUSB1`，CP2102 雷达仍映射到 `/dev/ttyUSB0`。代码已切换为直连方案；直连 PING、15 点 TRACE 高吞吐、真实飞行距离、飞行中干扰和长期稳定性仍需验证。

## 6. 推荐排障顺序

1. 确认两台主机可达，并分别确认机载端、地面站 HC-14 映射到稳定 `by-id` 路径。
2. 确认机载 CH340 为 USB ID `1a86:7523`；若稳定路径与默认值不同，设置 `D_TASK_HC14_PORT`。
3. 确认两端 HC-14 均为 `115200 8N1`、`DTR=False`、`RTS=False`。
4. 确认机载和地面站均正确处理 `BB 33 | length | FleetBus` 封装，且机载不再调用飞控 `0x0D/0x07`。
5. 两块 HC-14 当前均应为 `B115200 / C28 / S8`；若再次更换模块，必须先在带 SET/KEY 控制的 USB 转串口上确认参数。
6. 先做少量静态消息和 PING/ACK 联调，再测试遥测吞吐和重复命令过滤。
7. 检查 CRC 失败计数、丢包、ACK 超时和重复包行为。
8. 检查 TRACE 游标、积压和 `BUFFER_OVERRUN` 日志。
9. 真实任务前人工验证 STOP、任务结束和资源关闭路径；不要直接以完整飞行任务作为首次协议测试。

## 7. 安全边界

- 不因 HC-14 联调运行解锁、起飞、速度控制、导航、降落、PWM、电机或其他执行器程序。
- 地面站只发送高级任务命令，不直接发送飞控实时姿态或速度控制量。
- 未知、CRC 校验失败或重复的命令不得触发新的飞行动作。
- STOP 命令必须保持高优先级和幂等处理。
- 测试完成后关闭串口和 SSH 会话。
- 无法完成真实距离、飞行中干扰、高吞吐或长期稳定性测试时，必须明确标记为未验证。
