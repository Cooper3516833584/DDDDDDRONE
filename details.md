# 设备联调与排障细节

本文只保留当前仍适用的设备、HC-14、协议和安全信息。历史试验过程、已废弃协议和重复联调记录不再保留。

## 1. 当前设备与端口

### 地面站树莓派

- 地址：`192.168.31.107`
- 用户：`cooper`
- 主机名：`pi`
- HC-14：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 -> /dev/ttyUSB0`
- HC-14 USB 标识：CH340，USB ID `1a86:7523`
- RC927/STM32 屏幕候选串口：`/dev/ttyACM0`，USB ID `0483:5740`

### 机载上位机

- 地址：`192.168.31.176`
- 用户：`fc`
- SSH 别名：`fc`
- 主机名：`fc-ubuntu`
- 飞控串口：`/dev/ttyACM0`
- `/dev/ttyUSB0` 当前为 CP2102 接入的其他设备，不是 HC-14，不得用于 GroundStationLink
- HC-14 已移动到飞控 `UT2/USART2`，机载 Linux 不再直接打开 HC-14 串口

生产 GroundStationLink 复用机载上位机与飞控的现有连接，经飞控命令 `0x0D`、UT2/USART2 和 HC-14 收发。不要为 HC-14 打开机载 `/dev/ttyUSB0`，也不要打开地面站的 `/dev/ttyACM0`。

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

地面站代码默认波特率已同步为 `115200`。打开 CH340 串口时必须显式关闭 DTR 和 RTS：

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

飞控固件当前将 UT2/USART2 无线口初始化为 `115200 8N1`。移动到 UT2 的机载 HC-14 本地 UART 必须匹配 `115200`；地面站 CH340/HC-14 也已统一为 `115200 8N1`。两端本地 UART 属于不同设备接口，即使数值相同也不得与飞控 USB 的 500000 混用。

## 3. 当前 GroundStationLink 协议

机载端和地面站端的所有消息统一使用与 `Base.py` 发送帧相同的外层结构：

```text
AA 22 | message_type:u8 | data_len:u8 | data | checksum:u8
```

其中：

- `data_len` 是 `data` 的字节数；
- `checksum = sum(此前全部帧字节) & 0xFF`；
- `data` 内部为 `version | flags | session | seq | payload | HMAC-SHA256前8字节`；
- 当前 GroundStationLink 协议版本为 `2`；
- `payload` 上限为 128 字节；
- 命令、ACK、任务结果、遥测、告警和 LED 控制都使用该格式；
- 已不再发送或解析旧的 `A5 5A` 标准帧和 `C3 3C` 快速遥测帧。

HMAC、会话号、序号、ACK、超时重发和重复命令过滤仍然保留。地面站与机载端必须部署同一协议版本并使用同一 HMAC 密钥，否则不会互通。

经过飞控无线桥时，GroundStationLink 帧外还有一层仅用于 UART2 传输的封装：

```text
BB 33 | bridge_len:u8 | GroundStationLink帧
```

地面站串口传输层负责增加和移除该封装；机载端直接调用飞控的 `send_to_wireless()` 和无线回调。业务协议解析器始终只接收内部 `AA 22` 帧。

该外层格式复用了 `Base.py` 的 `AA 22 / 1字节长度 / 累加和`形式，但各层设备和波特率仍然独立：

```text
机载上位机 <-> 飞控：Base.py，本地飞控串口，默认 500000
机载上位机 <-> 飞控：GroundStationLink 调用 0x0D/0x07，无线 USART2 为 115200
飞控无线模块 <-> 地面站 HC-14：透明无线链路，地面站 CH340 为 115200 8N1
```

GroundStationLink 不转发飞控姿态或控制原始帧。地面站 HC-14 和飞控无线 USART2 当前都使用 `115200`，但它们仍是透明无线链路两端的独立本地 UART；飞控 USB 继续使用 `500000`。

## 4. 当前代码位置

机载端：

```text
C:\Users\TZDEZACR\Desktop\DDDDDrone_Cloned\python_sdk\FlightController\Components\GroundStationLink
```

地面站端：

```text
C:\Users\TZDEZACR\Desktop\ground_station\Ground_Station\components
```

修改协议时必须同步两端的 `protocol.py`、解析调用点和纯逻辑测试。禁止只部署其中一端。

## 5. 当前验证状态

2026-07-14 已完成：

- 机载 GroundStationLink 纯逻辑测试：12 项通过；
- 地面站协议、链路和重试纯逻辑测试：38 项通过；
- 双方对固定消息生成的完整帧逐字节一致；
- 机载上位机重启后 WLAN 当前不可达，飞控服务和远程 `FC_Client` 需要在网络恢复后重新确认；
- 已确认飞控无线桥使用 `BB 33 | 长度 | 数据`，地面站传输层已按此格式适配；
- 已确认 HC-14 位于飞控 UT2/USART2，机载 `/dev/ttyUSB0` 不是 HC-14；
- 已将地面站 HC-14 从 9600 修改为 115200，并回读确认 `B115200 / C28 / S8 / +20 dBm`；
- 地面站代码与运行目录已更新为 115200，远端 38 项纯逻辑测试通过；
- 修改后的真实 PING/ACK 尚未完成：机载上位机重启后 WLAN 暂不可达，不能标记为硬件联调通过。

本次协议迁移后尚未完成真实 HC-14 双端联调。旧协议或旧程序的历史联调结果不能作为新协议已通过硬件验证的证据。

## 6. 推荐排障顺序

1. 确认两台主机可达，并确认地面站 HC-14 映射到稳定 `by-id` 路径。
2. 机载侧只使用飞控 UT2/USART2 上的 HC-14，不把 `/dev/ttyUSB0` 当作 HC-14。
3. 确认地面站 HC-14 为 `115200 8N1`、`DTR=False`、`RTS=False`。
4. 确认机载端通过飞控 `0x0D/0x07` 无线桥收发，地面站端正确处理 `BB 33` 桥接封装。
5. 确认机载无线模块本地 UART 匹配飞控 USART2 的 `115200`。
6. 确认两端 HMAC 密钥一致，但不要把密钥打印到日志。
7. 先做少量静态消息和 PING/ACK 联调，再测试遥测吞吐和重复命令过滤。
8. 检查 checksum/HMAC 失败计数、丢包、ACK 超时和重复包行为。
9. 真实任务前人工验证 STOP、任务结束和资源关闭路径；不要直接以完整飞行任务作为首次协议测试。

## 7. 安全边界

- 不因 HC-14 联调运行解锁、起飞、速度控制、导航、降落、PWM、电机或其他执行器程序。
- 地面站只发送高级任务命令，不直接发送飞控实时姿态或速度控制量。
- 未知、校验失败、HMAC 失败或重复的命令不得触发新的飞行动作。
- STOP 命令必须保持高优先级和幂等处理。
- 测试完成后关闭串口和 SSH 会话。
- 无法完成真实距离、飞行中干扰、高吞吐或长期稳定性测试时，必须明确标记为未验证。
