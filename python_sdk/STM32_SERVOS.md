# 临时 STM32 四路 SG90 组件

`stm32_servos.py` 通过独立 USB-TTL 控制已经烧录的 STM32，115200/8N1。
逻辑关闭为 0°，打开为 90°。舵机外接电源，电源负极、STM32 和 TTL 必须共地。
2026-10-15 用户实测安装位置如下；机头正前方为 0°，逆时针为正。

| 组件通道（1 起算） | STM32 引脚 | 实测投放位置 |
| --- | --- | --- |
| 1 | PA6 | 72° |
| 2 | PA7 | 0° |
| 3 | PB0 | 144° |
| 4 | PB1 | 216° |

## 串口分配核查

2026-10-15 对当前代码的静态检查结果：

- `fleet_bus/air_node.py::attach_air_fleet_node()` 默认选择 `FCWirelessTransport`，
  HC14 接飞控 UT2/USART2，经飞控 USB 连接机载上位机；默认不会打开 CH340。
- `server_ros.py` 使用飞控 USB 和经飞控转发的串口屏；飞控自动识别依据为
  `VID:PID=66CC:2233`，与 CH340 的 `1a86:7523` 不同。
- 仍保留 `fleet_bus/hc14_transport.py` 的 USB 电台测试通道。只有显式传入
  `hc14_port`、`hc14_baudrate` 或测试传输工厂才进入该分支；环境变量本身不会切换默认链路。
- 仓库还保留 `FlightController/Components/relay_lcus.py` 的 CH340 继电器驱动，
  `details.md` 有此前继电器的接线记录。驱动存在不等于当前设备仍插着，
  也不等于启动任务会打开它。当前救援投放通过 `stm32_servos.py` 控制独立 STM32；
  主任务仍从 `payload_servos.py` 读取投放顺位和物理位置映射。

因此，在旧继电器和 USB 电台测试架均未接入、且只新增 STM32 USB-TTL 的前提下，
该 TTL 会是机载唯一 CH340。本次没有连接飞机枚举 USB，不能据代码保证实机插入数量。

组件端口优先级：显式 `port` / CLI `--port` > `STM32_SERVO_PORT` 环境变量 >
枚举得到的唯一 `1a86:7523`。零个或多个匹配都会报错，不猜 `/dev/ttyUSB0`。
若未来接入第二个 CH340，使用实际枚举确认的 `/dev/serial/by-path/...` 指定端口，
不要依赖无序列号 CH340 的通用 by-id 名称。显式指定端口仍会验证 STM32 协议。

## 命令行

在 `python_sdk` 目录执行：

```bash
python3 stm32_servos.py --list
python3 stm32_servos.py status
python3 stm32_servos.py open 1
python3 stm32_servos.py close 1
python3 stm32_servos.py open-all
python3 stm32_servos.py close-all
# 多个 CH340 时，换成实际已确认的路径：
python3 stm32_servos.py --port /dev/serial/by-path/实际路径 status
```

`--list` 只枚举，不打开串口；`status` 打开串口、握手和查询，不发送运动指令。
Windows 可用 `--port COM5`。依赖沿用现有 `pyserial==3.5`，无需新安装其它库。

## 任务中调用

```python
from stm32_servos import STM32Servos

with STM32Servos() as servos:   # 唯一 CH340；也可传 port=...
    servos.close_all()
    servos.open_servo(1)       # PA6；其它通道为 2、3、4
    servos.close_servo(1)
    states = servos.get_states()  # {1: False, 2: False, 3: False, 4: False}
```

导入和构造对象不访问硬件；`connect()` 或进入 `with` 才打开串口。连接时先完成
先用 `!` 作废旧的半条指令，再完成 `PING/PONG` 和严格的四路 `STATUS` 验证，
验证成功前禁止发送开闭指令。
DTR/RTS 显式关闭，Linux 请求独占串口；同一对象的命令与回执由锁串行处理。

命令只有收到匹配 `OK` 才返回；超时、设备错误、复位或异常回执会断开连接并抛异常，
不会自动重发开闭指令。重新 `connect()` 后先查询状态再由任务决定是否重发。
`OK` 和 `STATUS` 都只确认固件目标状态，实际角度、转动时间仍需实测。
调用会等待回执（默认超时 1 秒），应在任务动作线程使用，不要放进导航实时循环。

`close()` / 退出 `with` 只释放串口，舵机保持固件最后目标；需要关闭机构时必须显式
调用 `close_servo()` / `close_all()`。接口调用不等待舵机机械动作结束，连续动作需由任务安排间隔。

救援主任务使用此组件，不再向飞控 PWM 输出舵机命令。视觉投放槽位按
0°、72°、144°、216° 排列，依次映射到实测组件通道 2、1、3、4。
通道位置由用户实测确认；开闭机械方向和带载释放效果仍需单独核验。
在救援主入口 `--confirm-flight` 中，连接或启动握手失败会在终端显示原因并每 2 秒重试，
直到成功或人工停止；动作命令失败不自动重发。人工输入 `loaded` 后才发送 `ALL CLOSE`，
输入 `position_ok` 后才重建 FAST-LIO 位姿。收到小车 `CLOSE`/`REMAP` 时只核验已完成状态并回执，
不再次动作舵机或重启定位服务。此流程需通过 SSH 交互式终端运行；无标准输入的服务入口会在
接触硬件前拒绝实飞模式。起飞前异常退出会保留舵机最后目标状态。
旧 `payload_servos.py` 的未使用角度常量写作 `OPEN_ANGLE=0`、`CLOSE_ANGLE=90`，
与 STM32 固件的逻辑方向相反；旧任务实际发送 PWM 百分比，不能由角度常量推定
旧机构的真实开闭方向。无桨台架上确认新组件的 `OPEN` / `CLOSE` 方向后再带载。

## 无硬件验证

在仓库根目录执行：

```bash
python3 -m py_compile python_sdk/stm32_servos.py python_sdk/testcode/test_stm32_servos.py
python3 -m unittest discover -s python_sdk/testcode -p test_stm32_servos.py
```

测试只使用假串口，不连接飞控、不动作舵机。电脑端固件已在 PA6 完成 5 次开闭，
该记录不代替本组件在飞机 Linux 上的枚举、四路接线及带载验证。
