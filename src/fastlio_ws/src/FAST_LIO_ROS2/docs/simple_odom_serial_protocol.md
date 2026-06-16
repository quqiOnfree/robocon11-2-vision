# R2 串口通信协议与 ROS2 接口

串口现在由独立工作区 `/root/r2_serial_ws` 中的独立包 `r2_serial` 统一管理，`fast_lio/simple_odom`、视觉节点、后续路径规划节点都不要直接打开 `/dev/ttyACM0`。所有需要下发到 STM32 的数据都发到 `r2_serial` 的 ROS2 接口，由 `r2_data_downlink` 一个节点负责封包、CRC、串口写入和回传解析。

## 数据包容器

串口配置为 `115200 8N1`，无流控制。容器字段使用大端字节序：

| 字段 | 大小（字节） | 值 |
| --- | ---: | --- |
| 帧头 | 2 | `AA 55` |
| 总长度 | 2 | 完整数据包大小，含帧头与帧尾 |
| 消息码 | 2 | 消息类型 |
| CRC16 | 2 | `gdut::crc16_algorithm` |
| 负载 | 可变 | 消息特定字节 |
| 帧尾 | 2 | `55 AA` |

负载中的整数使用小端字节序，以匹配 STM32 侧结构体约定。

## 原始包话题

通用下发入口：

```text
/r2_serial/downlink/packet    r2_serial/msg/SerialPacket
```

通用回传出口：

```text
/r2_serial/uplink/packet      r2_serial/msg/SerialPacket
/r2_serial/uplink/event_code  std_msgs/msg/UInt16
/r2/uplink/packet              r2_serial/msg/SerialPacket    # 兼容旧话题
/r2/uplink/event_code          std_msgs/msg/UInt16            # 兼容旧话题
```

`SerialPacket` 定义：

```text
uint16 code
uint8[] payload
bool clear_pending
```

`clear_pending=true` 用于急停等优先级命令，会清空尚未写出的排队包。

## 已确认消息码

| 消息码 | 方向 | 含义 | 负载 |
| --- | --- | --- | --- |
| `0x0001` | 上位机 -> 下位机 | 视觉夹爪/长杆跟随输出 | `int16_t` |
| `0x0002` | 上位机 -> 下位机 | 视觉跟随输出兼容码 | `int16_t` |
| `0x0003` | 下位机 -> 上位机 | 改变视觉检测状态 | `int16_t`，转发到 `/vision/weapon_pole_cmd_state_2` |
| `0x0004` | 下位机 -> 上位机 | 改变视觉检测状态兼容码 | `int16_t`，转发到 `/vision/weapon_pole_cmd_state_2` |
| `0x0101` | 上位机 -> 下位机 | 上报当前位置 | `int16_t x_mm, y_mm, yaw_deg` |
| `0x0102` | 上位机 -> 下位机 | 下发目标点 | `int16_t x_mm, y_mm, yaw_deg` |
| `0x0103` | 上位机 -> 下位机 | 上台阶指令 | 空 |
| `0x0104` | 上位机 -> 下位机 | 下台阶指令 | 空 |
| `0x0105` | 上位机 -> 下位机 | 急停指令 | 空 |
| `0x0106` | 上位机 -> 下位机 | 进入高位模式 | 空 |
| `0x0107` | 上位机 -> 下位机 | 进入低位模式 | 空 |
| `0x0201` | 下位机 -> 上位机 | 到达目标点 | 空 |
| `0x0202` | 下位机 -> 上位机 | 进入高位模式 | 空 |
| `0x0203` | 下位机 -> 上位机 | 退出高位模式 | 空 |
| `0x0204` | 下位机 -> 上位机 | 上台阶完成 | 空 |
| `0x0205` | 下位机 -> 上位机 | 下台阶完成 | 空 |
| `0x0206` | 下位机 -> 上位机 | 急停完成 | 空 |
| `0x0301` | 下位机 -> 上位机 | 请求下一个路径规划指令 | 空，额外转发到 `/r2_serial/uplink/path_request_next` |
| `0x0311` | 上位机 -> 下位机 | 前进 | 空 |
| `0x0312` | 上位机 -> 下位机 | 后退 | 空 |
| `0x0313` | 上位机 -> 下位机 | 左转 90 度 | 空 |
| `0x0314` | 上位机 -> 下位机 | 右转 90 度 | 空 |
| `0x0315` | 上位机 -> 下位机 | 左移 | 空 |
| `0x0316` | 上位机 -> 下位机 | 右移 | 空 |
| `0x0317` | 上位机 -> 下位机 | 抓取低位 R2KFS | 空 |
| `0x0318` | 上位机 -> 下位机 | 抓取中位 R2KFS | 空 |
| `0x0319` | 上位机 -> 下位机 | 抓取高位 R2KFS | 空 |
| `0x031A` | 上位机 -> 下位机 | 抛弃手中 R2KFS 并抓新的 KFS | 空 |
| `0x031B` | 上位机 -> 下位机 | 已经无命令可获取 | 空 |

## 快捷话题

除了原始包话题，`r2_data_downlink` 还提供一些 `std_msgs/Empty` 或 `Int16MultiArray` 快捷话题：

```text
/r2_serial/downlink/nav_position_mm       std_msgs/msg/Int16MultiArray  -> 0x0101
/r2_serial/downlink/nav_target_mm         std_msgs/msg/Int16MultiArray  -> 0x0102
/r2_serial/downlink/stair_up              std_msgs/msg/Empty            -> 0x0103
/r2_serial/downlink/stair_down            std_msgs/msg/Empty            -> 0x0104
/r2_serial/downlink/emergency_stop        std_msgs/msg/Empty            -> 0x0105
/r2_serial/downlink/enter_high_mode       std_msgs/msg/Empty            -> 0x0106
/r2_serial/downlink/enter_low_mode        std_msgs/msg/Empty            -> 0x0107
/r2_serial/downlink/path/forward          std_msgs/msg/Empty            -> 0x0311
/r2_serial/downlink/path/backward         std_msgs/msg/Empty            -> 0x0312
/r2_serial/downlink/path/turn_left_90     std_msgs/msg/Empty            -> 0x0313
/r2_serial/downlink/path/turn_right_90    std_msgs/msg/Empty            -> 0x0314
/r2_serial/downlink/path/shift_left       std_msgs/msg/Empty            -> 0x0315
/r2_serial/downlink/path/shift_right      std_msgs/msg/Empty            -> 0x0316
/r2_serial/downlink/path/grab_low_kfs     std_msgs/msg/Empty            -> 0x0317
/r2_serial/downlink/path/grab_mid_kfs     std_msgs/msg/Empty            -> 0x0318
/r2_serial/downlink/path/grab_high_kfs    std_msgs/msg/Empty            -> 0x0319
/r2_serial/downlink/path/replace_kfs      std_msgs/msg/Empty            -> 0x031A
/r2_serial/downlink/path/no_command       std_msgs/msg/Empty            -> 0x031B
```

路径规划组也可以直接发布原始包，例如：

```bash
ros2 topic pub --once /r2_serial/downlink/packet r2_serial/msg/SerialPacket \
  "{code: 785, payload: [], clear_pending: false}"
```

其中 `785` 是 `0x0311` 的十进制。

## 启动方式

统一串口节点：

```bash
ros2 launch r2_serial r2_data_downlink.launch.py \
  serial_port:=/dev/ttyACM0 \
  serial_debug_raw:=false
```

`simple_odom` 默认发布到 `/r2_serial/downlink/packet`，不再直接打开串口：

```bash
ros2 run fast_lio simple_odom --ros-args \
  -p odom_topic:=/Odometry
```

如果使用全场重定位坐标：

```bash
ros2 run fast_lio simple_odom --ros-args \
  -p odom_topic:=/r2/global_odometry
```

## 串口下发限速

默认开启串口写入限速，任意两个实际写入串口的数据包之间至少间隔 `10 ms`。这个限速在 `r2_data_downlink` 的统一写队列里完成，所以雷达、视觉、路径规划和手动 raw packet 都会一起受保护。

关闭限速：

```bash
ros2 launch r2_serial r2_data_downlink.launch.py \
  serial_port:=/dev/ttyACM0 \
  write_rate_limit_enabled:=false
```

或者把间隔设为 `0`：

```bash
ros2 launch r2_serial r2_data_downlink.launch.py \
  serial_port:=/dev/ttyACM0 \
  write_min_interval_ms:=0
```

需要调慢时可以改大，例如 `write_min_interval_ms:=20`。
