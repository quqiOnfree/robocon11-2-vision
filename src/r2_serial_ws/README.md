# R2 串口通信工作区

这个工作区是 R2 上位机和 STM32 下位机之间的统一串口收发节点。

## 核心规定：

1.只有 `r2_serial` 包里的 `r2_data_downlink` 节点可以直接打开串口
2.其他程序只发布 ROS2 话题，不直接操作串口
3.下位机回传的数据也统一由 `r2_data_downlink` 解析后转发成 ROS2 话题（别管名字了能用就行）



## 串口断线自动重连

默认开启自动重连。下位机断电、重新烧录或 USB 串口短暂消失后，节点会清空旧连接中尚未写出的下发队列，并按固定间隔重新打开 `serial_port`。断线期间收到的 ROS 下发包会被丢弃，不会缓存到重连后再发送，避免小车恢复后执行过期命令。

默认重连间隔为 `1000 ms`：

```bash
ros2 launch r2_serial r2_data_downlink.launch.py \
  serial_port:=/dev/ttyACM0 \
  reconnect_interval_ms:=1000
```

临时关闭自动重连：

```bash
ros2 launch r2_serial r2_data_downlink.launch.py \
  serial_port:=/dev/ttyACM0 \
  reconnect_enabled:=false
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

## 推荐启动

```bash
source /opt/ros/humble/setup.bash
source /root/r2_serial_ws/install/setup.bash
ros2 launch r2_serial r2_data_downlink.launch.py \
  serial_port:=/dev/ttyACM0 \
  serial_debug_raw:=false
```

## 推荐话题

下发原始协议包：

```text
/r2_serial/downlink/packet    r2_serial/msg/SerialPacket
```

下位机回传原始协议包：

```text
/r2_serial/uplink/packet      r2_serial/msg/SerialPacket
/r2_serial/uplink/event_code  std_msgs/msg/UInt16
```

为了兼容已经写到 `/r2/downlink/packet` 的之前的测试程序，节点默认也订阅：

```text
/r2/downlink/packet            r2_serial/msg/SerialPacket
```

并默认额外发布：

```text
/r2/uplink/packet              r2_serial/msg/SerialPacket
/r2/uplink/event_code          std_msgs/msg/UInt16
```

新代码建议统一使用 `/r2_serial/...` 命名


## 从最早 fast_lio 版本迁移

之前代码可能是这样：

```cpp
#include <fast_lio/msg/serial_packet.hpp>

packet_pub_ = create_publisher<fast_lio::msg::SerialPacket>(
    "/r2/downlink/packet", 10);
```

现在应改为：

```cpp
#include <r2_serial/msg/serial_packet.hpp>

packet_pub_ = create_publisher<r2_serial::msg::SerialPacket>(
    "/r2_serial/downlink/packet", 10);
```

其实写到 `/r2/downlink/packet`也行，节点默认也兼容这个话题，但消息类型得改成 `r2_serial/msg/SerialPacket`，
之前是`fastlio_ws/msg/SerialPacket`，现在收发节点从fastlio里面独立出来了（）

推荐话题：`/r2_serial/downlink/packet`
旧话题兼容：`/r2/downlink/packet`
消息类型必须统一：`r2_serial/msg/SerialPacket`

CMake 依赖从 `fast_lio` 改成 `r2_serial`：

```cmake
find_package(r2_serial REQUIRED)
ament_target_dependencies(your_node rclcpp std_msgs r2_serial)
```

`package.xml` 加：

```xml
<depend>r2_serial</depend>
```

## 下位机回传怎么给其他组

所有通过串口收到的有效协议包都会转发成 ROS2 原始包：

```text
/r2_serial/uplink/packet      r2_serial/msg/SerialPacket
/r2/uplink/packet              r2_serial/msg/SerialPacket
```

如果只关心事件码，可以订阅：

```text
/r2_serial/uplink/event_code  std_msgs/msg/UInt16
/r2/uplink/event_code          std_msgs/msg/UInt16
```

特殊业务也会额外转发：

```text
0x0002/0x0003/0x0004 -> /vision/weapon_pole_cmd_state_2  std_msgs/msg/UInt8
0x0301 -> /r2_serial/uplink/path_request_next  std_msgs/msg/Empty
```

路径规划那边建议订阅 `/r2_serial/uplink/path_request_next`，收到请求后发布下一条路径动作到 `/r2_serial/downlink/path/...` 快捷话题，或者直接发原始 `SerialPacket` 到 `/r2_serial/downlink/packet`。

## 编译顺序

```bash
cd /root/r2_serial_ws
source /opt/ros/humble/setup.bash
colcon build

cd /root/fastlio_ws
source /opt/ros/humble/setup.bash
source /root/livox_ws/install/setup.bash
source /root/r2_serial_ws/install/setup.bash
colcon build --packages-select fast_lio
```

视觉工作区编译前也要先 source 串口工作区：

```bash
source /root/r2_serial_ws/install/setup.bash
```
