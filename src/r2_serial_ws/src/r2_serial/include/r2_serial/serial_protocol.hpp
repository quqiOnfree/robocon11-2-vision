#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace r2_serial::protocol {

// 上位机 -> 下位机：合并启动配置，payload 为 7×int16_t LE。
//   struct startup_config {
//     area_type area_type_value;        // 0=blue, 1=red
//     begin_type begin_type_value;      // 0=一区, 1=二区, 2=三区坡前, 3=三区重试
//     int16_t origin_x;                 // 起点 X (mm)
//     int16_t origin_y;                 // 起点 Y (mm)
//     int16_t kfs_amount;               // 车内初始方块数量 0~3
//     int16_t arena_load_kfs_amount;    // 三区加载方块数量 0~2
//     int16_t arena_delay_seconds;      // 三区启动等待秒数 10~60
//   };
inline constexpr std::uint16_t kStartupConfig = 0x0000;
inline constexpr std::uint16_t kStartupConfigAck = 0x000A;

// 视觉与下位机共享的消息：0x0001 是视觉下发跟随量；0x0002/0x0003/0x0004 都按下位机视觉状态控制处理。
inline constexpr std::uint16_t kVisionFollow = 0x0001;
inline constexpr std::uint16_t kVisionStateLegacy = 0x0002;
inline constexpr std::uint16_t kVisionState = 0x0003;
inline constexpr std::uint16_t kVisionStateCompat = 0x0004;

// 上位机 -> 下位机：导航、台阶、急停和升降模式命令。
inline constexpr std::uint16_t kPoseUpdate = 0x0101;
inline constexpr std::uint16_t kTargetPose = 0x0102;
inline constexpr std::uint16_t kAtomicStairUp = 0x0103;
inline constexpr std::uint16_t kAtomicStairDown = 0x0104;
inline constexpr std::uint16_t kEmergencyStop = 0x0105;
inline constexpr std::uint16_t kEnterHighMode = 0x0106;
inline constexpr std::uint16_t kEnterLowMode = 0x0107;

// 下位机 -> 上位机：导航状态反馈。事件码直接放在协议 code 字段里，payload 通常为空。
inline constexpr std::uint16_t kMotionCompleted = 0x0201;
inline constexpr std::uint16_t kHighModeEntered = 0x0202;
inline constexpr std::uint16_t kLoweringCompleted = 0x0203;
inline constexpr std::uint16_t kAtomicStairUpCompleted = 0x0204;
inline constexpr std::uint16_t kAtomicStairDownCompleted = 0x0205;
inline constexpr std::uint16_t kEmergencyStopCompleted = 0x0206;

// 下位机 -> 上位机：路径规划请求。收到后路径规划节点应该给出下一条 0x031x 指令。
inline constexpr std::uint16_t kPathRequestNext = 0x0301;

// 上位机 -> 下位机：路径规划动作指令，当前协议表里这些命令 payload 均为空。
inline constexpr std::uint16_t kPathForward = 0x0311;
inline constexpr std::uint16_t kPathBackward = 0x0312;
inline constexpr std::uint16_t kPathTurnLeft90 = 0x0313;
inline constexpr std::uint16_t kPathTurnRight90 = 0x0314;
inline constexpr std::uint16_t kPathShiftLeft = 0x0315;
inline constexpr std::uint16_t kPathShiftRight = 0x0316;
inline constexpr std::uint16_t kPathGrabLowKfs = 0x0317;
inline constexpr std::uint16_t kPathGrabMidKfs = 0x0318;
inline constexpr std::uint16_t kPathGrabHighKfs = 0x0319;
inline constexpr std::uint16_t kPathReplaceKfs = 0x031A;
inline constexpr std::uint16_t kPathNoCommand = 0x031B;
inline constexpr std::uint16_t kPathTurnAround180 = 0x031C;
inline constexpr std::uint16_t kPathRequestNextNew = 0x031D;
inline constexpr std::uint16_t kPathMoveToCol1 = 0x031E;
inline constexpr std::uint16_t kPathMoveToCol2 = 0x031F;
inline constexpr std::uint16_t kPathMoveToCol3 = 0x0320;

// 下位机 -> 上位机：调试消息，变长 char 数据。
inline constexpr std::uint16_t kDebugMessage = 0x0501;

// 下位机 -> 上位机：彩幕显示，payload = uint8_t R, uint8_t G, uint8_t B, char text[0..16]。
inline constexpr std::uint16_t kColorShower = 0x0502;

// STM32 payload 里的 int16_t 使用小端序；协议头、长度、code、CRC 仍是大端序。
inline void appendInt16Le(std::vector<std::uint8_t> &payload,
                          std::int16_t value) {
  const auto bits = static_cast<std::uint16_t>(value);
  payload.push_back(static_cast<std::uint8_t>(bits & 0xff));
  payload.push_back(static_cast<std::uint8_t>((bits >> 8) & 0xff));
}

inline std::optional<std::int16_t> readInt16Le(const std::uint8_t *data,
                                               std::size_t size) {
  if (size != sizeof(std::int16_t) || data == nullptr) {
    return std::nullopt;
  }
  const auto bits = static_cast<std::uint16_t>(data[0]) |
                    (static_cast<std::uint16_t>(data[1]) << 8);
  return static_cast<std::int16_t>(bits);
}

}  // namespace r2_serial::protocol
