// -*- coding: utf-8 -*-
// vim: set fileencoding=utf-8 :
// ============================================================================
// 文件: emb_bus/types/robot_state_frame.hpp
// 描述: 固定尺寸 POD 传感器数据结构，64 字节 Cache Line 对齐
//       严格禁止 std::vector / std::string 等动态堆分配容器
//       用于零拷贝传输的机器人状态帧定义
// 作者: 具身智能团队
// 日期: 2026-07-26
// ============================================================================
#pragma once

#include <cstdint>
#include <array>

namespace emb_bus {

/// 关节状态（最多 32 个关节）
struct alignas(64) JointState {
  std::array<double, 32> position{};  // 关节角度 (rad)
  std::array<double, 32> velocity{};  // 关节角速度 (rad/s)
  std::array<double, 32> effort{};    // 关节力矩 (Nm)
};

/// IMU 数据
struct alignas(64) ImuData {
  std::array<double, 4> orientation{};         // 四元数 (x, y, z, w)
  std::array<double, 3> angular_velocity{};    // 陀螺仪 (rad/s)
  std::array<double, 3> linear_acceleration{}; // 加速度计 (m/s^2)
};

/// 机器人状态融合帧
struct alignas(64) RobotStateFrame {
  uint64_t timestamp_ns = 0;  // 硬件/系统单调时间戳 (ns)
  uint64_t frame_id = 0;     // 递增帧序号
  JointState joints;
  ImuData imu;
};

// 编译期 POD 检查
static_assert(std::is_trivially_copyable_v<JointState>,
              "JointState must be trivially copyable");
static_assert(std::is_trivially_copyable_v<ImuData>,
              "ImuData must be trivially copyable");
static_assert(std::is_trivially_copyable_v<RobotStateFrame>,
              "RobotStateFrame must be trivially copyable");

// 对齐检查
static_assert(alignof(JointState) == 64, "JointState must be 64-byte aligned");
static_assert(alignof(ImuData) == 64, "ImuData must be 64-byte aligned");
static_assert(alignof(RobotStateFrame) == 64, "RobotStateFrame must be 64-byte aligned");

}  // namespace emb_bus
