// -*- coding: utf-8 -*-
// vim: set fileencoding=utf-8 :
// ============================================================================
// 文件: emb_bus/src/zero_copy_publisher.cpp
// 描述: 零拷贝发布器的非模板辅助实现（如有需要）
//       当前 ZeroCopyPublisher/Subscriber 均为 header-only 模板实现，
//       此文件保留用于未来扩展非模板工具函数。
// 作者: 具身智能团队
// 日期: 2026-07-26
// ============================================================================

#include "emb_bus/ros2/zero_copy_publisher.hpp"
#include "emb_bus/ros2/zero_copy_subscriber.hpp"

namespace emb_bus {

// 预留扩展点：
// - 运行时 DDS 零拷贝能力检测
// - QoS 配置文件解析
// - 统计计数器（loan 成功/失败次数）

}  // namespace emb_bus
