// -*- coding: utf-8 -*-
// vim: set fileencoding=utf-8 :
// ============================================================================
// 文件: emb_bus/src/zero_copy_publisher.cpp
// 描述: 全局零拷贝能力探测与统计计数器实现
//       - 运行时 DDS/RMW 零拷贝能力诊断
//       - 全局统计计数器（loan 成功/失败/回退拷贝）
// 作者: 具身智能团队
// 日期: 2026-07-26
// ============================================================================

#include "emb_bus/ros2/zero_copy_diagnostics.hpp"

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <cstdio>
#include <cstring>

namespace emb_bus {

// ============================================================================
// 全局统计计数器（线程安全）
// ============================================================================
struct ZeroCopyStats {
  std::atomic<uint64_t> total_loans{0};
  std::atomic<uint64_t> failed_loans{0};
  std::atomic<uint64_t> fallback_copies{0};
};

static ZeroCopyStats g_stats;

// ============================================================================
// 运行时 RMW 零拷贝能力探测
// ============================================================================
void log_rmw_zero_copy_capability() {
  const char* rmw_implementation = rmw_get_implementation_identifier();
  std::printf("[emb_bus] Current ROS 2 RMW Implementation: %s\n", rmw_implementation);

  // 判断常用零拷贝 RMW (如 CycloneDDS / Iceoryx / Connext)
  if (std::strstr(rmw_implementation, "cyclonedds") != nullptr ||
      std::strstr(rmw_implementation, "iceoryx") != nullptr ||
      std::strstr(rmw_implementation, "connext") != nullptr) {
    std::printf("[emb_bus] Hardware/Shm Zero-Copy Capability: ENABLED (LoanedMessage supported)\n");
  } else {
    std::printf("[emb_bus] Hardware/Shm Zero-Copy Capability: LIMITED/UNKNOWN\n");
    std::printf("[emb_bus] Hint: Set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp or rmw_iceoryx_cpp\n");
  }
}

// ============================================================================
// 统计计数器操作（供 ZeroCopyPublisher/Subscriber 调用）
// ============================================================================
void record_loan_success() {
  g_stats.total_loans.fetch_add(1, std::memory_order_relaxed);
}

void record_loan_failure() {
  g_stats.failed_loans.fetch_add(1, std::memory_order_relaxed);
}

void record_fallback_copy() {
  g_stats.fallback_copies.fetch_add(1, std::memory_order_relaxed);
}

// ============================================================================
// 统计信息打印
// ============================================================================
void print_zero_copy_statistics() {
  std::printf("[emb_bus Stats] Total Loans: %lu | Failures: %lu | Fallback Copies: %lu\n",
              g_stats.total_loans.load(std::memory_order_relaxed),
              g_stats.failed_loans.load(std::memory_order_relaxed),
              g_stats.fallback_copies.load(std::memory_order_relaxed));
}

}  // namespace emb_bus
