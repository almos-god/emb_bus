// -*- coding: utf-8 -*-
// vim: set fileencoding=utf-8 :
// ============================================================================
// 文件: emb_bus/core/thread_affinity.hpp
// 描述: CPU 绑核与 SCHED_FIFO 实时调度策略设置工具
//       用于将线程绑定到特定 CPU 核心并设置实时调度优先级
// 作者: 具身智能团队
// 日期: 2026-07-26
// ============================================================================
#pragma once

#include <cstddef>
#include <string>

namespace emb_bus {

/// 线程实时调度配置
struct ThreadAffinityConfig {
  int cpu_core = -1;         // 绑定的 CPU 物理核编号（-1=不绑核）
  bool use_rt_sched = false; // 是否启用 SCHED_FIFO
  int rt_priority = 50;      // RT 优先级 (1-99)
};

/// 将当前线程绑定到指定 CPU 核
/// @param core_id CPU 核编号
/// @return true 成功
bool set_thread_affinity(int core_id);

/// 设置当前线程为 SCHED_FIFO 实时调度策略
/// @param priority 实时优先级 (1-99)
/// @return true 成功
bool set_realtime_priority(int priority);

/// 同时设置 CPU 绑核 + RT 调度
/// @param config 配置
/// @return true 全部成功
bool apply_thread_config(const ThreadAffinityConfig& config);

/// 获取当前线程绑定的 CPU 核编号
/// @return CPU 核编号，-1 表示未绑定
int get_current_cpu();

/// 获取系统可用 CPU 核数量
size_t get_num_cpus();

}  // namespace emb_bus
