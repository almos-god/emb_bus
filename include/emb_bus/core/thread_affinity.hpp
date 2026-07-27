// -*- coding: utf-8 -*-
// vim: set fileencoding=utf-8 :
// ============================================================================
// 文件: emb_bus/core/thread_affinity.hpp
// 描述: CPU 绑核与 SCHED_FIFO 实时调度接口
// 作者: 具身智能团队
// 日期: 2026-07-26
// ============================================================================
#pragma once

#include <cstddef>

namespace emb_bus {

/// @brief 线程亲和性配置参数
struct ThreadAffinityConfig {
  int cpu_core = -1;        ///< CPU 核心编号，-1 表示不绑核
  bool use_rt_sched = false; ///< 是否启用 SCHED_FIFO 实时调度
  int rt_priority = 80;     ///< 实时优先级 [1, 99]
};

/// @brief 设置线程 CPU 亲和性
/// @param core_id CPU 核心编号，-1 表示不绑核
/// @return 成功返回 true
bool set_thread_affinity(int core_id);

/// @brief 设置线程实时调度优先级 (SCHED_FIFO)
/// @param priority 优先级 [1, 99]
/// @return 成功返回 true
bool set_realtime_priority(int priority);

/// @brief 应用线程亲和性配置
/// @param config 配置参数
/// @return 成功返回 true
bool apply_thread_config(const ThreadAffinityConfig& config);

/// @brief 获取当前线程所在的 CPU 核心编号
/// @return CPU 核心编号，失败返回 -1
int get_current_cpu();

/// @brief 获取系统可用的 CPU 核心数量
/// @return 可用核心数
size_t get_num_cpus();

}  // namespace emb_bus
