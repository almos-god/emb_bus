// -*- coding: utf-8 -*-
// vim: set fileencoding=utf-8 :
// ============================================================================
// 文件: emb_bus/src/thread_affinity.cpp
// 描述: CPU 绑核与 SCHED_FIFO 实时调度实现
// 作者: 具身智能团队
// 日期: 2026-07-26
// ============================================================================
#include "emb_bus/core/thread_affinity.hpp"

#include <pthread.h>
#include <sched.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

namespace emb_bus {

bool set_thread_affinity(int core_id) {
  if (core_id < 0) return true;  // -1 表示不绑核

  const size_t num_cpus = get_num_cpus();
  if (static_cast<size_t>(core_id) >= num_cpus) {
    std::fprintf(stderr, "[emb_bus] Invalid core_id %d (available cores: 0-%zu)\n",
                 core_id, num_cpus - 1);
    return false;
  }

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);

  pthread_t thread = pthread_self();
  int ret = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
  if (ret != 0) {
    std::fprintf(stderr, "[emb_bus] pthread_setaffinity_np failed for core %d: %s\n",
                 core_id, std::strerror(ret));
    return false;
  }
  return true;
}

bool set_realtime_priority(int priority) {
  if (priority < 1 || priority > 99) {
    std::fprintf(stderr, "[emb_bus] RT priority %d out of range [1, 99]\n", priority);
    return false;
  }

  struct sched_param param;
  std::memset(&param, 0, sizeof(param));
  param.sched_priority = priority;

  int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (ret != 0) {
    std::fprintf(stderr,
                 "[emb_bus] pthread_setschedparam(SCHED_FIFO) failed: %s\n"
                 "  (hint: run with CAP_SYS_NICE or sudo)\n",
                 std::strerror(ret));
    return false;
  }
  return true;
}

bool apply_thread_config(const ThreadAffinityConfig& config) {
  bool ok = true;
  if (config.cpu_core >= 0) {
    ok &= set_thread_affinity(config.cpu_core);
  }
  if (config.use_rt_sched) {
    ok &= set_realtime_priority(config.rt_priority);
  }
  return ok;
}

int get_current_cpu() {
  return sched_getcpu();  // 返回 -1 表示失败或未启用 NUMA/sched
}

size_t get_num_cpus() {
  long n = sysconf(_SC_NPROCESSORS_ONLN);
  return (n > 0) ? static_cast<size_t>(n) : 1u;
}

}  // namespace emb_bus