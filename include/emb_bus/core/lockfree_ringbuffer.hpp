// -*- coding: utf-8 -*-
// vim: set fileencoding=utf-8 :
// ============================================================================
// 文件: emb_bus/core/lockfree_ringbuffer.hpp
// 描述: 单生产者单消费者 (SPSC) 无锁环形缓冲区
//       基于 std::atomic<size_t> + acquire-release 内存序
//       目标延迟: < 100ns 级线程间传递
// 作者: 具身智能团队
// 日期: 2026-07-26
// ============================================================================
#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <new>
#include <type_traits>
#include <vector>

namespace emb_bus {

/// SPSC 无锁环形缓冲区（固定容量，power-of-two 自动对齐）
///
/// 设计要点：
/// - head/tail 分别由 consumer/producer 独占写入，另一方只读
/// - 使用 acquire-release 内存序保证 happens-before 关系
/// - 容量自动向上取 2 的幂，用位运算替代取模
/// - 内存预分配，无运行时堆分配
template <typename T>
class LockFreeRingBuffer {
  static_assert(std::is_trivially_copyable_v<T>,
                "T must be trivially copyable for lock-free transfer");

 public:
  explicit LockFreeRingBuffer(size_t requested_capacity)
      : capacity_(next_power_of_two(requested_capacity)),
        mask_(capacity_ - 1),
        buffer_(std::make_unique<T[]>(capacity_)) {
    // 确保对齐
    static_assert(alignof(T) <= 64,
                  "T alignment should not exceed 64 bytes");
  }

  // 禁止拷贝
  LockFreeRingBuffer(const LockFreeRingBuffer&) = delete;
  LockFreeRingBuffer& operator=(const LockFreeRingBuffer&) = delete;

  /// 生产者：尝试推入一个元素
  /// @return true 成功，false 缓冲区满
  bool try_push(const T& item) {
    const size_t head = head_.load(std::memory_order_relaxed);
    const size_t tail = tail_.load(std::memory_order_acquire);

    if (head - tail >= capacity_) {
      return false;  // 满
    }

    buffer_[head & mask_] = item;

    // release: 确保数据写入完成后才更新 head
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  /// 消费者：尝试弹出一个元素
  /// @return true 成功，false 缓冲区空
  bool try_pop(T& item) {
    const size_t tail = tail_.load(std::memory_order_relaxed);
    const size_t head = head_.load(std::memory_order_acquire);

    if (tail >= head) {
      return false;  // 空
    }

    item = buffer_[tail & mask_];

    // release: 确保数据读取完成后才更新 tail
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  /// 查询当前元素数量（非精确，仅用于统计/调试）
  size_t size_approx() const {
    const size_t head = head_.load(std::memory_order_acquire);
    const size_t tail = tail_.load(std::memory_order_acquire);
    return head - tail;
  }

  /// 查询容量
  size_t capacity() const { return capacity_; }

  /// 是否为空（非精确）
  bool empty() const {
    return head_.load(std::memory_order_acquire) ==
           tail_.load(std::memory_order_acquire);
  }

 private:
  static size_t next_power_of_two(size_t v) {
    if (v == 0) return 1;
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v |= v >> 32;
    return v + 1;
  }

  const size_t capacity_;
  const size_t mask_;

  // head 和 tail 分别占独立 cache line，防止伪共享
  alignas(64) std::atomic<size_t> head_{0};
  alignas(64) std::atomic<size_t> tail_{0};

  std::unique_ptr<T[]> buffer_;
};

}  // namespace emb_bus
