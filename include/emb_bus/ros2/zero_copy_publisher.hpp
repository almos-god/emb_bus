// -*- coding: utf-8 -*-
// vim: set fileencoding=utf-8 :
// ============================================================================
// 文件: emb_bus/ros2/zero_copy_publisher.hpp
// 描述: 零拷贝发布器 - 双轨道设计
//       轨道1: ROS2 LoanedMessage 机制（DDS 零拷贝）
//       轨道2: ShmChunkPool 兜底（进程内共享内存）
// 作者: 具身智能团队
// 日期: 2026-07-26
// ============================================================================
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <memory>
#include <string>
#include <utility>

#include "emb_bus/core/shm_pool_allocator.hpp"

namespace emb_bus {

/// 零拷贝发布器
///
/// 双轨道设计：
/// - 轨道1: ROS2 LoanedMessage（DDS 零拷贝，如 Iceoryx/FastDDS SHM）
/// - 轨道2: ShmChunkPool 兜底（进程内共享内存，当 DDS 不支持 loan 时）
///
/// 优先级：LoanedMessage > ShmChunkPool > 普通拷贝
template <typename MsgT>
class ZeroCopyPublisher {
 public:
  using SharedPtr = std::shared_ptr<ZeroCopyPublisher<MsgT>>;

  /// 发布模式
  enum class PublishMode {
    kLoanedMessage,    ///< DDS 零拷贝（最优）
    kShmPool,          ///< 共享内存池兜底
    kStandardCopy      ///< 普通拷贝（降级）
  };

  /// 构造
  /// @param node ROS2 节点指针
  /// @param topic 话题名称
  /// @param qos QoS 配置
  /// @param shm_pool_size 共享内存池大小（0 表示不启用 ShmChunkPool 兜底）
  ZeroCopyPublisher(rclcpp::Node* node, const std::string& topic,
                    const rclcpp::QoS& qos, size_t shm_pool_size = 0)
      : node_(node), topic_(topic) {
    pub_ = node->create_publisher<MsgT>(topic, qos);
    
    // 检测 DDS 是否支持 LoanedMessage
    if (pub_->can_loan_messages()) {
      mode_ = PublishMode::kLoanedMessage;
      RCLCPP_DEBUG(node_->get_logger(), 
                   "ZeroCopyPublisher[%s]: Using LoanedMessage (DDS zero-copy)", 
                   topic.c_str());
    } else if (shm_pool_size > 0) {
      // 启用 ShmChunkPool 兜底
      init_shm_pool(shm_pool_size);
      mode_ = PublishMode::kShmPool;
      RCLCPP_DEBUG(node_->get_logger(), 
                   "ZeroCopyPublisher[%s]: Using ShmChunkPool fallback", 
                   topic.c_str());
    } else {
      mode_ = PublishMode::kStandardCopy;
      RCLCPP_WARN(node_->get_logger(), 
                  "ZeroCopyPublisher[%s]: DDS does not support loan, using standard copy", 
                  topic.c_str());
    }
  }

  ~ZeroCopyPublisher() = default;

  /// 检查当前发布模式
  PublishMode mode() const { return mode_; }

  /// 检查底层 DDS 是否支持 LoanedMessage
  bool can_loan() const {
    return pub_->can_loan_messages();
  }

  /// 借出 Loaned Message（轨道1）
  /// @return LoanedMessage 对象，调用者负责填充并发布
  typename rclcpp::LoanedMessage<MsgT> borrow_loaned_message() {
    return pub_->borrow_loaned_message();
  }

  /// 发布借出的消息
  void publish(typename rclcpp::LoanedMessage<MsgT>&& loaned_msg) {
    pub_->publish(std::move(loaned_msg));
    loaned_count_.fetch_add(1, std::memory_order_relaxed);
  }

  /// 便捷发布接口 - 自动选择最优路径
  /// @param fill_fn 填充函数，接受 MsgT& 引用
  /// @return 实际使用的发布模式
  template <typename FillFn>
  PublishMode publish_with_filler(FillFn&& fill_fn) {
    if (mode_ == PublishMode::kLoanedMessage) {
      // 轨道1: LoanedMessage
      try {
        auto loaned_msg = pub_->borrow_loaned_message();
        fill_fn(loaned_msg.get());
        pub_->publish(std::move(loaned_msg));
        loaned_count_.fetch_add(1, std::memory_order_relaxed);
        return PublishMode::kLoanedMessage;
      } catch (...) {
        // Loan 失败，降级到 ShmPool 或标准拷贝
      }
    }
    
    if (mode_ == PublishMode::kShmPool && shm_pool_) {
      // 轨道2: ShmChunkPool
      auto* chunk = get_next_shm_chunk();
      if (chunk) {
        fill_fn(chunk);
        // 拷贝到 ROS2 消息并发布
        MsgT msg;
        std::memcpy(&msg, chunk, sizeof(MsgT));
        pub_->publish(msg);
        shm_count_.fetch_add(1, std::memory_order_relaxed);
        return PublishMode::kShmPool;
      }
    }
    
    // 轨道3: 标准拷贝
    MsgT msg;
    fill_fn(&msg);
    pub_->publish(msg);
    copy_count_.fetch_add(1, std::memory_order_relaxed);
    return PublishMode::kStandardCopy;
  }

  /// 普通发布（非零拷贝）
  void publish(const MsgT& msg) {
    pub_->publish(msg);
    copy_count_.fetch_add(1, std::memory_order_relaxed);
  }

  /// 获取统计信息
  struct Stats {
    uint64_t loaned_count;
    uint64_t shm_count;
    uint64_t copy_count;
  };

  Stats get_stats() const {
    return {
      loaned_count_.load(std::memory_order_relaxed),
      shm_count_.load(std::memory_order_relaxed),
      copy_count_.load(std::memory_order_relaxed)
    };
  }

  /// 获取底层 publisher
  typename rclcpp::Publisher<MsgT>::SharedPtr get_publisher() const {
    return pub_;
  }

  /// 获取话题名
  const std::string& topic() const { return topic_; }

 private:
  void init_shm_pool(size_t pool_size) {
    // 创建进程内共享内存池
    std::string shm_name = "/emb_bus_pub_" + topic_;
    // 替换非法字符
    for (auto& c : shm_name) {
      if (c == '/') continue;
      if (!std::isalnum(c)) c = '_';
    }
    
    try {
      shm_pool_ = std::make_unique<ShmPool>(shm_name, pool_size * sizeof(MsgT), true);
      shm_pool_capacity_ = pool_size;
      shm_write_index_ = 0;
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(), 
                  "Failed to create ShmPool for topic [%s]: %s", 
                  topic_.c_str(), e.what());
      mode_ = PublishMode::kStandardCopy;
    }
  }

  MsgT* get_next_shm_chunk() {
    if (!shm_pool_ || shm_write_index_ >= shm_pool_capacity_) {
      return nullptr;
    }
    auto* base = static_cast<uint8_t*>(shm_pool_->data());
    auto* chunk = reinterpret_cast<MsgT*>(base + shm_write_index_ * sizeof(MsgT));
    shm_write_index_ = (shm_write_index_ + 1) % shm_pool_capacity_;
    return chunk;
  }

  rclcpp::Node* node_;
  std::string topic_;
  typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
  PublishMode mode_{PublishMode::kStandardCopy};

  // ShmChunkPool 兜底
  std::unique_ptr<ShmPool> shm_pool_;
  size_t shm_pool_capacity_{0};
  size_t shm_write_index_{0};

  // 统计计数器
  std::atomic<uint64_t> loaned_count_{0};
  std::atomic<uint64_t> shm_count_{0};
  std::atomic<uint64_t> copy_count_{0};
};

/// 创建传感器最优 QoS 配置
/// - BEST_EFFORT 可靠性（降低 Ack/Nack 延迟）
/// - VOLATILE 持久性（不保存历史）
/// - KEEP_LAST depth=1（只保留最新一帧）
inline rclcpp::QoS create_sensor_qos() {
  return rclcpp::QoS(1)
      .best_effort()
      .durability_volatile()
      .keep_last(1);
}

/// 创建可靠零拷贝 QoS 配置
/// - RELIABLE 可靠性（配合零拷贝内存池）
/// - VOLATILE 持久性
/// - KEEP_LAST depth=1
inline rclcpp::QoS create_reliable_zerocopy_qos() {
  return rclcpp::QoS(1)
      .reliable()
      .durability_volatile()
      .keep_last(1);
}

}  // namespace emb_bus
