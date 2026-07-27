// -*- coding: utf-8 -*-
// vim: set fileencoding=utf-8 :
// ============================================================================
// 文件: emb_bus/ros2/zero_copy_subscriber.hpp
// 描述: 零拷贝订阅器 - 双轨道设计
//       轨道1: ROS2 LoanedMessage 机制（DDS 零拷贝）
//       轨道2: ShmChunkPool 兜底（进程内共享内存）
// 作者: 具身智能团队
// 日期: 2026-07-26
// ============================================================================
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include "emb_bus/core/shm_pool_allocator.hpp"

namespace emb_bus {

/// 零拷贝订阅器
///
/// 双轨道设计：
/// - 轨道1: ROS2 LoanedMessage（DDS 零拷贝，如 Iceoryx/FastDDS SHM）
/// - 轨道2: ShmChunkPool 兜底（进程内共享内存，当 DDS 不支持 loan 时）
///
/// 优先级：LoanedMessage > ShmChunkPool > 普通拷贝
template <typename MsgT>
class ZeroCopySubscriber {
 public:
  using SharedPtr = std::shared_ptr<ZeroCopySubscriber<MsgT>>;
  using CallbackT = std::function<void(const MsgT&)>;
  using LoanedCallbackT = std::function<void(const MsgT&, bool is_zero_copy)>;

  /// 订阅模式
  enum class SubscribeMode {
    kLoanedMessage,    ///< DDS 零拷贝（最优）
    kShmPool,          ///< 共享内存池兜底
    kStandardCopy      ///< 普通拷贝（降级）
  };

  /// 构造
  /// @param node ROS2 节点指针
  /// @param topic 话题名称
  /// @param qos QoS 配置
  /// @param callback 消息回调函数
  /// @param shm_pool_size 共享内存池大小（0 表示不启用 ShmChunkPool 兜底）
  ZeroCopySubscriber(rclcpp::Node* node, const std::string& topic,
                     const rclcpp::QoS& qos, CallbackT callback,
                     size_t shm_pool_size = 0)
      : node_(node), topic_(topic), callback_(std::move(callback)) {
    sub_ = node->create_subscription<MsgT>(
        topic, qos,
        [this](const typename MsgT::SharedPtr msg) {
          if (callback_) {
            callback_(*msg);
            copy_count_.fetch_add(1, std::memory_order_relaxed);
          }
        });

    // 检测 DDS 是否支持 LoanedMessage
    if (sub_->can_loan_messages()) {
      mode_ = SubscribeMode::kLoanedMessage;
      RCLCPP_DEBUG(node_->get_logger(), 
                   "ZeroCopySubscriber[%s]: Using LoanedMessage (DDS zero-copy)", 
                   topic.c_str());
    } else if (shm_pool_size > 0) {
      // 启用 ShmChunkPool 兜底
      init_shm_pool(shm_pool_size);
      mode_ = SubscribeMode::kShmPool;
      RCLCPP_DEBUG(node_->get_logger(), 
                   "ZeroCopySubscriber[%s]: Using ShmChunkPool fallback", 
                   topic.c_str());
    } else {
      mode_ = SubscribeMode::kStandardCopy;
      RCLCPP_WARN(node_->get_logger(), 
                  "ZeroCopySubscriber[%s]: DDS does not support loan, using standard copy", 
                  topic.c_str());
    }
  }

  /// 构造（带零拷贝感知回调）
  /// @param node ROS2 节点指针
  /// @param topic 话题名称
  /// @param qos QoS 配置
  /// @param callback 消息回调函数（第二个参数表示是否零拷贝）
  /// @param shm_pool_size 共享内存池大小（0 表示不启用 ShmChunkPool 兜底）
  ZeroCopySubscriber(rclcpp::Node* node, const std::string& topic,
                     const rclcpp::QoS& qos, LoanedCallbackT callback,
                     size_t shm_pool_size = 0)
      : node_(node), topic_(topic), loaned_callback_(std::move(callback)) {
    sub_ = node->create_subscription<MsgT>(
        topic, qos,
        [this](const typename MsgT::SharedPtr msg) {
          if (loaned_callback_) {
            loaned_callback_(*msg, false);
            copy_count_.fetch_add(1, std::memory_order_relaxed);
          }
        });

    // 检测 DDS 是否支持 LoanedMessage
    if (sub_->can_loan_messages()) {
      mode_ = SubscribeMode::kLoanedMessage;
      RCLCPP_DEBUG(node_->get_logger(), 
                   "ZeroCopySubscriber[%s]: Using LoanedMessage (DDS zero-copy)", 
                   topic.c_str());
    } else if (shm_pool_size > 0) {
      // 启用 ShmChunkPool 兜底
      init_shm_pool(shm_pool_size);
      mode_ = SubscribeMode::kShmPool;
      RCLCPP_DEBUG(node_->get_logger(), 
                   "ZeroCopySubscriber[%s]: Using ShmChunkPool fallback", 
                   topic.c_str());
    } else {
      mode_ = SubscribeMode::kStandardCopy;
      RCLCPP_WARN(node_->get_logger(), 
                  "ZeroCopySubscriber[%s]: DDS does not support loan, using standard copy", 
                  topic.c_str());
    }
  }

  ~ZeroCopySubscriber() = default;

  /// 检查当前订阅模式
  SubscribeMode mode() const { return mode_; }

  /// 检查底层 DDS 是否支持 LoanedMessage
  bool can_loan() const {
    return sub_->can_loan_messages();
  }

  /// 获取底层 subscription
  typename rclcpp::Subscription<MsgT>::SharedPtr get_subscription() const {
    return sub_;
  }

  /// 获取话题名
  const std::string& topic() const { return topic_; }

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

 private:
  void init_shm_pool(size_t pool_size) {
    // 创建进程内共享内存池
    std::string shm_name = "/emb_bus_sub_" + topic_;
    // 替换非法字符
    for (auto& c : shm_name) {
      if (c == '/') continue;
      if (!std::isalnum(c)) c = '_';
    }
    
    try {
      shm_pool_ = std::make_unique<ShmPool>(shm_name, pool_size * sizeof(MsgT), true);
      shm_pool_capacity_ = pool_size;
      shm_read_index_ = 0;
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(), 
                  "Failed to create ShmPool for topic [%s]: %s", 
                  topic_.c_str(), e.what());
      mode_ = SubscribeMode::kStandardCopy;
    }
  }

  MsgT* get_next_shm_chunk() {
    if (!shm_pool_ || shm_read_index_ >= shm_pool_capacity_) {
      return nullptr;
    }
    auto* base = static_cast<uint8_t*>(shm_pool_->data());
    auto* chunk = reinterpret_cast<MsgT*>(base + shm_read_index_ * sizeof(MsgT));
    shm_read_index_ = (shm_read_index_ + 1) % shm_pool_capacity_;
    return chunk;
  }

  rclcpp::Node* node_;
  std::string topic_;
  CallbackT callback_;
  LoanedCallbackT loaned_callback_;
  typename rclcpp::Subscription<MsgT>::SharedPtr sub_;
  SubscribeMode mode_{SubscribeMode::kStandardCopy};

  // ShmChunkPool 兜底
  std::unique_ptr<ShmPool> shm_pool_;
  size_t shm_pool_capacity_{0};
  size_t shm_read_index_{0};

  // 统计计数器
  std::atomic<uint64_t> loaned_count_{0};
  std::atomic<uint64_t> shm_count_{0};
  std::atomic<uint64_t> copy_count_{0};
};

}  // namespace emb_bus
