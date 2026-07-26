// -*- coding: utf-8 -*-
// vim: set fileencoding=utf-8 :
// ============================================================================
// 文件: emb_bus/ros2/zero_copy_publisher.hpp
// 描述: 封装 rclcpp LoanedMessage 零拷贝增量发布器
//       通过 borrow_loaned_message() 直接从 DDS 共享内存池借出预分配 Chunk
//       实现传感器数据的零拷贝传输
// 作者: 具身智能团队
// 日期: 2026-07-26
// ============================================================================
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <utility>

namespace emb_bus {

/// 零拷贝发布者
///
/// 使用 rclcpp::Publisher::borrow_loaned_message() 从底层 DDS 内存池
/// 借出预分配 Chunk，原地填充数据后发布，全程 0 次内存拷贝。
///
/// 支持的消息类型需实现 ROS2 loanable 接口（如 sensor_msgs::msg::Image）。
/// 对于非 loanable 消息类型，自动退化为普通 publish。
template <typename MsgT>
class ZeroCopyPublisher {
 public:
  using SharedPtr = std::shared_ptr<ZeroCopyPublisher<MsgT>>;

  /// 构造
  /// @param node ROS2 节点指针
  /// @param topic 话题名称
  /// @param qos QoS 配置
  ZeroCopyPublisher(rclcpp::Node* node, const std::string& topic,
                    const rclcpp::QoS& qos)
      : node_(node), topic_(topic) {
    pub_ = node->create_publisher<MsgT>(topic, qos);
  }

  /// 检查底层 DDS 是否支持 LoanedMessage
  bool can_loan() const {
    return pub_->can_loan_messages();
  }

  /// 尝试借出 Loaned Message 并原地填充
  /// @param fill_fn 填充函数，接受 MsgT& 引用
  /// @return true 成功借出并发布，false 不支持 loan
  template <typename FillFn>
  bool publish_loaned(FillFn&& fill_fn) {
    if (!can_loan()) {
      return false;
    }
    try {
      auto loaned_msg = pub_->borrow_loaned_message();
      fill_fn(loaned_msg.get());
      pub_->publish(std::move(loaned_msg));
      return true;
    } catch (const rclcpp::exceptions::RCLErrorBase&) {
      // DDS 不支持 loan，退化为普通发布
      return false;
    }
  }

  /// 普通发布（非零拷贝回退路径）
  void publish(const MsgT& msg) {
    pub_->publish(msg);
  }

  /// 获取底层 publisher
  typename rclcpp::Publisher<MsgT>::SharedPtr get_publisher() const {
    return pub_;
  }

  /// 获取话题名
  const std::string& topic() const { return topic_; }

 private:
  rclcpp::Node* node_;
  std::string topic_;
  typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
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
