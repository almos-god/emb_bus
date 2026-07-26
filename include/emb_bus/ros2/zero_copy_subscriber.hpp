// -*- coding: utf-8 -*-
// vim: set fileencoding=utf-8 :
// ============================================================================
// 文件: emb_bus/ros2/zero_copy_subscriber.hpp
// 描述: 零拷贝只读订阅器
//       消费端获取受保护的共享内存只读指针，可直接转为 Tensor/OpenCV 输入
//       实现传感器数据的零拷贝接收
// 作者: 具身智能团队
// 日期: 2026-07-26
// ============================================================================
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <string>

namespace emb_bus {

/// 零拷贝订阅器
///
/// 订阅 ROS2 话题，当 DDS 中间件支持零拷贝时（如 Iceoryx/FastDDS SHM），
/// 回调中直接获取共享内存只读指针，无需任何内存拷贝。
///
/// 对于非 loanable 消息，退化为普通回调。
template <typename MsgT>
class ZeroCopySubscriber {
 public:
  using SharedPtr = std::shared_ptr<ZeroCopySubscriber<MsgT>>;
  using CallbackT = std::function<void(const MsgT&)>;

  /// 构造
  /// @param node ROS2 节点指针
  /// @param topic 话题名称
  /// @param qos QoS 配置
  /// @param callback 消息回调函数
  ZeroCopySubscriber(rclcpp::Node* node, const std::string& topic,
                     const rclcpp::QoS& qos, CallbackT callback)
      : node_(node), topic_(topic), callback_(std::move(callback)) {
    sub_ = node->create_subscription<MsgT>(
        topic, qos,
        [this](const typename MsgT::SharedPtr msg) {
          if (callback_) {
            callback_(*msg);
          }
        });
  }

  /// 获取底层 subscription
  typename rclcpp::Subscription<MsgT>::SharedPtr get_subscription() const {
    return sub_;
  }

  /// 获取话题名
  const std::string& topic() const { return topic_; }

 private:
  rclcpp::Node* node_;
  std::string topic_;
  CallbackT callback_;
  typename rclcpp::Subscription<MsgT>::SharedPtr sub_;
};

}  // namespace emb_bus
