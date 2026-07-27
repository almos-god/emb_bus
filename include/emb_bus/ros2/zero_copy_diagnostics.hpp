// -*- coding: utf-8 -*-
// vim: set fileencoding=utf-8 :
// ============================================================================
// 文件: emb_bus/ros2/zero_copy_diagnostics.hpp
// 描述: 零拷贝诊断与统计接口声明
//       - 运行时 RMW 零拷贝能力探测
//       - 全局统计计数器（loan 成功/失败/回退拷贝）
// 作者: 具身智能团队
// 日期: 2026-07-26
// ============================================================================
#pragma once

#include <cstdint>

namespace emb_bus {

/// 打印当前 RMW 实现的零拷贝能力（启动时调用一次即可）
void log_rmw_zero_copy_capability();

/// 记录一次成功的 LoanedMessage 借出
void record_loan_success();

/// 记录一次 LoanedMessage 借出失败
void record_loan_failure();

/// 记录一次回退到标准拷贝
void record_fallback_copy();

/// 打印全局统计信息（程序退出前调用）
void print_zero_copy_statistics();

}  // namespace emb_bus
