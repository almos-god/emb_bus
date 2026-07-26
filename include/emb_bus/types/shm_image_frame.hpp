// -*- coding: utf-8 -*-
// vim: set fileencoding=utf-8 :
// ============================================================================
// 文件: emb_bus/types/shm_image_frame.hpp
// 描述: 定长共享内存图像帧，支持最大 1920x1080 RGB8
//       固定尺寸 POD，无动态分配
//       用于零拷贝传输的图像数据帧定义
// 作者: 具身智能团队
// 日期: 2026-07-26
// ============================================================================
#pragma once

#include <cstdint>
#include <array>
#include <cstring>

namespace emb_bus {

/// 图像帧最大分辨率常量
constexpr uint32_t kMaxImageWidth = 1920;
constexpr uint32_t kMaxImageHeight = 1080;
constexpr uint32_t kMaxImageChannels = 4;  // RGBA
constexpr uint32_t kMaxImageSize =
    kMaxImageWidth * kMaxImageHeight * kMaxImageChannels;

/// 像素编码枚举
enum class PixelFormat : uint32_t {
  kRGB8 = 0,
  kRGBA8 = 1,
  kMono8 = 2,
  kDepth16 = 3,
};

/// 定长共享内存图像帧
/// 总大小 = 64B header + 最大像素数据，alignas(64)
struct alignas(64) ShmImageFrame {
  // --- Header (固定 64 字节) ---
  uint64_t timestamp_ns = 0;
  uint64_t frame_id = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t channels = 0;
  uint32_t pixel_format = 0;  // PixelFormat enum
  uint32_t data_size = 0;     // 实际有效数据字节数
  uint32_t reserved[9] = {};  // 填充到 64 字节

  // --- 像素数据区 (固定最大尺寸) ---
  alignas(64) std::array<uint8_t, kMaxImageSize> data{};

  /// 计算给定分辨率和通道数的实际数据大小
  static constexpr uint32_t calc_data_size(uint32_t w, uint32_t h,
                                           uint32_t ch) {
    return w * h * ch;
  }

  /// 设置图像参数
  void set_params(uint32_t w, uint32_t h, uint32_t ch, PixelFormat fmt) {
    width = w;
    height = h;
    channels = ch;
    pixel_format = static_cast<uint32_t>(fmt);
    data_size = calc_data_size(w, h, ch);
  }

  /// 拷贝外部像素数据到帧（仅用于首次填充）
  void copy_from(const uint8_t* src, uint32_t size) {
    if (size > kMaxImageSize) size = kMaxImageSize;
    std::memcpy(data.data(), src, size);
    data_size = size;
  }
};

static_assert(std::is_trivially_copyable_v<ShmImageFrame>,
              "ShmImageFrame must be trivially copyable");
static_assert(alignof(ShmImageFrame) == 64,
              "ShmImageFrame must be 64-byte aligned");

}  // namespace emb_bus
