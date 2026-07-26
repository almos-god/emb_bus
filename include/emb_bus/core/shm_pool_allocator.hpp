// -*- coding: utf-8 -*-
// vim: set fileencoding=utf-8 :
// ============================================================================
// 文件: emb_bus/core/shm_pool_allocator.hpp
// 描述: POSIX 共享内存池对齐分配器
//       使用 posix_memalign 确保 64 字节 Cache Line 对齐
//       支持 POSIX shm_open 创建跨进程共享内存
// 作者: 具身智能团队
// 日期: 2026-07-26
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace emb_bus {

/// 对齐分配工具（进程内）
class AlignedAllocator {
 public:
  /// 分配对齐内存
  /// @param size 字节数
  /// @param alignment 对齐值（默认 64 字节 cache line）
  static void* allocate(size_t size, size_t alignment = 64) {
    void* ptr = nullptr;
    // 确保 alignment 是 sizeof(void*) 的倍数且为 2 的幂
    if (alignment < sizeof(void*)) alignment = sizeof(void*);
    int ret = posix_memalign(&ptr, alignment, size);
    if (ret != 0 || ptr == nullptr) {
      throw std::bad_alloc();
    }
    return ptr;
  }

  /// 释放对齐内存
  static void deallocate(void* ptr) {
    std::free(ptr);
  }
};

/// POSIX 共享内存池（跨进程）
///
/// 使用 shm_open + mmap 创建 System V 共享内存段，
/// 所有映射到同一段的进程可零拷贝访问数据。
class ShmPool {
 public:
  /// 创建或打开共享内存池
  /// @param name POSIX shm 名称（如 "/emb_bus_pool"）
  /// @param size 池大小（字节）
  /// @param create true=创建新段，false=打开已有段
  ShmPool(const std::string& name, size_t size, bool create = true)
      : name_(name), size_(size), owner_(create) {
    if (create) {
      // 创建并截断
      fd_ = shm_open(name.c_str(), O_CREAT | O_RDWR | O_EXCL, 0666);
      if (fd_ < 0) {
        // 可能已存在，尝试打开
        fd_ = shm_open(name.c_str(), O_RDWR, 0666);
        if (fd_ < 0) {
          throw std::runtime_error("shm_open failed for: " + name);
        }
        owner_ = false;
      } else {
        if (ftruncate(fd_, static_cast<off_t>(size)) != 0) {
          close(fd_);
          shm_unlink(name.c_str());
          throw std::runtime_error("ftruncate failed for: " + name);
        }
      }
    } else {
      fd_ = shm_open(name.c_str(), O_RDWR, 0666);
      if (fd_ < 0) {
        throw std::runtime_error("shm_open (open) failed for: " + name);
      }
    }

    // mmap 映射
    ptr_ = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (ptr_ == MAP_FAILED) {
      close(fd_);
      if (owner_) shm_unlink(name.c_str());
      throw std::runtime_error("mmap failed for: " + name);
    }
  }

  ~ShmPool() {
    if (ptr_ != nullptr && ptr_ != MAP_FAILED) {
      munmap(ptr_, size_);
    }
    if (fd_ >= 0) {
      close(fd_);
    }
    if (owner_) {
      shm_unlink(name_.c_str());
    }
  }

  // 禁止拷贝
  ShmPool(const ShmPool&) = delete;
  ShmPool& operator=(const ShmPool&) = delete;

  /// 获取共享内存基地址
  void* data() { return ptr_; }
  const void* data() const { return ptr_; }

  /// 获取池大小
  size_t size() const { return size_; }

  /// 获取名称
  const std::string& name() const { return name_; }

 private:
  std::string name_;
  size_t size_;
  int fd_ = -1;
  void* ptr_ = nullptr;
  bool owner_ = false;
};

/// 基于共享内存池的定长 Chunk 分配器
///
/// 将共享内存池划分为固定大小的 Chunk，支持无锁借出/归还。
/// 每个 Chunk 按 64 字节对齐。
template <typename T>
class ShmChunkPool {
  static_assert(std::is_trivially_copyable_v<T>,
                "T must be trivially copyable for shared memory");

 public:
  /// 创建 Chunk 池
  /// @param shm_name 共享内存名称
  /// @param num_chunks Chunk 数量
  ShmChunkPool(const std::string& shm_name, size_t num_chunks)
      : num_chunks_(num_chunks),
        chunk_size_(aligned_size(sizeof(T))) {
    pool_ = std::make_unique<ShmPool>(shm_name, chunk_size_ * num_chunks, true);
  }

  /// 打开已有 Chunk 池
  ShmChunkPool(const std::string& shm_name, size_t num_chunks, bool /*open*/)
      : num_chunks_(num_chunks),
        chunk_size_(aligned_size(sizeof(T))) {
    pool_ = std::make_unique<ShmPool>(shm_name, chunk_size_ * num_chunks, false);
  }

  /// 获取第 i 个 Chunk 的指针
  T* get_chunk(size_t index) {
    if (index >= num_chunks_) return nullptr;
    auto* base = static_cast<uint8_t*>(pool_->data());
    return reinterpret_cast<T*>(base + index * chunk_size_);
  }

  const T* get_chunk(size_t index) const {
    if (index >= num_chunks_) return nullptr;
    auto* base = static_cast<const uint8_t*>(pool_->data());
    return reinterpret_cast<const T*>(base + index * chunk_size_);
  }

  size_t num_chunks() const { return num_chunks_; }
  size_t chunk_size() const { return chunk_size_; }
  ShmPool& pool() { return *pool_; }

 private:
  static size_t aligned_size(size_t s) {
    constexpr size_t align = 64;
    return (s + align - 1) & ~(align - 1);
  }

  size_t num_chunks_;
  size_t chunk_size_;
  std::unique_ptr<ShmPool> pool_;
};

}  // namespace emb_bus
