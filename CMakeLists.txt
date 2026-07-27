# -*- coding: utf-8 -*-
# vim: set fileencoding=utf-8 :
# ============================================================================
# 文件: CMakeLists.txt
# 描述: emb_bus 零拷贝数据总线库的 CMake 构建配置
#       支持独立编译和 ROS2 集成两种模式
# 作者: 具身智能团队
# 日期: 2026-07-26
# ============================================================================
cmake_minimum_required(VERSION 3.10)
project(emb_bus VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Sanitizer 支持（开发/测试时开启）
# 用法: cmake -DEMB_BUS_SANITIZER=ON ..
option(EMB_BUS_SANITIZER "Enable AddressSanitizer + UndefinedBehaviorSanitizer" OFF)
if(EMB_BUS_SANITIZER)
  message(STATUS "[emb_bus] Sanitizer enabled (ASan + UBSan)")
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

# 核心库（thread_affinity.cpp 实现 CPU 绑核与实时调度）
add_library(${PROJECT_NAME}
  src/thread_affinity.cpp
)

# 创建 ALIAS target，支持 emb_bus::emb_bus 语法
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 设置 C++17 标准
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装库
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 导出 CMake 配置（供其他项目 find_package）
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# 生成配置文件
include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# 测试
if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  
  add_executable(test_lockfree_queue tests/test_lockfree_queue.cpp)
  target_link_libraries(test_lockfree_queue ${PROJECT_NAME} GTest::GTest GTest::Main)
  add_test(NAME test_lockfree_queue COMMAND test_lockfree_queue)
  
  add_executable(benchmark_latency tests/benchmark_latency.cpp)
  target_link_libraries(benchmark_latency ${PROJECT_NAME})
endif()
