# emb_bus
一个严格 500Hz（每 2ms 触发一次）的实时控制线程循环高性能数据总线 (IPC &amp; Multi-threading)。配置 ROS2 的零拷贝，让仿真器的大容量数据传递到控制节点，杜绝传统网络套接字带来的序列化开销。

    核心功能：
    - 无锁环形缓冲区 (SPSC)：< 100ns 级线程间传递
    - 共享内存池分配器：64 字节 Cache Line 对齐
    - CPU 绑核与 SCHED_FIFO 实时调度
    - ROS2 零拷贝发布/订阅封装
    - 定长 POD 数据结构（RobotStateFrame, ShmImageFrame）
    - 添加移动语义支持（move constructor/assignment）
    - 确保 creator 进程负责 shm_unlink，防止多进程竞争


**500Hz × 500 样本实测**（VMware 虚拟机）：

| QoS | P50 | P95 | P99 | avg | 丢包 |
|---|---|---|---|---|---|
| BEST_EFFORT | **384µs** | 1.55ms | 3.06ms | 567µs | 0% |
| RELIABLE | — | — | **1.39ms** | 401µs | 0% |


环境：os：ubuntu ros:ros jazzy C++:C++ 17 构建系统生成工具：cmake
