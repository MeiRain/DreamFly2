# 软件结构

[返回项目首页](../README.md) · [文档索引](README.md)

DreamFly2 按职责分层组织。上层可以依赖下层，下层不应反向依赖飞行逻辑。

```text
Application / FreeRTOS Tasks
            ↓
Flight Management and Control
            ↓
State Estimation and Algorithms
            ↓
Device Drivers
            ↓
Board Support Package
            ↓
STM32 HAL and CMSIS
```

基本原则：

- CubeMX 负责 MCU 引脚、时钟、DMA 和外设初始化配置。
- 自动生成代码与手写业务代码分开存放。
- 驱动只负责设备访问，不包含姿态、PID 或飞行模式逻辑。
- 算法通过输入参数和数据结构工作，不直接调用 HAL。
- 公共模块通过头文件公开最小必要接口，不直接暴露内部变量。
- 每类数据应有明确的生产者和使用者，减少跨模块修改全局状态。
- 中断服务函数保持简短，复杂处理交给任务完成。
- 控制链路不得被日志、Flash 写入或阻塞式通信影响。
- 所有传感器数据应包含有效状态；需要时增加采样时间戳。
- 坐标系、方向、物理单位和电机编号在全工程中保持一致。
- 调试功能可以关闭，不应改变飞行控制的执行结果。
