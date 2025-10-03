# motor drivers

本仓库为基于 STM32 HAL 库 + FreeRTOS CMSISv2 的统一电机驱动接口

## 支持的电机类型

- [x] DJI
    - [x] M3508-C620
    - [x] M2006-C610
- [x] TB6612 + 编码器（STM32 定时器）
- [x] VESC 电调 + 各种电机
