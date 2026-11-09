# ROBOCON 排球机器人下位机（2027 赛季）

STM32F407 · FreeRTOS · 双 CAN · RS485 · USB CDC
**Status: 🟡 在研（2027 赛季开发中，功能持续迭代）**

> 本仓库是 2027 赛季下位机工程，架构沿用并重构自 2026 赛季（见 [volleyball](https://github.com/LjiaruiAT/volleyball)）。
> **注意：赛季未结束，仓库可能随时变更；部分模块仍在迁移中。**

---

## 1. 目录结构

```
Core/            CubeMX 生成：CAN / UART / DMA / GPIO / FreeRTOS
MyLib/           驱动与算法
  ├─ CANDrive        CAN 底层收发（双总线）
  ├─ motor/motorEx   DJI 电机解析（M3508 / M2006）、串级 PID
  ├─ PID / PID_old   位置式 / 增量式 PID
  ├─ 485_bus         RS485 总线抽象
  ├─ go_motor        Unitree GoMotor RS485 协议
  ├─ usb_trans       USB CDC 收发
  ├─ crc_ccitt       CRC-CCITT 校验
  ├─ bsp_dwt         DWT 计时/延时
  ├─ mylist          通用链表
  └─ RMLibHead       跨平台适配层
MyTask/
  ├─ app.c           任务与业务逻辑
  ├─ Task_Init.c     初始化与周期任务
  └─ config.h        机械/运动参数
Remote_control/   自定义遥控协议栈（命令-回调注册、ACK 重传）
USB_DEVICE/       CubeMX 生成的 CDC 设备层
MDK-ARM/ · EWARM/ Keil 与 IAR 双工具链工程
```

---

## 2. 技术要点

- **双 CAN 并行**：转向/机构与底盘行走分属两条总线，避免反馈与指令互相挤占
- **多协议电机**：DJI（`0x200` 电流帧 + `0x201~` 反馈）、VESC（扩展帧）、Unitree GoMotor（RS485）
- **USB CDC 上位机链路**：`usb_trans.c` 负责收发，配合自定义帧（帧头 + 命令字 + 长度 + CRC16）
- **遥控协议栈**：`register_comm_recv_cb()` 按命令字注册回调，变长包用 DMA + IDLE 空闲中断分包
- **串口错误恢复**：按手册顺序清 ORE/NE/FE/PE 后重挂 DMA

---

## 3. 个人职责

团队项目中的下位机部分：电机驱动、通信协议、状态机与整机联调。

---

## 4. 当前状态

**已具备**
- 双 CAN、USB CDC、RS485、遥控协议栈的基础框架
- 电机驱动与 PID 库可复用

**进行中**
- 上层动作状态机与新机构适配
- 参数整定（与机械强相关，需实机）

**未包含**
- 单元测试、上位机软件、机械图纸

---

## 5. 编译

Keil：`MDK-ARM/` 下打开工程；IAR：`EWARM/` 下打开工程。
CubeMX 配置源文件为根目录 `.ioc`。
