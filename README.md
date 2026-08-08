# 2022 双车跟随系统

> 基于 MSPM0G3507、Zigbee 和距离传感器的双车跟随历史源码快照。

## 项目简介

1车负责循迹、赛题状态机和 Zigbee 调度；2车接收1车命令，结合灰度、IMU、DL1B 距离传感器与电机控制，完成跟随、停车和超车实验。仓库选用原资料中更新时间较新的 `1车/0Empty` 与 `2车/0Empty`，未采用旧“副本”目录。

## 项目状态

- 发布类型：历史实验源码快照。
- MCU：两车均完成 ArmClang 6.21 新鲜 Rebuild，结果为 `0 Error / 0 Warning`。
- 配套视觉：原目录没有本项目对应的 K230 脚本或模型，只保留 MCU 侧预期帧。
- 硬件：未完成烧录、无线链路、跟随距离、超车轨迹或安全验收。
- 已知断点：Q5 双车调度、Q6 反向停止和部分接收路径尚不完整。

## 仓库结构

```text
.
├── vehicle-1/project/     # 1车 Keil 工程与业务源码
├── vehicle-2/project/     # 2车 Keil 工程与业务源码
├── source/                # 两车共用的 TI SDK/CMSIS 依赖
├── docs/                  # 架构、协议、引脚和已知问题
├── LICENSE
└── THIRD_PARTY_NOTICES.md
```

原始两车 `source` 内容完全一致，因此发布副本只保留一份共享依赖，并调整两份 Keil 工程的相对引用路径。业务源码未因去重而改变。

## 硬件与软件环境

- MCU：TI MSPM0G3507 × 2
- IDE：Keil MDK
- 编译器：Arm Compiler 6.21
- Device Family Pack：TI MSPM0G1X0X/G3X0X 1.3.1
- 传感器：灰度、ICM45686 IMU、DL1B 距离传感器
- 双车通信：Zigbee

## 快速开始

1. 安装 Keil、Arm Compiler 6.21 和 TI DFP 1.3.1。
2. 分别打开两车的 `project/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`。
3. 工程包含现有 `ti_msp_dl_config.c/.h`；修改引脚或时钟后应使用匹配的 TI SysConfig 重新生成。
4. 分别 Rebuild，并核对芯片、下载算法和新产物时间。
5. 架空车轮验证电机方向、编码器、IMU、灰度、距离传感器和急停。
6. 先向2车注入串口命令帧，再连接 Zigbee；最后进行低速双车测试。

## 通信与接口

- Zigbee 帧负责 Q1～Q4 的发车、停车和阶段调度。
- K230 预留帧负责 Q6 超车/停车状态，但仓库没有对应视觉端程序。
- 详细定义见 [通信协议](docs/protocol.md)，主要接线见 [引脚说明](docs/pinout.md)。

## 验证记录

- 1车：ArmClang 6.21 Rebuild，`0 Error / 0 Warning`。
- 2车：ArmClang 6.21 Rebuild，`0 Error / 0 Warning`。
- 两车发布副本仅在 `MyDriver/UART/UART.h` 补充已有 `my_printf` 的声明。
- Keil 工程的全部文件引用存在；文档为 UTF-8 无 BOM且无本地断链。
- SysConfig Before Build 路径在发布目录中失败，编译实际使用已有生成配置。

## 已知问题

- 1车未启用完整的 Zigbee/K230 接收消费路径。
- 2车 Q6 的 `state_stop` 跳转使实际制动分支不可达。
- 1车 Q5 没有像 Q1～Q4 一样发送完整调度帧。
- 串口接收缓冲和协议可靠性需要加强。
- 详见 [已知问题与复验边界](docs/known-issues.md)。

## 开源与许可

原始比赛目录保持不变。项目原创代码和文档采用 [MIT License](LICENSE)；TI SDK、CMSIS、MCUboot、InvenSense、逐飞及其他第三方代码仍受各自许可约束，详见 [第三方说明](THIRD_PARTY_NOTICES.md)。
