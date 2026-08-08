# 2022 双车跟随系统（MSPM0G3507）

这是从个人比赛资料中整理出的双车跟随历史源码快照。两辆车均使用 MSPM0G3507：1车负责循迹、赛题状态机和 Zigbee 调度；2车接收1车命令，结合灰度、IMU、DL1B 距离传感器与电机控制完成跟随、停车和超车实验。

仓库选用原目录中更新时间较新的 `1车/0Empty` 和 `2车/0Empty`，没有使用较旧的“副本”目录。两个原始工程目录均未修改。

## 重要说明

这是历史实验源码，不是经过当前硬件完整验收的发行版：

- 源码包含 Q1–Q6 的多组状态机和大量现场参数，但部分双车链路存在明确的软件断点，详见 [已知问题](docs/known-issues.md)。
- 原目录没有本项目配套的 K230 脚本或模型；仓库只记录 MCU 侧预期帧格式。
- 新鲜 Keil Rebuild 只能证明副本可生成固件，不证明烧录、传感器、无线链路、跟随距离、超车轨迹或安全性合格。

## 目录

```text
.
├─ vehicle-1/project/          # 1车 Keil 工程与业务源码
├─ vehicle-2/project/          # 2车 Keil 工程与业务源码
├─ source/                     # 两车共用且内容相同的 TI SDK/CMSIS 依赖
└─ docs/
   ├─ architecture.md
   ├─ protocol.md
   ├─ pinout.md
   └─ known-issues.md
```

原始两车各自携带一份完全相同的 `source` 目录。开源副本只保留一份共享依赖，并将两份 Keil 工程中的 include/library 相对路径各向上调整一级；业务源码没有因此改变。

## 构建方法

1. 安装 Keil MDK、Arm Compiler 6.21 和 Texas Instruments MSPM0G1X0X_G3X0X DFP 1.3.1。
2. 分别打开：
   - `vehicle-1/project/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`
   - `vehicle-2/project/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`
3. 工程已包含现有 `ti_msp_dl_config.c/.h`。如需修改硬件资源，应使用匹配版本的 TI SysConfig 打开各车 `project/empty.syscfg` 并重新生成。
4. 分别执行 Rebuild，烧录前核对目标芯片、下载算法、电机方向、编码器、IMU、灰度、距离传感器和串口引脚。
5. 先让两辆车离地验证输出与急停，再单车低速测试，最后才接入 Zigbee 和运动场地。

为兼容 Arm Compiler 6，开源副本只在两车 `MyDriver/UART/UART.h` 中补充了已有 `my_printf` 函数的声明。原实现逻辑未改。

## 使用建议

1. 分别确认两车按键可以选择 `Q_Num`，且启动/停止状态可控。
2. 用串口工具向2车注入 [Zigbee 命令帧](docs/protocol.md)，不要一开始就接无线模块。
3. 单独验证2车 DL1B 距离值和距离 PID 输出，确保目标丢失时能够安全停车。
4. 对1车循迹与每个路口状态逐段验证，再进行双车任务。
5. Q6 超车与反向停止链路存在已知断点，修复并复验前不得直接用于高速双车运行。

## 文档

- [系统结构](docs/architecture.md)
- [串口协议](docs/protocol.md)
- [主要引脚](docs/pinout.md)
- [已知问题与验证边界](docs/known-issues.md)

## 许可证

项目原创部分采用 [MIT License](LICENSE)。TI SDK、CMSIS、MCUboot、InvenSense、逐飞和其他第三方代码仍受各自许可证约束，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
