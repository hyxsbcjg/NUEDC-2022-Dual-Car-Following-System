# 已知问题与验证边界

## 明确的软件问题

1. **1车反向 Zigbee 链路未启用**：1车 `main()` 没有使能 UART0 Zigbee 接收中断，也没有调用 `UART_Zigbee_Proc()`。2车发送的 Q6 `A6 A6` 停止通知不会被1车当前主循环处理。
2. **2车 Q6 停车状态跳转异常**：`Q6_Proc()` 的 `state_stop` 只实现 `case 0` 和 `case 1`，但 `case 0` 把状态设为 `9`，后续不会进入实际刹车的 `case 1`。
3. **Q5 缺少双车调度帧**：1车 Q5 状态机没有像 Q1–Q4 那样向2车发送启动/停止命令；2车 Q5 只能通过本地按键等路径进入。
4. **接收缓冲区缺少完整边界保护**：K230/Zigbee ISR 持续递增接收位置，未对所有缓冲区写入做可靠上限和溢出恢复。
5. **协议无可靠交付机制**：没有 CRC、ACK、序号、重发、超时和链路失效停车策略。

## 构建边界

- 原始两车工程各自包含一份完全相同的 SDK。开源副本合并为根目录共享 `source`，只调整两份 Keil 工程的 include/library 相对路径。
- 两车原 `UART.h` 都缺少已有 `my_printf` 实现的声明。开源副本各补充一条声明以满足 Arm Compiler 6 的 C99 检查。
- 2026-08-08 新鲜 Rebuild：
  - 1车：Arm Compiler 6.21，`0 Error(s), 0 Warning(s)`；Code 36892、RO 3728、RW 80、ZI 5744，并生成 AXF/HEX。
  - 2车：Arm Compiler 6.21，`0 Error(s), 0 Warning(s)`；Code 43220、RO 3968、RW 96、ZI 8016，并生成 AXF/HEX。
- 两车 Before Build 都因仓库外 `tools/keil/syscfg.bat` 路径不存在而失败。本次编译使用已有 `ti_msp_dl_config.c/.h`，不能宣称 SysConfig 已重新生成。

## 尚未证明

- 固件已烧录并在两块 MSPM0G3507 上冷启动。
- 电机、编码器、IMU、灰度、DL1B、OLED、Zigbee 和 K230 接线正确。
- 跟随距离、PID、转向、停车和超车参数适配当前车辆。
- 双车在丢帧、目标丢失、复位、低电压和机械阻塞时能够安全停车。
- Q1–Q6 满足原赛题全部指标。

修复上述控制链问题并完成低速台架验证前，不应直接进行高速双车测试。
