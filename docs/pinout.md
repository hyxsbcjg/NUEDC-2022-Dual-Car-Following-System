# 主要引脚

以下引脚来自两车现有 `ti_msp_dl_config.h`；两车通信接口配置一致。实际接线仍应核对开发板原理图、模块电平和 SysConfig。

## 串口

| 用途 | 外设 | RX | TX | 波特率 |
|---|---|---|---|---:|
| Zigbee | UART0 | PB1 | PB0 | 9600 |
| K230 | UART1 | PA9 | PA17 | 38400 |
| 调试/参数 | UART2 | PA22 | PB15 | 115200 |

串口连接必须交叉 TX/RX 并共地。不要仅凭模块端口名称假定 IO 电平兼容。

## 主要执行与传感接口

- 电机 PWM：PA8、PA7。
- 电机 A 方向：PA31、PA28。
- 电机 B 方向：PA1、PB21。
- 编码器：PB4、PB5、PB6、PB7。
- ICM45686 SPI：PB17 PICO、PB19 POCI、PA12 SCLK、PB13 CS。
- OLED 软件 I²C：PB9 SCL、PB8 SDA。
- 灰度地址：PB14、PB11、PB10；模拟输入包含 PA27。
- 按键：PA14、PB20、PA16、PB25。
- 2车 DL1B 接口由 `vehicle-2/project/ti_msp_dl_config.h` 中 `TOP_*` 定义和逐飞驱动共同确定，接线前需再次核对。

首次调试时让车轮离地，禁用高功率输出，先验证每个方向、编码器极性和急停。
