# Third-party notices

根目录 MIT License 只适用于可确认的项目原创部分，不覆盖下列第三方组件：

- Texas Instruments MSPM0 SDK、DriverLib、设备文件、启动代码和预编译库。
- Arm CMSIS Core。
- MCUboot 及其工具脚本。
- TDK InvenSense ICM45686 驱动。
- 逐飞科技 DL1B、GPIO、软件 I2C 等驱动。
- LCKFB、OLED、灰度传感器及其他保留上游标识的示例代码。

源码文件中的版权声明和许可条款继续有效。再分发或商用前，应逐文件核对上游许可证、NOTICE、厂商条款和原创权属；来源不明的历史片段不会因本仓库发布而自动获得额外授权。

为避免传播可直接用于签名的私钥材料，原 SDK 中的
`source/third_party/mcuboot/image_signTEST.pem` 和
`source/third_party/mcuboot/root-ec-p256.pem` 未纳入 Git 仓库。若确需使用
MCUboot 签名功能，请自行生成并妥善保管密钥，不要使用公开示例私钥。
