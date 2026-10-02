# 校内赛第一阶段：C 板基础功能

本工程基于 STM32F407IG 的 RoboMaster 开发板 C 型和 `sp_middleware` 子模块。业务代码位于 `applications/`，遵循上级目录的《代码风格和命名规范.pdf》。中间件源码未修改。

## 四项功能

1. **蜂鸣器**：上电后由 TIM4_CH3（PD14）播放三次短提示音。
2. **LED**：板载 RGB LED（PH12 红、PH11 绿、PH10 蓝）每 200 ms 依次点亮。程序正常运行时持续循环。
3. **串口打印**：板载 BMI088 通过 SPI1 读取三轴加速度和角速度，每 100 ms 从 USART1 TX 输出一行。加速度单位为 m/s²，角速度单位为 rad/s。串口设置为 115200、8N1。
4. **遥控器**：DT7 与 DR16 配对后，把 DR16 的 DBUS 信号接到 C 板 DBUS 口（PWM 接口第 8 列：A8 GND、B8 5V、C8 DBUS）。USART3 RX（PC11）以 100000、8E1 接收；`remote_task.cpp` 中的 `remote` 和 `remote_link_alive` 可在调试器中观察。`remote_link_alive` 在收到有效帧后为 `true`，断联超过 100 ms 后变为 `false`。

## 接线与查看输出

- C 板供电按用户手册使用 8–28 V 电源；烧录和调试使用 SWD。
- 串口输出使用板上 **4-pin UART1 接口**：1 RXD、2 TXD、3 GND、4 5V。将第 2 脚 TXD 和第 3 脚 GND 连接至 USB 转串口模块的 RXD 和 GND；USB 转串口模块需要支持 3.3 V 信号。板壳丝印与芯片的 UART 编号可能不同，应按用户手册的 4-pin 接口和线序识别。
- 串口示例：`acc_mps2=0.012,0.034,9.800 gyro_radps=0.001,0.002,0.003`。`BOARD_TO_ROBOT` 目前为单位矩阵，表示直接打印 BMI088 原始板载坐标；如果板子相对于机器人的安装方向有变化，应按实际安装姿态调整该矩阵。
- 遥控器必须先与接收机配对。接收机每约 14 ms 发出一帧 18 字节 DBUS 数据，C 板内置反相电路，DBUS 口可以直接接入。

## 编译

安装 Arm GNU Toolchain、CMake 和 Ninja 后：

```powershell
git clone --recurse-submodules https://github.com/shitao462/campus-competition-stage-one.git
cd campus-competition-stage-one
cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug
```

生成的固件位于 `build/Debug/text3.elf`。STM32CubeMX 的 `.ioc` 保留原始定时器配置；SPI1、USART1、USART3 与 RGB LED 的补充初始化由 `applications/board_io.c` 完成。重新生成 CubeMX 代码时需确认 `Core` 中 USER CODE 区域和 CMake 源文件列表仍包含本项目的接入代码。

## 验收

编译和链接已验证。烧录后的蜂鸣器、LED、串口和遥控器功能需要在实物板卡上检查；本仓库没有声称完成硬件实测。
