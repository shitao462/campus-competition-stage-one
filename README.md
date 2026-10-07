# 校内赛第一阶段：C 板基础功能

本工程基于 STM32F407IG 的 RoboMaster 开发板 C 型和 `sp_middleware` 子模块。业务代码位于 `applications/`，遵循上级目录的《代码风格和命名规范.pdf》。中间件源码未修改。

## 四项功能

1. **蜂鸣器**：上电后由 TIM4_CH3（PD14）播放《两只老虎》开头四小节，约 4 秒，仅播放一次。
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

## 电机失能（姿态联动第 1 项）

两台 GM6020 接在 CAN1，CAN 比特率为 1 Mbps，电机 A/B 的 ID 为 1/2。
`applications/motor_task.cpp` 每 5 ms 向 GM6020 发送电流指令。
右拨杆下档、遥控器未连接或断联时保持零输出；中档执行姿态联动，上档执行固定编码器映射的试验复位。
当前根据实物电机收到电压指令后橙灯常亮的现象，采用电流控制模式，发送帧 ID 为 0x1FE 和 0x2FE（`MOTOR_CURRENT_CONTROL = true`）。两台电机需要采用相同控制模式；若确认为默认电压模式，将该常量改为 `false` 后重新编译。用户已烧录验证进阶第 1 项失能功能。

串口每秒额外输出一行状态：

```text
motor_disabled=1 down=1 rc_alive=1 motor_a_alive=1 motor_b_alive=1 tx_failures=0
```

`down=1` 表示右拨杆下档，`rc_alive=1` 表示遥控器连接正常，两个 `motor_*_alive=1` 表示最近 100 ms 内收到对应电机反馈。
`motor_disabled=1` 表示软件要求零输出，不能单独证明电机收到指令；`tx_failures` 是发送入队失败计数，也不能代替实物验证。
烧录后将右拨杆拨至下档，确认电机没有主动转动或位置保持力。电机自身的轻微磁阻不属于软件位置保持。

编译和链接已验证。烧录后的蜂鸣器、LED、串口和遥控器功能需要在实物板卡上检查；本仓库没有声称完成硬件实测。

## 姿态联动（进阶第 2 项）

代码和测试说明见 [姿态联动与 PID](docs/linkage_control.md)。本次中档联动尚未完成实物验收。
上电后保持 C 板静止，等待 `imu_ready=1`（至少约 2 秒）。确认两个电机反馈在线，再切换右拨杆中档。
左拨杆下/中/上档对应 B 电机与 C 板比例 0.5/-1/3，A 固定为 1。
当前位置作为进入中档时的起点；切换比例会重新捕获起点，避免突然转动。
当前限流 0.25 A，限速 1.0 rad/s（约 9.5 rpm）；PID 增益仍需要在实物上验证和调整。
旋转测试前固定两台电机的定子，保留右拨杆下档作为失能操作。

## 进阶第 3 项：固定角度映射试验

基于 `07df61b`「进阶功能2进阶版」加入右上档复位，无拨杆标定流程，也不将电机启动位置作为复位零位。A/B 的映射参数分别位于 `applications/reset_mapping.hpp`：`MOTOR_A_R_ALIGNMENT_ENCODER`、`MOTOR_B_R_ALIGNMENT_ENCODER`。

这两个参数表示：电机 R 标朝向 C 板启动方向时，各自反馈的原始编码器数值。A=647（yaw_rel_deg=14.88°，enc_a=986），用户反馈对齐效果良好；B 再次按手动同向测量（yaw_rel_deg=-0.86°，enc_b=265）修正为 285，待复位实测确认。定子安装方向和 C 板启动摆放方向需要固定；改变其中任一方向后原映射不再适用。重新加载固件前应恢复 C 板原启动方向，避免参考方向被重置到当前姿态。

1. 右拨杆下档上电，C 板保持近水平且静止，等待 IMU 就绪、两台电机在线。
2. 将右拨杆切到上档，两台电机沿最短路径转到配置的机械方向；左拨杆比例不参与复位。
3. 串口 `reset=1 done=1` 只表示电机已到配置目标，肉眼检查 R 标是否真的同向。`enc=A,B` 是两台原始编码器计数，`error_deg=A,B` 是配置目标减实际角度，`yaw_rel_deg` 是 C 板相对启动方向的角度。
4. 若到位后 R 标存在固定偏角，保留定子和 C 板的位置，右拨杆下档，手动将 R 标对齐 C 板并读取 `enc`。当 `yaw_rel_deg` 接近 0 时，这两个读数即可分别作为映射参数；写入参数后重新编译烧录，运行时不用再拨杆标定。

启动后转动 C 板时，目标方向按板子 yaw 增量 1:1 变化。切回中档从当前位置联动，下档立即零输出。电流上限 0.25 A，原手动识别保持不变。编译和模拟测试通过，用户实机反馈复位大致方向正确，精确对齐仍待调试；本版记录为“进阶功能三基本实现版”。
