# 电机模式与代码入口

上位机在停机状态选择算法，应用后以固件回报为准。上电默认仍由
`foc/sensorless_config.hpp` 中的 `FOC_USE_MXLEMMING` 决定；运行时切换只保存在 RAM。

| 模式 | 串口指令 | 算法入口 | 电流含义 |
| --- | --- | --- | --- |
| FOC · Ortega-lambda | `M=0\n` | `foc/ortega_lambda.hpp` | d/q 电流 |
| FOC · MXLEMMING | `M=1\n` | `foc/mxlemming_lambda.hpp` | d/q 电流 |
| BLDC · 六步换相 | `M=2\n` | `bldc/bldc.hpp` | 导通线电流 |

## 目录分工

- `setup.cpp`：仅启动调用和 ADC HAL 回调转发，共 18 行。
- `motor_mode.hpp`：稳定的模式编号。
- `foc/motor_control.hpp`：控制模块对外入口；`motor_control_impl.hpp` 组织初始化和前台循环。
- `foc/motor_state.hpp`：控制状态与固定存储；`state_machine_impl.hpp` 管理启停、保护和模式分派。
- `foc/controllers_impl.hpp`：控制器初始化、速度环及电压裕量限流。
- `foc/adc_sampling.hpp`：ADC rank、DMA 下标和电压换算；`adc_control_impl.hpp` 处理每帧采样与控制。
- `foc/drive_impl.hpp`：FOC 电流环、SVPWM 输出和无感启动。
- `foc/parameter_detection_impl.hpp`：FOC 的 R/L/磁链辨识。
- `foc/foc_math.cpp`：坐标变换、观测器共同校验和 PLL，分派到两种独立观测器。
- `foc/diagnostics_impl.hpp`：可选的电流环诊断记录。
- `app/serial_command.hpp`：上位机命令格式解析。
- `app/communication_state.hpp`：串口 RX、状态和高速遥测的固定缓冲区。
- `app/host_protocol_impl.hpp`：命令执行、确认、OBS 状态和 CUR 高速电流协议。
- `app/uart_irq_impl.hpp`：UART 接收、DMA idle 事件和错误回调。
- `bldc/bldc.hpp`：无硬件依赖的六步表、反电动势换相、启动和线电流 PI。
- `bldc/drive_impl.hpp`：BLDC 控制与共同状态机之间的连接。
- `bldc/timer_output.hpp`：G431 三个桥臂的 PWM/强制低电平/悬空输出及 FOC 恢复。
- `bldc/phase_feedback.hpp`：板级 U/V/W 相电压采样接口。

控制模块实现仅由现有 `setup.cpp` 编译单元包含一次，内部再按上述职责组织私有实现，
不单独把 `*_impl.hpp` 加入 Keil 编译组。
新增观测器分支也从原有 `foc_math.cpp` 编译，不需要更改 CubeMX 或工程文件列表。
嵌入式全部使用固定存储，无动态分配、虚函数或 C++ 容器。

## ADC 数据排列与 BLDC 接入

参考本地 `bldc-master/motor/mcpwm.c` 的六步输出表和 `COMM_MODE_DELAY`：
换相后消隐，检测悬空相反电动势过零，延时约 30 电角度后换相。
六步表按本工程 A/B Clarke 正方向排列，正反转方向与 FOC 一致。
启动使用定向保持与强制换相，连续检测到有效反电动势才进入闭环。
线电流 PI 使用本工程的相 R/L 参数，外层复用速度 PI。
此次没有移植 VESC 的积分换相、霍尔/混合模式及全部参数检测流程。
来源许可见 `foc/VESC_LICENSE.txt`（GPL-3.0-or-later）。

按用户更新的 CubeMX 配置，保持**两相电流、三相电压**，双 ADC 均为 3 ranks：

| Rank | ADC1 / DMA 低 16 位 | ADC2 / DMA 高 16 位 |
| --- | --- | --- |
| 1 | OPAMP1 / Ia，索引 0 | OPAMP2 / Ib，索引 1 |
| 2 | IN1 / 母线，索引 2 | IN5 / V 相（PC4），索引 3 |
| 3 | IN14 / W 相（PB11），索引 4 | IN17 / U 相（PA4），索引 5 |

DMA 缓冲区为 6 个 `uint16_t`，启动长度为 **3 个 32 位 DMA 字**。
只校准 A/B 两路电流零点，C 相由同一帧 `Ic = -Ia - Ib` 重建。
FOC 保持用同步 A/B 电流做两相 Clarke；重建的 C 相用于遥测和过流保护。
索引 3 是 V 相电压，不能再当作原来的 C 相电流 ADC。

三相电压按板上 33 kΩ / 2.2 kΩ 分压（16 倍）换算为对地电压，
每次完整 ADC DMA 回调均按 U/V/W 顺序发布给 BLDC，待机时也更新，
因此收到有效采样后 `bfb=1`，可通过原有 BLDC 启动反馈检查。

BLDC 的 `D` 参数辨识指令仍返回 `cmd=7`；辨识先切回 FOC。
相电压输入无效或停止发布时保留 `cmd=6` / `f=13` 的反馈保护。
本次按现有配置解析采样，保留用户的 ADC/PWM 触发时序，没有修改 `.ioc`、`Core`、
`Drivers` 或 Keil 工程。BLDC 的实际相序、PWM 采样窗口和换相仍需实机验证，
主机信号测试不代表已经完成带载调试；本次没有烧录或启动真实电机。

## 串口与上位机

- 使用 Python / PyQt6 上位机：`python matlab/current_ui/current_ui.py`。
- 串口工具栏下方的“控制算法”选择 Ortega-lambda / MXLEMMING / BLDC，点击“应用算法”。
- 未连接时也显示算法区域；修改 Python 代码后需退出旧窗口并重新运行。
- USART2：921600 / 8N1。
- 原有 `R`、`S`、`D`、`IQ=`、`V=`、`ILIM=`、`PH=` 保留。
- `M=` 仅允许 `IDLE`、无故障且没有自动重试意图时使用；其他状态返回 `cmd=4`。
- `OBS` 增加 `drv=0/1/2` 和 `bfb=0/1`，分别为实际模式和相电压反馈是否新鲜。
- `obs=O/M/B` 对应 Ortega、MXLEMMING、BLDC；BLDC 下 `id=0`，`iq` 为有符号线电流。
- 仍使用 `mode=I/S` 表示电流/速度控制，与算法选择 `drv` 分开。
- 界面根据固件确认更新当前算法，运行时锁定切换，状态超时禁用控制；停止按钮保持可用。

## 验证入口

```text
python matlab/tests/test_motor_modes.py
python matlab/tests/test_adc_rank_sync.py
python matlab/tests/test_clark_two_phase.py
python matlab/tests/test_startup_regression.py
python matlab/tests/test_adaptive_flux.py
python matlab/build_sensorless.py --output matlab/firmware_bldc_modes
python -m unittest discover -s matlab/current_ui/tests -p test_pyqt_protocol.py -v
python -m unittest discover -s matlab/current_ui/tests -p test_algorithm_ui.py -v
```

界面测试使用 PyQt6 离屏平台，模拟串口确认，不会连接或启动真实电机。
反电动势测试是独立信号测试，寄存器测试为软件模型；不能代替实机带载验证。
