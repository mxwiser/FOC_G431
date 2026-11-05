# 电机模式与代码入口

上位机在停机状态选择算法，应用后以固件回报为准。上电默认仍由
`foc/sensorless_config.hpp` 中的 `FOC_USE_MXLEMMING` 决定；运行时切换只保存在 RAM。

| 模式 | 串口指令 | 算法入口 | 电流含义 |
| --- | --- | --- | --- |
| FOC · Ortega-lambda | `M=0\n` | `foc/ortega_lambda.hpp` | d/q 电流 |
| FOC · MXLEMMING | `M=1\n` | `foc/mxlemming_lambda.hpp` | d/q 电流 |
| BLDC · 六步换相 | `M=2\n` | `bldc/bldc.hpp` | 导通线电流 |

## 目录分工

- `setup.cpp`：调用各外设 `init()`、组织主循环和转发 HAL 回调。
- `motor_mode.hpp`：稳定的模式编号。
- `foc/motor_control.hpp`：控制模块对外入口；`initialize(period)` 初始化逻辑，`run()` 执行一次前台处理。
- `foc/motor_state.hpp`：控制状态与固定存储；`state_machine_impl.hpp` 管理启停、保护和模式分派。
- `foc/controllers_impl.hpp`：控制器初始化、速度环及电压裕量限流。
- `foc/adc_control_impl.hpp`：只消费 SI 输入帧，不访问 ADC DMA 缓冲或原始通道。
- `foc/drive_impl.hpp`：FOC 电流环、SVPWM 输出和无感启动。
- `foc/parameter_detection_impl.hpp`：FOC 的 R/L/磁链辨识。
- `foc/foc_math.cpp`：坐标变换、观测器共同校验和 PLL，分派到两种独立观测器。
- `foc/diagnostics_impl.hpp`：可选的电流环诊断记录。
- `app/serial_command.hpp`：上位机命令格式解析。
- `app/data_port.hpp` / `app/data_port.cpp`：无 HAL 依赖的收发接口、固定 RX 队列和错误事件。
- `app/communication_state.hpp`：协议状态和高速遥测的固定缓冲区。
- `app/host_protocol_impl.hpp`：命令执行、确认、OBS 状态和 CUR 高速电流协议。
- `board/uart_port_impl.hpp` / `board/uart_irq_impl.hpp`：STM32 UART2、DMA 与 HAL 回调，绑定到 data_port。
- `board/adc_port_impl.hpp`：ADC 初始化、DMA 缓冲、rank 映射、零点校准和 A/V 换算；`read()` 返回物理数值。
- `board/pwm_init.hpp` / `uart_init.hpp` / `system_init.hpp`：对应外设初始化与板级时钟/等待操作，由 `setup()` 调用。
- `bldc/bldc.hpp`：无硬件依赖的六步表、反电动势换相、启动和线电流 PI。
- `bldc/drive_impl.hpp`：BLDC 控制与共同状态机之间的连接。
- `bldc/timer_output.hpp`：G431 三个桥臂的 PWM/强制低电平/悬空输出及 FOC 恢复。
- `bldc/phase_feedback.hpp`：板级 U/V/W 相电压采样接口。

控制模块实现仅由现有 `setup.cpp` 编译单元包含一次，内部再按上述职责组织私有实现，
不单独把 `*_impl.hpp` 加入 Keil 编译组。
新增观测器分支也从原有 `foc_math.cpp` 编译，不需要更改 CubeMX 或工程文件列表。
嵌入式全部使用固定存储，无动态分配、虚函数或 C++ 容器。

## 可移植的数据接口

控制输入头文件 `foc/motor_control.hpp` 不包含 STM32/HAL 类型。每一帧使用：
`HAL_ADC_ConvCpltCallback` 调用 `board_adc::read()` 取得量化、校准后的 `float`，在回调中逐项调用下列 setter。
原始 DMA 采样和换算统一放在 `board/adc_port_impl.hpp`，控制输入函数不接收 ADC 缓冲区或 HAL 句柄。

```cpp
motor_control::set_phase_currents(ia, ib);        // 已校准电流，A；Ic=-Ia-Ib
motor_control::set_phase_voltages(vu, vv, vw);    // 对地相电压，V
motor_control::set_bus_voltage(vbus);            // 母线电压，V
motor_control::adc_complete();                   // 提交完整一帧并执行控制，无句柄
```

驱动错误调用 `motor_control::adc_error()`，也不传句柄。三个必需输入组都要在每帧提交，
缺少任一组会停止控制，避免静默复用旧输入。原始 ADC 零点、饱和和校准处理留在 `board`。
`set_encoder_angle(electrical_radians)` 是可选的电角度输入，单位 rad；当前三个模式仍为
无感模式，保存该输入不会自动切换到编码器控制。

串口适配层用 `data_port::bind(Driver)` 提供 TX 是否空闲、发送、恢复接收、临界区、
内存屏障和毫秒时钟。接收数据用 `receive_byte()` / `receive()`，接收错误用
`receive_error()`。应用协议不访问 `huart2` 或调用 HAL UART 函数，可换接其他传输驱动。

`app/data_port.cpp` 作为独立编译单元参与构建，`setup.cpp` 不包含 `.cpp` 文件。
当前 EIDE 工程会自动收集该源文件；其他构建系统也应将它加入源文件列表。
板级时钟、运放/ADC/DMA/串口/采样 PWM 启动和 `__WFI()` 位于 `board` 对应文件，由 `setup.cpp` 调用；
控制模块通过 data_port 时钟做前台超时处理。PWM 输出驱动仍需按目标板适配。

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

当前 PWM 关断续流区采样时，悬空相负电压会被 ADC 钳在零附近。BLDC 检测现在根据
两个导通桥臂的实测电压识别续流区，对钳位波形分别检测上升/下降边沿，再进行换相延时；
不再要求这种采样波形跨过一个无法测得的负阈值。导通区使用双极性反电动势。
启动接管时，在消隐结束后对有效的过零后半周累计延时，不强制要求先观察到过零前电平，
避免启动相位错开后下降沿扇区始终无法接管。仍需连续 12 次有效换相才报告锁定。
BLDC 的线电流 PI 由积分项建立反电动势所需电压，不在锁定瞬间叠加速度前馈，
避免重复补偿和换相周期抖动经前馈放大；FOC 电流环及观测器参数保持独立。

BLDC 的 `D` 参数辨识指令仍返回 `cmd=7`；辨识先切回 FOC。
相电压输入无效或停止发布时保留 `cmd=6` / `f=13` 的反馈保护。
换相后的前两个采样周期（当前 80 us）允许相电压短时到母线 +2 V，并继续消隐过零检测；
其他时间保持母线 +0.5 V 检查。2 V 是本板实测瞬态的余量，不是 VESC 默认值。
超出瞬态余量、持续越界、非有限值和反馈过期仍触发保护；首次越界帧保存在固定诊断记录中。
高转速时按 ADC 时基向下选择半扇区延时，避免向上取整多等待一整个 PWM 周期。
本次按现有配置解析采样，保留用户的 ADC/PWM 触发时序，没有修改 `.ioc`、`Core`、
`Drivers` 或 Keil 工程。BLDC 的实际相序、PWM 采样窗口和换相仍需实机验证，
主机信号测试不代表已经完成带载调试。

## 串口与上位机

- 使用 Python / PyQt6 上位机：`python matlab/current_ui/current_ui.py`。
- 串口工具栏下方的“控制算法”选择 Ortega-lambda / MXLEMMING / BLDC，点击“应用算法”。
- 未连接时也显示算法区域；修改 Python 代码后需退出旧窗口并重新运行。
- USART2：921600 / 8N1。
- 原有 `R`、`S`、`D`、`IQ=`、`V=`、`ILIM=`、`PH=` 保留。
- Iq、目标转速、速度模式限流值没有应用上限；固件与 Python 输入框均已取消 5 A/4000 rpm 限制。
  仍要求有限、可用固件 float 表示的数值；原有下限和零值停止语义保留。
  控制器跟踪用户目标，实际输出受母线电压/调制范围约束；过流、失锁等保护继续生效。
  `FOC_STARTUP_CURRENT_SCALE` 只用于 FOC 启动速度归一化，不限制电流指令。
- `M=` 仅允许 `IDLE`、无故障且没有自动重试意图时使用；其他状态返回 `cmd=4`。
- `OBS` 增加 `drv=0/1/2` 和 `bfb=0/1`，分别为实际模式和相电压反馈是否新鲜。
- `obs=O/M/B` 对应 Ortega、MXLEMMING、BLDC；BLDC 下 `id=0`，`iq` 为有符号线电流。
- 仍使用 `mode=I/S` 表示电流/速度控制，与算法选择 `drv` 分开。
- 速度环允许负转矩电流制动，FOC 和 BLDC 都受 `ILIM` 的正负幅值限制，电流变化率为 100 A/s。
  正向转矩仍受反电动势电压裕量限制；该降额不禁止负向制动。加减速参考斜坡均固定为
  VESC 默认的 25000 ERPM/s，当前 10 对极对应 2500 rpm/s，不再按低速分段降额。
  BLDC 未锁定的启动阶段保留 500 rpm/s，锁定运行后使用上述固定斜坡。
  降目标时，超出实际转速和新目标的旧参考会被裁剪，并按实际转矩电流约束正积分。
  FOC 电压限幅和 BLDC 最大占空比会反馈给速度环，避免不可达高目标导致积分继续增长。
  制动会向母线回送能量，默认母线高于 30 V 时触发保护；当前没有完整的电池回充电流/BMS 限制。
- 界面根据固件确认更新当前算法，运行时锁定切换，状态超时禁用控制；停止按钮保持可用。

## 验证入口

```text
python matlab/tests/test_motor_modes.py
python matlab/tests/test_data_ports.py
python matlab/tests/test_command_ranges.py
python matlab/tests/test_speed_braking.py
python matlab/tests/test_unreachable_speed_fix.py
python matlab/tests/test_vesc_speed_ramp.py
python matlab/tests/test_bldc_pwm_off.py
python matlab/tests/test_bldc_startup.py
python matlab/tests/test_bldc_commutation_transient.py
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
