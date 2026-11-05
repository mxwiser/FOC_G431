# 电机模式与代码入口

上位机在停机状态选择算法，应用后以固件回报为准。上电默认仍由
`foc/sensorless_config.hpp` 中的 `FOC_USE_MXLEMMING` 决定；运行时切换只保存在 RAM。

| 模式 | 串口指令 | 算法入口 | 电流含义 |
| --- | --- | --- | --- |
| FOC · Ortega-lambda | `M=0\n` | `foc/ortega_lambda.hpp` | d/q 电流 |
| FOC · MXLEMMING | `M=1\n` | `foc/mxlemming_lambda.hpp` | d/q 电流 |
| BLDC · 六步换相 | `M=2\n` | `bldc/bldc.hpp` | 导通线电流 |

## 目录分工

- `setup.cpp`：ADC、串口、状态机、共同保护、速度 PI 和模式分派。
- `motor_mode.hpp`：稳定的模式编号。
- `foc/drive_impl.hpp`：FOC 电流环、SVPWM 输出和无感启动。
- `foc/parameter_detection_impl.hpp`：FOC 的 R/L/磁链辨识。
- `foc/foc_math.cpp`：坐标变换、观测器共同校验和 PLL，分派到两种独立观测器。
- `bldc/bldc.hpp`：无硬件依赖的六步表、反电动势换相、启动和线电流 PI。
- `bldc/drive_impl.hpp`：BLDC 控制与共同状态机之间的连接。
- `bldc/timer_output.hpp`：G431 三个桥臂的 PWM/强制低电平/悬空输出及 FOC 恢复。
- `bldc/phase_feedback.hpp`：板级 U/V/W 相电压采样接口。

`*_impl.hpp` 只由 `setup.cpp` 在私有命名空间中包含，不单独加入 Keil 编译组。
新增观测器分支也从原有 `foc_math.cpp` 编译，不需要更改 CubeMX 或工程文件列表。
嵌入式全部使用固定存储，无动态分配、虚函数或 C++ 容器。

## BLDC 移植范围与当前硬件缺口

参考本地 `bldc-master/motor/mcpwm.c` 的六步输出表和 `COMM_MODE_DELAY`：
换相后消隐，检测悬空相反电动势过零，延时约 30 电角度后换相。
六步表按本工程 A/B Clarke 正方向排列，正反转方向与 FOC 一致。
启动使用定向保持与强制换相，连续检测到有效反电动势才进入闭环。
线电流 PI 使用本工程的相 R/L 参数，外层复用速度 PI。
此次没有移植 VESC 的积分换相、霍尔/混合模式及全部参数检测流程。
来源许可见 `foc/VESC_LICENSE.txt`（GPL-3.0-or-later）。

**当前 `.ioc` 只采集三相电流和母线电压，尚无 U/V/W 相电压采样。**
因此可以切换选择 BLDC，但启动会返回 `cmd=6`，不会给电机施加六步驱动。
BLDC 的 `D` 参数辨识指令返回 `cmd=7`；需要辨识时先切回 FOC。

接通 BLDC 闭环前，需要先确定硬件上的 U/V/W ADC 引脚、分压比与零偏，
由用户在 CubeMX 中配置相电压采样。保留现有双 ADC 电流 DMA 的顺序和长度；
额外电压采样的 ADC 序列/触发资源需根据实际引脚确定。
相电压应在 PWM 有效导通平台采集，避开开关沿、死区和换相消隐区。
现有 CH4 的电流采样点位于 PWM 峰值附近，不能直接当作相电压的有效采样点。

板级采样回调在完成一次新的 U/V/W 采样并换算为对地电压（V）后调用：

```cpp
#include "bldc/phase_feedback.hpp"
bldc_publish_phase_voltages(voltage_u, voltage_v, voltage_w);
```

必须发布真实的新 ADC 结果，不能从占空比重建电压，也不能重复发布旧缓冲区。
发布函数内部记录当前电流 ADC 帧号；超过两帧未更新或输入无效，BLDC 停机并
锁存 `f=13`。接入采样后仍需实机验证相序、采样窗口、启动及换相参数。
本次没有修改 `.ioc`、`Core`、`Drivers` 或 Keil 工程，也没有烧录/启动真实电机。

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
python matlab/tests/test_startup_regression.py
python matlab/tests/test_adaptive_flux.py
python matlab/build_sensorless.py --output matlab/firmware_bldc_modes
python -m unittest discover -s matlab/current_ui/tests -p test_pyqt_protocol.py -v
python -m unittest discover -s matlab/current_ui/tests -p test_algorithm_ui.py -v
```

界面测试使用 PyQt6 离屏平台，模拟串口确认，不会连接或启动真实电机。
反电动势测试是独立信号测试，寄存器测试为软件模型；不能代替实机带载验证。
