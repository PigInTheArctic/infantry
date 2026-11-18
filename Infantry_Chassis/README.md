# Rudder Infantry Chassis

基于 STM32F407 的力控舵轮步兵底盘控制工程。底盘采用双舵轮（RF、LB）驱动结构，通过舵向调节实现全向运动，并使用独立的偏航电机反馈完成云台坐标系与底盘坐标系之间的运动解耦。

本工程的核心特点不是简单地将速度 PID 输出直接映射为电机电流，而是先在车体层求解期望合力与合力矩，再通过动力学逆解分配到各驱动轮。针对坡面运动，控制器还会根据 IMU 姿态实时计算重力沿坡面的分量，并以前馈形式补偿，使速度闭环主要负责处理模型误差和扰动。

## 硬件与工具链

- 主控：STM32F407，系统时钟 168 MHz
- 惯性测量：BMI088，通过 SPI 读取加速度和角速度
- 电机：M3508 舵轮驱动电机、GM6020 舵向/偏航电机
- 通信：CAN1 用于偏航电机和板间通信，CAN2 用于舵轮电机与超级电容
- 软件：STM32CubeMX + CMake + GCC + OpenOCD，开发环境推荐 VS Code
- 实时系统：FreeRTOS，底盘控制任务周期约 2 ms（500 Hz）

## 目录结构

```text
├── User
│   ├── algorithm   # PID、卡尔曼滤波、AHRS、斜坡规划等算法
│   ├── Application # 底盘、INS、离线检测和任务接口
│   ├── bsp         # CAN、DWT、PWM、UART 等底层封装
│   ├── modules     # DJI 电机、BMI088、裁判系统、板间通信和超级电容
│   └── utils       # 矩阵、RLS、功率模型等工具
├── Inc             # CubeMX 生成的头文件
├── Src             # CubeMX 生成的 HAL、FreeRTOS 和中断代码
├── CMakeLists.txt
└── Rudder_Infantry_Chassis.ioc
```

## 核心算法特性

### 1. 基于动力学逆解的力控制

底盘控制采用“速度闭环—合力/合力矩—轮端力矩”的分层结构：

```text
遥控器速度指令
        ↓
坐标变换、模式控制、斜坡限制
        ↓
正运动学 + BMI088/陀螺仪速度融合
        ↓
      vx/vy/vw PID
        ↓
期望合力 Fx、Fy 与合力矩 τ
        ↓
舵轮动力学逆解
        ↓
轮端力矩前馈 + 轮速 PID
        ↓
M3508 电流输出
```

其中，速度 PID 的输出被解释为车体所需的合力和合力矩，而不是直接作为轮电机转速指令。对于每个舵轮，程序根据当前舵角将车体合力投影到轮子滚动方向，并叠加旋转运动产生的力矩需求：

\[
F_i = F_x\cos\alpha_i + F_y\sin\alpha_i
      - \frac{\tau}{L}\sin(\beta_i-\alpha_i)
\]

\[
\tau_i = rF_i
\]

其中 `αᵢ` 为舵轮当前角度，`L` 为安装几何尺寸，`r` 为轮半径。得到的轮端力矩通过 M3508 的力矩—输出量标定系数转换为电机输出，同时保留轮速 PID 作为动态误差修正：

\[
u_i = \tau_i K_{\tau\rightarrow out} + u_{\mathrm{speed},i}
\]

这种结构能够在舵角变化、平移与旋转同时发生时保持力分配的一致性，减少单纯速度控制在复杂运动中的耦合和响应滞后。

### 2. 坡面重力力前馈补偿

坡面运动是本工程最具辨识度的控制特性。程序从 AHRS 输出的四元数恢复重力方向，计算底盘倾斜角 `slope_ang_` 与坡面朝向 `slope_toward_ang_`。当坡度超过 5° 时，计算需要抵消的重力切向分量：

\[
F_g = mg\sin(\theta)
\]

再将其分解到车体坐标系：

\[
\begin{aligned}
F_{gx} &= F_g\cos(\phi) \\
F_{gy} &= F_g\sin(\phi)
\end{aligned}
\]

控制器最终使用：

\[
\mathbf F_{\mathrm{total}}
 = \mathbf F_{\mathrm{PID}} + \mathbf F_g
\]

重力补偿在动力学逆解之前加入，因此补偿力会按照实时舵角自然投影到 RF、LB 两个轮子的滚动方向，而不是为每个电机单独设置固定的补偿电流。这样做有三个直接收益：

- 上坡启动时提前提供抗重力驱动力，降低速度 PID 积分累积和启动迟滞。
- 下坡或坡向变化时，补偿方向随 IMU 姿态实时变化，避免固定方向补偿失效。
- 舵轮转向时，重力补偿仍经过统一动力学分配，减少两轮受力不均和车体偏航。

低速或停止时，如果检测到坡度，舵轮目标角会对准坡面方向；平地则保持当前舵角。该策略可以避免静止时舵轮因微小速度指令抖动，同时为坡面起步准备合理的受力方向。

### 3. 多源速度融合

底盘正运动学由两只驱动轮速度和舵角计算 `vx`、`vy`、`vw`，角速度同时使用 BMI088 陀螺仪测量值。四状态卡尔曼滤波器融合轮速、陀螺仪和 IMU 线加速度，并使用 DWT 实时更新采样周期，提高低速、颠簸和轮胎打滑情况下的速度估计稳定性。

### 4. 舵轮最近角与模式控制

- 对每个舵轮执行最近角换向：当目标舵角超过 ±90° 时，舵角增加 π 并反向轮速，减少舵机大角度转动。
- 支持不动、跟随云台偏航、斜向跟随和自旋模式。
- 跟随模式使用偏航角闭环，并设置 0.1 rad 死区；速度指令经过斜坡限制，降低启动和换向冲击。
- 自旋模式下关闭轮端力矩前馈，使用轮速闭环完成连续旋转。

### 5. 电机与功率保护

PID 控制器支持积分限幅、微分先行、微分低通滤波、变积分速率和输出限幅，适用于速度环、舵角环和舵速环。代码中还实现了基于电机转速、电流的功率拟合模型，以及结合裁判系统功率限制和超级电容状态的功率反解算法。

## 任务结构

```text
FreeRTOS
├── ChassisTask   # 初始化并运行底盘控制，2 ms 周期
├── INS_Task      # BMI088 采样、偏置处理和 AHRS 更新
├── UI_Task       # 裁判系统/UI 相关处理
├── DetectTask    # 电机、裁判系统和超级电容离线检测
└── DefaultTask   # 系统基础任务
```

底盘主控制入口为 `User/Application/src/chassis.cpp`，配置和 C 接口封装位于 `User/Application/src/infantry.cpp`。修改电机 CAN ID、减速比和零位偏置时，优先检查 `infantry.cpp` 中的 `ChassisConfig`。

## 构建

确保已安装 ARM GCC、CMake、Ninja 和 OpenOCD：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

发布版本：

```bash
cmake --preset Release
cmake --build --preset Release
```

## 调参提示

1. 先校准 BMI088 的零偏、重力标定值以及各电机舵角零位。
2. 在平地关闭或减小坡面补偿，先完成舵角环、舵速环和轮速环调试。
3. 确认轮端力矩方向、M3508 减速比和力矩换算系数正确后，再逐步增加速度环增益。
4. 在不同坡度下观察 `slope_ang_`、`slope_toward_ang_` 和重力补偿输出，检查补偿方向及整车质量参数 `wholeWeight`。

### ESO 使用

ESO 已合并到 `ADRC.h/.cpp` 中，但仍保留为独立的 `LESO` 和 `NLESO` 两个观测器类，可单独使用。两者均通过 `Update(feedback, control)` 更新，并通过 `GetZ1()`、`GetZ2()`、`GetZ3()` 读取估计状态；采样周期由 DWT 自动获取。

`NLESO` 使用 `beta1`、`beta2`、`beta3` 和 `delta` 参数，`LESO` 使用观测器带宽自动生成三阶线性观测器增益。ADRC 的 `Init` 末尾增加 `ESOType::NLESO` 或 `ESOType::LESO` 选择，省略时保持原有 NLESO 行为；选择 LESO 时通过最后一个 `leso_bandwidth` 参数设置带宽。

## 参考项目与致谢

本工程在设计和实现过程中参考了以下开源项目，感谢相关作者和团队的分享：

- [Rudder-Swerve-Control](https://github.com/LYHrmer/Rudder-Swerve-Control)：参考其舵轮底盘的功率控制、功率模型拟合以及动力学解算思路。
- [6-Axis-AHRS](https://github.com/liuskywalkerjskd/6-Axis-AHRS)：参考其 IMU 姿态解算算法，用于 BMI088 数据处理和 AHRS 更新。
- [RM2024_Reborn_LegWheel_Chassis](https://github.com/JerryGong0911/RM2024_Reborn_LegWheel_Chassis)：参考其工程组织方式、CMake/FreeRTOS 软件架构和模块化设计思路。

本工程结合自身硬件连接、双舵轮结构和控制需求进行了适配、重构与参数调整，具体实现以仓库中的源代码为准。
