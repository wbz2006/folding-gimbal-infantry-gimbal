# 达妙 DM 电机库说明

本文档对应 `dmmotor.c` 和 `dmmotor.h`，用于说明新版达妙电机库的控制模式、接口和使用约束。

## 设计目标

该库把达妙 MIT 报文发送能力和上层控制策略分开，支持每台电机独立选择控制模式：

1. `DMMOTOR_CONTROL_EXTERNAL_IMPEDANCE`：IMU 外部阻抗控制。
2. `DMMOTOR_CONTROL_MIT`：IMU 外环或轨迹规划 + DM 内部 MIT 位置速度环控制。

`DMMotorControl()` 只处理外部阻抗模式的电机，`DMMotorMITControl()` 只处理 MIT 模式的电机。
因此混合使用时，两个函数在同一个控制周期内各调用一次即可；同一台电机不会被两个入口重复发送。

控制模式保存在公开的 `DMMotorInstance::control_mode` 成员中，有两种配置方式：

```c
/* 方式一：初始化后直接配置实例成员。 */
lower_pitch_motor->control_mode = DMMOTOR_CONTROL_MIT;

/* 方式二：使用接口配置，适合运行过程中切换。 */
DMMotorSetControlMode(
    lower_pitch_motor,
    DMMOTOR_CONTROL_MIT);
```

两种方式效果相同。新注册的电机默认使用
`DMMOTOR_CONTROL_EXTERNAL_IMPEDANCE`。

## 数据单位

MIT 参数使用物理量配置，驱动内部负责限幅、编码和 CAN 打包。

| 字段 | 含义 | 单位 |
|---|---|---|
| `position_des` | 电机轴目标位置 | rad |
| `velocity_des` | 电机轴目标速度 | rad/s |
| `kp` | MIT 位置刚度 | 协议单位 |
| `kd` | MIT 速度阻尼 | 协议单位 |
| `torque_des` | 力矩前馈 | N*m |

反馈测量值中：

- `motor->measure.position` 为达妙协议解码得到的电机轴位置，单位为 `rad`；
- `motor->measure.angle_single_round` 为由 `position` 换算得到的单圈角度，单位为 `degree`，范围为 `[0, 360)`；
- 达妙反馈报文不包含 DJI 电机的原始 ECD 字段，因此 `angle_single_round` 不是由编码器计数直接换算得到；
- `angle_single_round` 的零点与达妙电机当前编码器零位一致，不能替代尚未实现的多圈角度 `total_round`。

注意：如果应用层使用 IMU 角度，通常是角度制或云台坐标，不能直接传给 `position_des`。应先完成：

```text
云台角度 -> 弧度 -> 减速比/传动比 -> 电机轴角度 -> 电机零位偏置
```

## 模式一：IMU 外部阻抗控制

调用入口：

```c
DMMotorControl();
```

该模式内部执行通用电机配置中的外部 PID 链：

```text
pid_ref
  -> 角度环 PID
  -> 速度环 PID
  -> 力矩/电流环 PID
  -> MIT torque 字段
```

发送的 MIT 报文被强制设为：

```c
p  = 0.0f;
v  = 0.0f;
kp = 0.0f;
kd = 0.0f;
tau = 外部PID输出 + motor->mit_config.torque_des;
```

因此该模式下 DM4310 内部位置/速度环不参与控制，云台刚性和阻尼由外部 IMU 角度环、速度环和前馈决定。

适用场景：

- 云台陀螺稳定；
- 抗底盘扰动；
- 使用 IMU 作为角度和角速度反馈；
- 不希望电机内部编码器位置环与 IMU 位置环互相对抗。

典型初始化配置：

```c
Motor_Init_Config_s pitch_config = {
    .can_init_config = {
        .can_handle = &hcan1,
        .tx_id = 0x06,
        .rx_id = 0x302,
    },
    .controller_param_init_config = {
        .angle_PID = {
            .Kp = 0.5f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .MaxOut = 10.0f,
        },
        .speed_PID = {
            .Kp = 0.3f,
            .Ki = 0.18f,
            .Kd = 0.0f,
            .MaxOut = 1.5f,
        },
        .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,
        .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[0],
        .current_feedforward_ptr = &gimbal_ff.pitch_current_ff,
    },
    .controller_setting_init_config = {
        .angle_feedback_source = OTHER_FEED,
        .speed_feedback_source = OTHER_FEED,
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        .feedforward_flag = CURRENT_FEEDFORWARD,
    },
    .motor_type = DM4310,
};

DMMotorInstance *pitch_motor = DMMotorInit(&pitch_config);
DMMotorSetRef(pitch_motor, target_pitch);
```

需要显式设置外部阻抗模式时，可以直接修改实例成员：

```c
pitch_motor->control_mode = DMMOTOR_CONTROL_EXTERNAL_IMPEDANCE;
```

控制任务中：

```c
void MotorControlTask(void)
{
    DMMotorControl();
}
```

如果需要重力补偿，可只配置力矩前馈：

```c
DMMotorSetMITRef(pitch_motor, 0.0f, 0.0f, 0.0f, 0.0f, gravity_torque);
```

在 `DMMotorControl()` 模式下，上面传入的 `p/v/kp/kd` 会被忽略并强制发送为 0，只有 `gravity_torque` 生效。

## 模式二：IMU 外环 + MIT 内部位置速度环

设置模式并配置参数：

```c
lower_pitch_motor->control_mode = DMMOTOR_CONTROL_MIT;

DMMotorSetMITRef(
    lower_pitch_motor,
    target_motor_position,
    target_motor_velocity,
    30.0f,
    0.8f,
    gravity_torque);
```

该模式不计算驱动内部的外部 PID 链，而是直接发送应用层配置的 MIT 参数：

```c
DMMotor_MIT_Config_s mit_config = {
    .position_des = target_motor_position,
    .velocity_des = target_motor_velocity,
    .kp = 30.0f,
    .kd = 0.8f,
    .torque_des = gravity_torque,
};

DMMotorSetMITConfig(lower_pitch_motor, &mit_config);
```

控制任务中：

```c
void MotorControlTask(void)
{
    /* 外部阻抗电机和 MIT 电机分别过滤处理。 */
    DMMotorControl();
    DMMotorMITControl();
}
```

适用场景：

- 机械臂式关节控制；
- 电机轴位置和云台角度传动关系明确；
- 希望利用 DM4310 内部位置刚度和速度阻尼；
- 应用层已经完成 IMU 外环或轨迹规划，只需要电机执行 `p/v/kp/kd/tau`。

使用该模式时，`position_des` 和 `velocity_des` 应为电机轴坐标，不是 IMU 原始角度。

## 两种模式如何选择

| 需求 | 推荐模式 |
|---|---|
| 云台稳定、抗扰动 | `DMMotorControl()` |
| IMU 角度/角速度闭环 | `DMMotorControl()` |
| 明确的电机轴位置伺服 | `DMMotorMITControl()` |
| 机械臂式刚性保持 | `DMMotorMITControl()` |
| 想先快速调通云台 | 先用 `DMMotorControl()` |
| 想利用 DM 内部刚度 | 再评估 `DMMotorMITControl()` |

不要在同一台电机上同时配置外部高增益 IMU 位置环和 MIT 内部高刚度位置环。
两种模式可以在同一个任务中混合，但每台电机只能选择其中一种模式。

## 停止和失联处理

`DMMotorStop()` 会设置停止标志。底层 `DMMotorSendMIT()` 检测到停止状态后，会发送：

```c
p = 0;
v = 0;
kp = 0;
kd = 0;
tau = 0;
```

`DMMotorLostCallback()` 在反馈超时时会清零 `pid_ref` 并置停止标志。后续周期控制函数会发送全零报文。

## 调参建议

### 外部阻抗模式

推荐顺序：

1. `MIT kp/kd` 保持 0。
2. 关闭前馈，只调速度环 `Kp`。
3. 增加角度环 `Kp`，提高姿态刚性。
4. 小幅增加速度环 `Ki`，消除静态误差。
5. 加入重力补偿力矩。
6. 最后加入速度或加速度前馈。

### MIT 内部模式

推荐顺序：

1. 切换模式前将 `position_des` 初始化为当前电机位置，避免跳变。
2. 从较小 `kp` 开始增加刚度。
3. 增加 `kd` 抑制振荡。
4. 确认零位、方向、减速比正确后再提高增益。
5. 逐步加入重力补偿或轨迹前馈。

## 常见错误

1. 同一周期重复调用 `DMMotorControl()` 或 `DMMotorMITControl()`。
2. 忘记为需要 MIT 控制的电机设置 `DMMOTOR_CONTROL_MIT`。
3. 把 IMU 角度直接当作 `position_des`。
4. 在外部 IMU 力矩闭环中同时打开大 `kp/kd`。
5. 把 DJI 电机的电流前馈数值直接当作 DM4310 力矩前馈。
6. 停止时只清力矩但保留 `kp/kd`，导致电机仍然保持位置。

## 文件接口摘要

```c
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);
void DMMotorSetRef(DMMotorInstance *motor, float ref);
void DMMotorSetControlMode(DMMotorInstance *motor, DMMotor_Control_Mode_e mode);
void DMMotorSetMITConfig(DMMotorInstance *motor, const DMMotor_MIT_Config_s *config);
void DMMotorSetMITRef(DMMotorInstance *motor, float p, float v, float kp, float kd, float tau_ff);
void DMMotorControl(void);
void DMMotorMITControl(void);
void DMMotorStop(DMMotorInstance *motor);
void DMMotorEnable(DMMotorInstance *motor);
```
