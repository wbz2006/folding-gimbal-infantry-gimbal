# 折叠模式恢复说明

云台分支 `codex/restore-folded-mode` 直接从 `1a587a0175d02987c416cbb36974a03f72c4725b` 创建。修改位于独立 worktree `folding-gimbal_infantry-gimbal-restore-folded-mode`，原工作区保留。

配套底盘位于同级 `folding-gimbal_infantry-chassis-restore-folded-mode`，分支同名，基于底盘仓库 `206930470def97fb8a9529e6c391bc8f95f850d6`。它保留当前硬件配置，并恢复 `CHASSIS_FOLDED_ROTATE = 4` 的接收、限幅和 UI 显示。需要使用这套配套底盘代码，现有底盘版本会拒绝模式 4。

## 动作和输入

| 形态 | 云台行为 | 底盘行为 |
| --- | --- | --- |
| 折叠中 | 上 Pitch 调平并稳定 → Yaw 机械回中并稳定 → 下 Pitch 缓慢收纳 | 右摇杆逐周期平移，无旋转 |
| 折叠完成 | Yaw 零力矩，上 Pitch 调平，下 Pitch 保持收纳 | 固定模式 4；右摇杆平移，左横杆旋转，忽略右拨杆选模 |
| 展开中 | 上 Pitch 调平并稳定 → 下 Pitch 缓慢展开，Yaw 零力矩 | 右摇杆逐周期平移，无旋转 |
| 展开完成 | Yaw、上 Pitch 接受遥控目标，下 Pitch 保持展开 | 右摇杆逐周期平移，右拨杆选择原有模式 |

左下档请求折叠，左中档请求展开；左上档保留请求并停止遥控运动增量。右摇杆回中即清除平移命令。急停、失联及动作故障仍覆盖运动权限。过渡期间反向请求从下 Pitch 的实际位置重新开始；重复请求不重置超时。上 Pitch 偏离调平窗口时锁存下轴位置，恢复调平后继续顺序动作。

## 硬件和标定

- 三轴均为 DM4310：Yaw CAN2 `0x03/0x603`，上 Pitch CAN1 `0x04/0x604`，下 Pitch CAN1 `0x02/0x602`。
- 下 Pitch 收纳 `1.1 rad`、展开 `0 rad`；展开态上 Pitch 目标限制为 `-35°..25°`。
- Yaw 机械中位宏在 `application/robot_def.h`：`GIMBAL_YAW_MECHANICAL_CENTER_DEG = 0.0f`，按约定暂留占位值。实测云台与底盘正前方对齐时的 `yaw_motor_single_round_angle`，将其单圈角度值（degree，范围 `[0, 360)`）填入此宏。回中判定与底盘偏角计算共用该宏。
- 三轴启动均不发送编码器清零命令，保留固定标定参考。
- Yaw 角度/速度闭环使用 IMU `YawTotalAngle/Gyro[2]`，上 Pitch 使用 IMU `Pitch/Gyro[0]`；编码器用于 Yaw 机械回中误差和下 Pitch 位置反馈。Yaw 回中目标逐周期取 `IMU 连续角 - 编码器机械偏角`。
- Yaw、上 Pitch 角度 PID 按 degree 输入换算当前 rad 输入时的比例增益，保留当前输出限幅；Yaw 死区改为 `0.1°`，与 `1°` 回中窗口兼容。上 Pitch 使用绝对姿态反馈，不再叠加电机反馈方案的下轴角度补偿和速度前馈。
- 下轴参考速度沿用 `0.25 rad/s`，动作超时 `15 s`，稳定时间 `100 ms`；下轴到位容差沿用当前 `1.0 rad`。这些参数和实际轴向、机械位置、闭环响应仍需实机确认。

## 故障机制

从当前工作区移植了 CAN 状态/恢复、命令与反馈新鲜度、遥控帧校验、IMU 有效性、分类故障上报及其依赖；同步当前发射和运行配置。保留当前联调策略：反馈健康故障上报，运行中不新增失反馈动作锁存；首次动作等待三轴有效反馈，电机失能先停止，再每 `100 ms` 请求使能，确认后重新锁存实际位置。

无效或过期命令停止云台；控制所用 IMU 字段为非有限值时停止。动作超时锁存并停止，右拨杆上档发送零力矩命令确认后恢复。遥控失联和拨轮急停沿用当前行为。

## 验证

在云台 worktree 执行 `python tests/run_host_tests.py`：17 个测试程序全部通过，包括生产 CMD/Gimbal/DM 集成测试（机械中位分别为 `0°` 和 `123°`）、底盘模式 4、急停/超时/恢复、发射、遥控、消息队列、角度取模和两板协议检查。测试直接运行生产控制代码，硬件传输、时钟及 IMU 由测试夹具提供。

两个 worktree 均执行 `mingw32-make -j8 BUILD_DIR=build/restore_fold_verify` 并成功生成 ELF/HEX/BIN。两板共享声明与 ABI 一致；板间命令/反馈长度仍为 `24/65 bytes`。日志位于云台 `build/host_restore_fold_tests.log` 和各自 `build/build_restore_fold_verify.log`，固件位于各自 `build/restore_fold_verify/`。编译仍有现有未使用变量和链接 RWX 提示。

以上为主机与交叉编译验证，尚未烧录或进行实机动作验证。修改保留在新分支工作区，尚未提交。
