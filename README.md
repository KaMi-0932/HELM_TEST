# Chassis

底盘控制模块：麦轮、全向轮与舵轮底盘的速度闭环、功率限制与模式切换 / Chassis control Module with velocity loops, power limiting and mode switching for mecanum, omnidirectional and swerve (helm) chassis

## 1. 模块作用 / Purpose

`Chassis<ChassisType>` 是一个模板外壳：模板参数 `ChassisType` 选择具体的底盘实现（`Mecanum` 麦轮、`Omni` 全向轮、`Helm` 舵轮，分别位于 `Mecanum.hpp`、`Omni.hpp`、`Helm.hpp`），外壳转发构造参数并通过 `GetEvent()` 提供模式切换事件。

构造时，所选底盘创建控制线程（`MecanumChassisThread`、`OmniChassisThread` 或 `HelmChassisThread`），栈深为 `task_stack_depth`，优先级为 `param.thread_priority`，周期 2 ms。

模式切换：`GetEvent()` 返回的 `LibXR::Event` 注册了 `ChassisType::ChassisMode` 的各个值，激活对应的事件 ID 即切换模式并复位速度相关的 PID，通常由 `EventBinder` 把遥控器事件绑定到这些 ID。`Mecanum` 与 `Omni` 在 CMD 的 `CMD_EVENT_LOST_CTRL` 与 `CMD_EVENT_START_CTRL` 时切换到 `RELAX`。`RELAX` 模式下所有电机执行 `Relax()`。

控制链（`Mecanum`、`Omni`）：由 `chassis_cmd` 的 x、y、z 得到目标速度，经逆运动学得到各轮目标角速度；轮速 PID 的输出加上速度环与角速度环 PID 得到的前馈力，经 PowerControl 限幅后，以 `MODE_TORQUE` 下发给轮电机，力矩限幅 ±6 N·m。`Helm` 由目标速度解算各轮舵角与轮速，采用低速舵角保持和 cos³ 轮速补偿。正运动学反馈进入车体速度 PID，输出加速度并按质量、转动惯量得到合力和偏航力矩请求。DAQP 将实际舵角、轮速 PID 修正、驱动轮功率和输出约束一起纳入加权分配，再经原功率限幅以 `MODE_TORQUE` 下发。舵向采用角度环加速度环，驱动轮力矩限幅 ±4.4 N·m，舵力矩限幅 ±2.5 N·m。Helm 功率预算仅限制驱动轮，舵向输出保持不变。

功率上限：裁判系统 `chassis_ref` 在 1 s 内有数据且 `chassis_power_limit` 大于 1 W 时使用该值，否则使用默认值（`Mecanum` 70 W，`Omni` 与 `Helm` 100 W）。`chassis_cmd` 的 `self_define` 为 `CMD::ChasStat::BOOST` 且超级电容在线时，按电容能量大于 0.25、0.5、0.8 分别额外增加功率上限：`Mecanum` 与 `Omni` 为 100、200、300 W，`Helm` 为 40、70、100 W。限幅由 `PowerControl` 完成。

小陀螺（`ROTOR`）转速缩放（`Mecanum`、`Omni`）：平移输入越大，按 `rotor_speed_scale` 降低的转速越多；另按裁判系统缓冲能量（在 `rotor_buffer_low_j` 与 `rotor_buffer_high_j` 之间线性映射）和功率受限程度动态缩放，下限为 `rotor_omega_min_scale`，经系数为 `rotor_scale_lpf_alpha` 的一阶低通。

`Chassis<ChassisType>` is a template shell: the template parameter `ChassisType` selects the chassis implementation (`Mecanum`, `Omni` or `Helm`, located in `Mecanum.hpp`, `Omni.hpp` and `Helm.hpp`). The shell forwards the constructor parameters and provides the mode-switching events through `GetEvent()`.

Upon construction, the selected chassis creates its control thread (`MecanumChassisThread`, `OmniChassisThread` or `HelmChassisThread`) with stack depth `task_stack_depth`, priority `param.thread_priority` and a period of 2 ms.

Mode switching: the `LibXR::Event` returned by `GetEvent()` has every value of `ChassisType::ChassisMode` registered. Activating the corresponding event ID switches the mode and resets the velocity-related PIDs; an `EventBinder` usually binds remote-controller events to these IDs. `Mecanum` and `Omni` switch to `RELAX` on the CMD events `CMD_EVENT_LOST_CTRL` and `CMD_EVENT_START_CTRL`. In `RELAX` mode all motors execute `Relax()`.

Control chain (`Mecanum`, `Omni`): the x, y and z of `chassis_cmd` give the target velocity, and inverse kinematics gives the target angular velocity of each wheel. The wheel-speed PID output plus the feedforward force from the velocity and angular-velocity PIDs is limited by PowerControl and sent to the wheel motors in `MODE_TORQUE`, with a torque limit of ±6 N·m. `Helm` uses forward and inverse kinematics, low-speed steering hold and cos³ wheel-speed compensation. Chassis velocity PIDs produce accelerations; mass and inertia give a requested wrench. DAQP allocates final wheel torques using actual steering angles, wheel PID preferences, drive power and actuator bounds, followed by the existing power guard. Steering uses an angle loop plus a speed loop. Outputs use `MODE_TORQUE`, limited to ±4.4 N·m for drive wheels and ±2.5 N·m for steering motors. The Helm power budget limits only the drive wheels; steering output is passed through.

Power limit: when the referee `chassis_ref` has delivered data within 1 s and `chassis_power_limit` is greater than 1 W, that value is used; otherwise the default applies (`Mecanum` 70 W, `Omni` and `Helm` 100 W). When `self_define` of `chassis_cmd` is `CMD::ChasStat::BOOST` and the supercapacitor is online, the limit is raised for capacitor energy above 0.25, 0.5 and 0.8: by 100, 200 and 300 W for `Mecanum` and `Omni`, and by 40, 70 and 100 W for `Helm`. `PowerControl` performs the limiting.

Spin (`ROTOR`) speed scaling (`Mecanum`, `Omni`): the larger the translation input, the more the speed is reduced according to `rotor_speed_scale`. The speed is also scaled dynamically by the referee buffer energy (mapped linearly between `rotor_buffer_low_j` and `rotor_buffer_high_j`) and by how strongly the power is limited, with a lower bound of `rotor_omega_min_scale`, and passes a first-order low-pass filter with coefficient `rotor_scale_lpf_alpha`.

Helm 的功率约束分配、权重、回退与普通成员观测值详见 [HelmPowerAllocation.md](HelmPowerAllocation.md)。 / See [HelmPowerAllocation.md](HelmPowerAllocation.md) for the Helm allocator, priorities, fallback and diagnostics.

## 2. 底盘类型 / Chassis Types

Helm 的普通成员功率观测值（单位 W，无需超级电容在线）：

- `helm_chassis.chassis_.wheel_output_power_estimated_w_`：最终 Helm 驱动轮力矩命令换算的电流与反馈转速计算的模型正功率总和，在功率限幅和最终力矩/电流限幅之后统计。
- `helm_chassis.chassis_.wheel_feedback_power_w_`：四个驱动轮反馈力矩换算的电流与反馈转速计算的模型正功率总和；电机 `reverse` 的反馈力矩方向已与转速统一。

两者均逐轮取 `max(0, P)` 再求和，不用制动回馈抵扣其他轮消耗，不计舵向电机或底盘静态损耗，也不是电源端实测功率。输出估计依据 Helm 命令，电机驱动器保护、CAN 延迟和实际电流跟踪差异需结合反馈估计判断。RELAX 时输出估计使用零电流，若轮子仍在转，模型的转速平方损耗项仍可能非零；反馈无效时观测值清零。

| | `Mecanum` | `Omni` | `Helm` |
| --- | --- | --- | --- |
| 模式 `ChassisMode` | `RELAX`、`INDEPENDENT`、`ROTOR`、`FOLLOW`、`TRACK_START` | `RELAX`、`INDEPENDENT`、`ROTOR`、`FOLLOW` | `RELAX`、`INDEPENDENT`、`ROTOR`、`FOLLOW` |
| `motor_steer_0` | 履带电机，可为 `nullptr` | `nullptr` | 舵向电机 0，非空 |
| `motor_steer_1..3` | `nullptr` | `nullptr` | 舵向电机 1..3，非空 |
| 使用的 PID | `pid_follow`、`pid_velocity_x/y`、`pid_omega`、`pid_wheel_speed_0..3`；`pid_steer_angle_0` 为履带速度 PID | `pid_follow`、`pid_velocity_x/y`、`pid_omega`、`pid_wheel_speed_0..3` | `pid_follow`、`pid_velocity_x/y`、`pid_omega`、`pid_wheel_speed_0..3`、`pid_steer_angle_0..3`、`pid_steer_speed_0..3` |
| 使用的 `chassis_param` | `wheel_radius`、`wheel_to_center`、`reduction_ratio`、`rotor_*` | 与 `Mecanum` 相同，另有 `length`、`width`、`gravity_height`、`gravity`（姿态前馈） | `wheel_radius`、`wheel_to_center`、`reduction_ratio`、`gravity`、`moment_of_inertia` |
| 裁判系统 UI | 定时任务每 52 ms 绘制模式字符、辅助线与电容能量条 | 定时任务每 100 ms 绘制模式、AI 状态与电容能量 | 无 |

`Mecanum` 与 `Omni` 的轮序（x 向右，y 向前）：wheel0 左前，wheel1 左后，wheel2 右后，wheel3 右前。`Mecanum` 的 `TRACK_START` 模式由履带（`motor_steer_0`）负责前后，麦轮提供横移与辅助，履带目标速度带斜坡，功率分配时优先分配给履带。`Helm` 的舵向零点取自 `Helm.hpp` 中的 `zero_` 数组。

| | `Mecanum` | `Omni` | `Helm` |
| --- | --- | --- | --- |
| Modes `ChassisMode` | `RELAX`, `INDEPENDENT`, `ROTOR`, `FOLLOW`, `TRACK_START` | `RELAX`, `INDEPENDENT`, `ROTOR`, `FOLLOW` | `RELAX`, `INDEPENDENT`, `ROTOR`, `FOLLOW` |
| `motor_steer_0` | Track motor, may be `nullptr` | `nullptr` | Steering motor 0, non-null |
| `motor_steer_1..3` | `nullptr` | `nullptr` | Steering motors 1..3, non-null |
| PIDs used | `pid_follow`, `pid_velocity_x/y`, `pid_omega`, `pid_wheel_speed_0..3`; `pid_steer_angle_0` is the track speed PID | `pid_follow`, `pid_velocity_x/y`, `pid_omega`, `pid_wheel_speed_0..3` | `pid_follow`, `pid_velocity_x/y`, `pid_omega`, `pid_wheel_speed_0..3`, `pid_steer_angle_0..3`, `pid_steer_speed_0..3` |
| `chassis_param` used | `wheel_radius`, `wheel_to_center`, `reduction_ratio`, `rotor_*` | Same as `Mecanum`, plus `length`, `width`, `gravity_height`, `gravity` (attitude feedforward) | `wheel_radius`, `wheel_to_center`, `reduction_ratio`, `gravity`, `moment_of_inertia` |
| Referee UI | Timer task every 52 ms draws the mode text, guide lines and capacitor energy bar | Timer task every 100 ms draws the mode, AI status and capacitor energy | None |

Wheel numbering of `Mecanum` and `Omni` (x to the right, y forward): wheel0 front left, wheel1 rear left, wheel2 rear right, wheel3 front right. In the `TRACK_START` mode of `Mecanum` the track (`motor_steer_0`) drives forward and backward while the mecanum wheels provide lateral motion and assistance; the track target speed has a ramp, and the power allocation favors the track. The steering zero points of `Helm` come from the `zero_` array in `Helm.hpp`.

## 3. 构造接口 / Constructor

```cpp
template <typename ChassisType>
class Chassis;

Chassis(Motor& motor_wheel_0, Motor& motor_wheel_1, Motor& motor_wheel_2,
        Motor& motor_wheel_3, Motor* motor_steer_0, Motor* motor_steer_1,
        Motor* motor_steer_2, Motor* motor_steer_3, CMD& cmd,
        PowerControl& power_control, Referee& referee, uint32_t task_stack_depth = 1536,
        const Param& param = {...});  // 节选 / excerpt
```

模板参数：

- `ChassisType`：`Mecanum`、`Omni` 或 `Helm`，见第 2 节。

依赖：

- `motor_wheel_0..3`：`Motor`，四个驱动轮电机，例如 `RMMotor` 实例。
- `motor_steer_0..3`：`Motor*`，舵向电机或履带电机，按底盘类型填写，不使用的填 `nullptr`。
- `cmd`：`CMD` 实例，提供控制命令、控制模式与 CMD 事件。
- `power_control`：`PowerControl` 实例，提供功率限制与电容能量。
- `referee`：`Referee` 实例，提供裁判系统数据与 UI 绘制。

配置参数：

- `task_stack_depth`：控制线程栈深，默认 1536。
- `param.chassis_param`：底盘几何与动力参数，默认值见下，默认为 0 的字段取实车数值：
  - `wheel_radius`：轮半径，单位 m；
  - `wheel_to_center`：轮心到底盘中心的距离，单位 m；
  - `moment_of_inertia`：Helm 绕底盘竖直轴的转动惯量，单位 kg·m²，默认 0，表示关闭旋转力矩前馈；
  - `reduction_ratio`：轮电机减速比；
  - `wheel_resistance`：轮阻；
  - `error_compensation`：误差补偿；
  - `gravity_height`：质心高度，单位 m，用于 `Omni` 姿态前馈；
  - `gravity`：底盘重力，单位 N，用于 `Omni` 姿态前馈；
  - `length`、`width`：底盘轮距的长与宽，单位 m，用于 `Omni` 姿态前馈；
  - `rotor_speed_scale`：平移输入下的小陀螺转速缩放，默认 1.0；
  - `rotor_omega_min_scale`：小陀螺动态缩放的下限，默认 0.55；
  - `rotor_buffer_low_j`、`rotor_buffer_high_j`：缓冲能量的低、高阈值，单位 J，默认 35 与 70；
  - `rotor_scale_lpf_alpha`：动态缩放的一阶低通系数，默认 0.2。
- `param.pid_follow`、`pid_velocity_x`、`pid_velocity_y`、`pid_omega`、`pid_wheel_speed_0..3`、`pid_steer_angle_0..3`、`pid_steer_speed_0..3`：`LibXR::PID<float>::Param`，字段为 `k`、`p`、`i`、`d`、`i_limit`、`out_limit`、`cycle`，默认 `k = 1.0`，其余为 0，`cycle = false`。
- `param.thread_priority`：控制线程优先级，默认 `LibXR::Thread::Priority::HIGH`。
- `param.chassis_cmd_topic_name`：订阅的底盘控制命令 Topic 名称，默认 `"chassis_cmd"`。
- `param.chassis_ref_topic_name`：订阅的裁判系统底盘数据 Topic 名称，默认 `"chassis_ref"`。
- `param.gimbal_euler_topic_name`：订阅的云台欧拉角 Topic 名称，默认 `"gimbal_euler"`，由 `Omni` 使用。

Template parameter:

- `ChassisType`: `Mecanum`, `Omni` or `Helm`, see section 2.

Dependencies:

- `motor_wheel_0..3`: `Motor` objects for the four drive wheels, for example `RMMotor` instances.
- `motor_steer_0..3`: `Motor*`, steering or track motors; filled according to the chassis type, with `nullptr` for the unused ones.
- `cmd`: the `CMD` instance, providing the control command, the control mode and the CMD events.
- `power_control`: the `PowerControl` instance, providing power limiting and capacitor energy.
- `referee`: the `Referee` instance, providing the referee data and the UI drawing.

Configuration parameters:

- `task_stack_depth`: control thread stack depth, default 1536.
- `param.chassis_param`: chassis geometry and dynamics parameters, defaults below; fields defaulting to 0 take the values of the machine:
  - `wheel_radius`: wheel radius in m;
  - `wheel_to_center`: distance from the wheel center to the chassis center in m;
  - `moment_of_inertia`: Helm vertical-axis inertia in kg·m²; default 0 disables angular torque feedforward;
  - `reduction_ratio`: reduction ratio of the wheel motors;
  - `wheel_resistance`: wheel resistance;
  - `error_compensation`: error compensation;
  - `gravity_height`: center-of-mass height in m, used by the `Omni` attitude feedforward;
  - `gravity`: chassis weight in N, used by the `Omni` attitude feedforward;
  - `length`, `width`: length and width of the wheel layout in m, used by the `Omni` attitude feedforward;
  - `rotor_speed_scale`: spin speed scale under translation input, default 1.0;
  - `rotor_omega_min_scale`: lower bound of the dynamic spin scale, default 0.55;
  - `rotor_buffer_low_j`, `rotor_buffer_high_j`: low and high buffer energy thresholds in J, default 35 and 70;
  - `rotor_scale_lpf_alpha`: first-order low-pass coefficient of the dynamic scale, default 0.2.
- `param.pid_follow`, `pid_velocity_x`, `pid_velocity_y`, `pid_omega`, `pid_wheel_speed_0..3`, `pid_steer_angle_0..3`, `pid_steer_speed_0..3`: `LibXR::PID<float>::Param` with fields `k`, `p`, `i`, `d`, `i_limit`, `out_limit`, `cycle`; default `k = 1.0`, the others 0 and `cycle = false`.
- `param.thread_priority`: control thread priority, default `LibXR::Thread::Priority::HIGH`.
- `param.chassis_cmd_topic_name`: name of the subscribed chassis command Topic, default `"chassis_cmd"`.
- `param.chassis_ref_topic_name`: name of the subscribed referee chassis data Topic, default `"chassis_ref"`.
- `param.gimbal_euler_topic_name`: name of the subscribed gimbal Euler angle Topic, default `"gimbal_euler"`, used by `Omni`.

## 4. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `param.chassis_cmd_topic_name`（默认 `chassis_cmd`） | 订阅 | `CMD::ChassisCMD` | CMD 发布的底盘控制命令 |
| `param.chassis_ref_topic_name`（默认 `chassis_ref`） | 订阅 | `Referee::ChassisPack` | 裁判系统的功率上限与缓冲能量 |
| `yawmotor_angle` | 订阅 | `float` | Gimbal 发布的 yaw 电机相对零点的角度，用于 `FOLLOW` 与坐标变换 |
| `param.gimbal_euler_topic_name`（默认 `gimbal_euler`） | 订阅 | `LibXR::EulerAngle<float>` | `Omni`：云台姿态，用于姿态前馈 |
| `pitchmotor_angle` | 订阅 | `float` | `Omni`：Gimbal 发布的 pitch 电机角度，用于姿态前馈 |
| `delta_yaw` | 发布 | `float` | `Helm`：每个控制周期发布，值为 `yawmotor_angle` 归一化后取反 |

订阅的 Topic 名称与发布方实例使用的名称相同：`yawmotor_angle` 与 `pitchmotor_angle` 由 Gimbal 以固定名称发布。

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `param.chassis_cmd_topic_name` (default `chassis_cmd`) | Subscribe | `CMD::ChassisCMD` | Chassis control command published by CMD |
| `param.chassis_ref_topic_name` (default `chassis_ref`) | Subscribe | `Referee::ChassisPack` | Power limit and buffer energy from the referee system |
| `yawmotor_angle` | Subscribe | `float` | Yaw motor angle relative to its zero point, published by Gimbal; used by `FOLLOW` and the coordinate transform |
| `param.gimbal_euler_topic_name` (default `gimbal_euler`) | Subscribe | `LibXR::EulerAngle<float>` | `Omni`: gimbal attitude, used for the attitude feedforward |
| `pitchmotor_angle` | Subscribe | `float` | `Omni`: pitch motor angle published by Gimbal, used for the attitude feedforward |
| `delta_yaw` | Publish | `float` | `Helm`: published every control cycle, the negated, normalized `yawmotor_angle` |

The subscribed Topic names match the names used by the publishing instances: `yawmotor_angle` and `pitchmotor_angle` are published by Gimbal under these fixed names.

## 5. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/Chassis --template-arg Omni` 写入的实例，`template_args` 为底盘类型（本例为 `Omni`），依赖填写为其他模块实例的 id，几何参数与 PID 取自一份全向轮步兵的实机配置：

An instance written by `xrobot instance add QDU-Robomaster/Chassis --template-arg Omni`, with `template_args` holding the chassis type (`Omni` in this example), the dependencies set to the ids of other Module instances, and the geometry and PID values taken from an omnidirectional-wheel infantry configuration:

```yaml
modules:
  - module: QDU-Robomaster/Chassis
    id: chassis
    template_args:
      - Omni
    args:
      - motor_wheel_0: motor_wheel_0
      - motor_wheel_1: motor_wheel_1
      - motor_wheel_2: motor_wheel_2
      - motor_wheel_3: motor_wheel_3
      - motor_steer_0: nullptr
      - motor_steer_1: nullptr
      - motor_steer_2: nullptr
      - motor_steer_3: nullptr
      - cmd: cmd
      - power_control: power_control
      - referee: ref
      - task_stack_depth: 1536
      - param:
          chassis_param:
            wheel_radius: 0.065f
            wheel_to_center: 0.26f
            gravity_height: 0.2f
            reduction_ratio: 15.7647f
            wheel_resistance: 0.0f
            error_compensation: 0.0f
            gravity: 230.0f
            length: 0.3f
            width: 0.3f
            rotor_speed_scale: 1.0f
            rotor_omega_min_scale: 0.55f
            rotor_buffer_low_j: 35.0f
            rotor_buffer_high_j: 70.0f
            rotor_scale_lpf_alpha: 0.3f
            moment_of_inertia: 0.0f
          pid_follow: '{.p = 10.0, .cycle = true}'
          pid_velocity_x: '{.p = 300.0}'
          pid_velocity_y: '{.p = 300.0}'
          pid_omega: '{.p = 300.0}'
          pid_wheel_speed_0: '{.p = 0.05, .out_limit = 5.0}'
          pid_wheel_speed_1: '{.p = 0.05, .out_limit = 5.0}'
          pid_wheel_speed_2: '{.p = 0.05, .out_limit = 5.0}'
          pid_wheel_speed_3: '{.p = 0.05, .out_limit = 5.0}'
          pid_steer_angle_0: '{}'
          pid_steer_angle_1: '{}'
          pid_steer_angle_2: '{}'
          pid_steer_angle_3: '{}'
          pid_steer_speed_0: '{}'
          pid_steer_speed_1: '{}'
          pid_steer_speed_2: '{}'
          pid_steer_speed_3: '{}'
          thread_priority: LibXR::Thread::Priority::HIGH
          chassis_cmd_topic_name: "chassis_cmd"
          chassis_ref_topic_name: "chassis_ref"
          gimbal_euler_topic_name: "gimbal_euler"
```

`motor_wheel_0..3` 取自 `QDU-Robomaster/RMMotor` 实例，`cmd` 取自 `QDU-Robomaster/CMD` 实例，`power_control` 取自 `QDU-Robomaster/PowerControl` 实例，`referee` 取自 `QDU-Robomaster/Referee` 实例，它们须在本实例之前列出。`Omni` 的 `motor_steer_0..3` 为 `nullptr`；带履带的 `Mecanum` 把 `motor_steer_0` 填为履带电机的实例或 `XR_REGISTER` 注册的对象；`Helm` 把 `motor_steer_0..3` 都填为舵向电机。`motor_steer_0..3` 是指针依赖，写成 `'&id'`。PID 为 C++ 表达式，使用指定初始化器，省略的字段取默认值。

`motor_wheel_0..3` are taken from `QDU-Robomaster/RMMotor` instances, `cmd` from a `QDU-Robomaster/CMD` instance, `power_control` from a `QDU-Robomaster/PowerControl` instance and `referee` from a `QDU-Robomaster/Referee` instance; they are listed before this instance. For `Omni`, `motor_steer_0..3` are `nullptr`; a `Mecanum` with a track sets `motor_steer_0` to the instance or the `XR_REGISTER` object of the track motor; `Helm` sets all of `motor_steer_0..3` to the steering motors. `motor_steer_0..3` are pointer dependencies written as `'&id'`. The PIDs are C++ expressions written as designated initializers, and omitted fields take their defaults.

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/Motor`：轮电机与舵向电机的接口。
- `QDU-Robomaster/RMMotor`：RoboMaster 电机的量程常量。
- `QDU-Robomaster/CMD`：控制命令类型与 CMD 事件。
- `QDU-Robomaster/PowerControl`：功率限制与电容能量。
- `QDU-Robomaster/Referee`：裁判系统数据类型与 UI 绘制。
- `QDU-Robomaster/SuperPower`：超级电容电源模块，`Mecanum.hpp` 包含其头文件。
- `xrobot-org/BMI088`、`xrobot-org/MadgwickAHRS`：云台 IMU 与姿态解算，`Omni` 订阅的云台姿态 Topic 由该类实例发布。
- LibXR。

硬件：四个驱动轮电机（功率模型按 M3508 的电流与扭矩比例计算）；`Helm` 另有四个舵向电机（按 GM6020 的比例计算），`Mecanum` 可选一个履带电机；超级电容（PowerControl 提供电容能量）与裁判系统；云台提供 `yawmotor_angle`，`Omni` 另需云台姿态与 `pitchmotor_angle`。

Dependencies:

- `QDU-Robomaster/Motor`: interface of the wheel and steering motors.
- `QDU-Robomaster/RMMotor`: range constants of the RoboMaster motors.
- `QDU-Robomaster/CMD`: control command types and CMD events.
- `QDU-Robomaster/PowerControl`: power limiting and capacitor energy.
- `QDU-Robomaster/Referee`: referee data types and UI drawing.
- `QDU-Robomaster/SuperPower`: supercapacitor power Module, whose header is included by `Mecanum.hpp`.
- `xrobot-org/BMI088`, `xrobot-org/MadgwickAHRS`: gimbal IMU and attitude estimation; the gimbal attitude Topic subscribed by `Omni` is published by instances of these Modules.
- LibXR.

Hardware: four drive-wheel motors (the power model uses the M3508 current-to-torque ratio); `Helm` adds four steering motors (using the GM6020 ratio) and `Mecanum` may add one track motor; a supercapacitor (PowerControl provides the capacitor energy) and the referee system; the gimbal provides `yawmotor_angle`, and `Omni` also needs the gimbal attitude and `pitchmotor_angle`.
