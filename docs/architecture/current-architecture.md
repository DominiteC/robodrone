# RoboDrone 当前架构说明

## 文档用途

本文档记录当前工程已经落地的分层架构、关键文件职责和后续变更追加位置。

后续每次做架构相关重构时，都需要在本文档的“架构变更记录”中追加一条记录，说明本次改了什么、影响哪些文件、是否改变运行行为。详细执行过程仍记录在 `docs/architecture/refactor-notes.md`，长期目标仍参考 `docs/architecture/project-refactor-roadmap.md`。

## 当前启动链路

```text
main.c
  -> App_InitTask()
     -> App_Init()
        -> App_InitSystemModules()
     -> App_StartTasks()
        -> App_CreateTasks()
     -> App_LogStartupComplete()
     -> vTaskDelete(NULL)
```

当前启动入口已经统一到 app 层。旧的 `rtos_init.c/.h` 已移除。

## 当前分层

### app 层：启动编排

`user/app` 负责系统启动阶段的高层编排，不承载控制算法。

| 文件 | 作用 |
| --- | --- |
| `app_init.c/.h` | 当前启动入口和启动流程编排。负责按顺序调用系统初始化、任务创建、启动诊断，然后删除 `Init` 任务。 |
| `app_system.c/.h` | 系统初始化编排。负责日志、延时、全局时间、串口策略、设备初始化、业务模块初始化等顺序。 |
| `app_tasks.c/.h` | FreeRTOS 业务任务创建清单。集中维护任务名、栈大小、优先级和创建顺序。 |
| `app_diagnostics.c/.h` | 启动完成后的基础诊断日志，例如剩余堆和启动完成提示。 |
| `app_config.h` | app 层启动相关编译期开关，例如地面站串口任务选择。 |

### domain 层：领域类型

`user/domain` 负责无人机领域公共类型，不依赖硬件驱动和 FreeRTOS 任务实现。

| 文件 | 作用 |
| --- | --- |
| `drone_types.h` | 兼容聚合头。旧模块仍可 include 它获得全部领域类型。新代码优先 include 更精确的头文件。 |
| `vector_types.h` | 速度、角度、加速度、角速度、二维位置等基础领域数据类型。 |
| `flight_mode.h` | 位置控制模式、控制模式、姿态模式等枚举。 |
| `vehicle_state.h` | 飞行器当前状态 `state_t` 和控制目标 `setpoint_t`。 |
| `actuator_types.h` | 电机、电调和执行器输出相关类型，例如 `MotorCtrl`。 |
| `drone_params.c/.h` | 机体物理参数和动力学参数，例如 `DroneParams` 和当前机体参数 `drone`。 |

### control 层：飞控逻辑

`user/control` 承载飞控核心逻辑。当前已经开始从大文件 `control.c` 中拆出边界清晰的子模块，但控制算法、任务周期和输出行为仍保持原样。

| 文件 | 作用 |
| --- | --- |
| `control.c/.h` | 控制任务入口、模式判断、姿态/高度/位置控制、输出混控和安全保护。后续继续按职责拆分。 |
| `control_pid.c/.h` | PID 参数、PID 实例和控制初始化入口。负责集中维护 PID 配置。 |
| `control_rates.h` | 控制任务频率、主循环周期和分频执行宏。 |
| `control_state.c/.h` | 控制状态刷新和状态打印。负责从陀螺仪、光流/定位模块读取 `state_t` 所需数据。 |
| `control_output.c/.h` | 控制输出落地。负责把 `MotorCtrl` 写入电调和电机驱动。 |
| `control_safety.c/.h` | 运行时安全检查。负责异常状态锁定、姿态超限保护和安全输出切断。 |
| `PIDcontroller.c/.h` | PID 控制器实现和参数结构。 |
| `change.c/.h` | 形态/姿态切换相关控制逻辑。 |

### services / communicate / abstract / module 层

当前通信、传感器、定位和硬件适配仍有历史混合情况：

- `user/services`：遥控数据服务、遥控命令服务等业务服务。`commander` 负责控制目标生成，`remotedata` 负责遥控数据帧解析和状态回传，`wireless` 负责无线链路收发和恢复，`alarm` 负责电池和告警状态服务。
- `user/communicate`：后续保留给地面站协议、通信协议适配等更偏协议层的代码。
- `user/abstract`：部分传感器、定位、无线、调试协议等抽象模块。
- `user/module` / `hardware`：芯片驱动、外设适配和底层硬件访问。

后续会逐步把业务服务、硬件驱动和控制逻辑边界拆清楚。

## 当前约束

- 不提交 `MDK-ARM/project.uvoptx`。
- 不把用户已有未提交改动混入无关重构提交。
- 不改任务周期、任务优先级、任务栈大小和任务创建顺序，除非阶段目标明确说明。
- 不在非控制专项阶段修改控制算法、电机/电调/舵机/电杆初始行为。
- 修改源码前必须检查编码、BOM 和换行，写回时保持文件原编码和换行风格。
- 新增源码文件必须写清文件职责注释；涉及中文注释时要特别确认 Keil 显示不会乱码。

## 架构变更记录

### 2026-06-29：建立 app 启动分层

- 新增 `user/app`，将启动编排迁移到 app 层。
- `main.c` 通过 `App_InitTask` 启动初始化任务。
- 后续回退并重新小步推进，最终移除旧 `rtos_init.c/.h`。

### 2026-06-29：拆分 app 任务、系统和诊断模块

- `app_tasks.c/.h` 承接 FreeRTOS 任务创建。
- `app_system.c/.h` 承接系统、设备和业务模块初始化。
- `app_diagnostics.c/.h` 承接启动完成日志。
- `app_init.c` 收敛为启动流程编排入口。

### 2026-06-29：拆分 domain 领域类型头文件

- 从 `drone_types.h` 拆出向量类型、飞行模式、飞行器状态、执行器类型和机体参数。
- `drone_types.h` 保留为兼容聚合头，避免一次性改动所有 include。
- Keil 工程 `domain` 分组加入新的领域类型头文件。

### 2026-06-29：新增当前架构说明文档和文件职责注释

- 新增本文档，用于记录当前架构、文件职责和后续架构变更。
- 为已新增的 app/domain 文件补充文件级职责注释。
- 本阶段不改变运行逻辑。

### 2026-06-29：收敛领域类型 include 依赖

- `gyro.h`、`position.h` 和 `remotedata.h` 开始直接依赖 `vector_types.h`。
- `commander.h` 开始直接依赖 `flight_mode.h` 和 `vehicle_state.h`。
- 本阶段只调整头文件依赖，不改变运行逻辑。

### 2026-06-29：移除干净源文件中的 structConfig 间接类型依赖

- `gyro.c` 和 `remotedata.c` 开始直接依赖 `vector_types.h`。
- `structConfig.h` 暂时保留，等待剩余引用清理完成后再移除。
- 本阶段只调整 include，不改变运行逻辑。

### 2026-06-29：移除 structConfig 兼容头文件

- 删除已经没有源码引用的 `user/abstract/structConfig.h`。
- Keil 工程 abstract 分组同步移除该文件。
- 后续领域类型依赖应直接使用 `user/domain` 下的类型头文件。

### 2026-06-29：迁移遥控业务到 services 层

- `commander.c/.h` 和 `remotedata.c/.h` 从 `user/communicate` 迁移到 `user/services`。
- Keil 工程 include path 和分组同步调整为 `services`。
- 本阶段不改变遥控协议、控制目标生成逻辑和任务行为。

### 2026-06-29：迁移无线链路服务到 services 层

- `wireless.c/.h` 从 `user/abstract` 迁移到 `user/services`。
- `wireless` 作为无线链路服务，底层芯片访问仍由 `module/RF/nRF24L01P` 驱动负责。
- 本阶段不改变无线协议、接收任务和链路恢复行为。

### 2026-06-29：迁移告警服务到 services 层

- `alarm.c/.h` 从 `user/abstract` 迁移到 `user/services`。
- `alarm` 作为电池检测和告警状态服务，底层 GPIO/ADC 访问仍由 HAL 和硬件配置提供。
- 本阶段不改变告警阈值、告警状态机、任务周期和硬件输出行为。

### 2026-06-29：拆分控制状态刷新模块

- 新增 `control_state.c/.h`，承接 `refreshState()` 和 `printState()`。
- `control.c` 通过 `control_state.h` 调用状态刷新，不再直接承载状态采集函数实现。
- 本阶段不改变状态刷新顺序、控制算法、任务周期、PID 参数、输出混控和安全保护逻辑。

### 2026-06-29：拆分控制输出和安全保护模块

- 新增 `control_output.c/.h`，承接 `MotorControl()`，隔离电调和电机驱动调用。
- 新增 `control_safety.c/.h`，承接 `safeCheck()`，隔离运行时安全保护逻辑。
- `control.c` 继续负责任务主循环和控制算法调用，但不再直接 include `esc.h`、`motor.h`、`alarm.h`。
- 本阶段不改变安全阈值、输出数值、控制算法、任务周期和硬件动作。

### 2026-06-29：拆分控制 PID 配置模块

- 新增 `control_pid.c/.h`，承接 PID 参数、PID 实例和 `Control_Init()`。
- `control.h` 通过 `control_pid.h` 暴露已有 PID 实例，兼容 `ANO_DT` 等调参/回传代码。
- `control.c` 不再直接承载大段 PID 参数配置，但继续执行控制算法、PID 计算和复位逻辑。
- 本阶段不改变任何 PID 参数数值、初始化顺序、控制算法和任务行为。

### 2026-06-29：迁移机体参数定义到 domain 层

- 新增 `drone_params.c`，承接当前机体物理参数实例 `drone`。
- `drone_params.h` 继续定义 `DroneParams` 类型，并声明 `extern const DroneParams drone`。
- `control.c` 不再直接定义机体参数，控制主文件继续只关注控制任务和控制算法。
- 本阶段不改变任何机体参数数值、控制算法和任务行为。

### 2026-06-29：拆分控制周期配置头文件

- 新增 `control_rates.h`，承接控制任务频率、主循环周期和 `RATE_DO_EXECUTE()`。
- `control.h` 改为 include `control_rates.h`，继续对外提供原有控制周期宏。
- 本阶段不修改任何周期数值、任务周期、控制算法和硬件动作。
