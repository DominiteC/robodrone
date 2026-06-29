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
| `drone_params.h` | 机体物理参数和动力学参数，例如 `DroneParams`。 |

### control 层：飞控逻辑

`user/control` 当前仍承载控制任务、控制模式判断、姿态/位置控制、安全保护和输出混控。该层风险最高，后续拆分时必须先搬函数、不改算法，并且每一步都要 Build 和上板验证。

### communicate / abstract / module 层

当前通信、传感器、定位和硬件适配仍有历史混合情况：

- `user/communicate`：遥控命令、地面站和通信数据处理。
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
