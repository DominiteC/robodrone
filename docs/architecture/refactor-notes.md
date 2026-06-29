# 架构重构记录

## 重构前保护点

- 原始分支：`second`
- 重构分支：`codex/architecture-refactor`
- 重构前标签：`pre-architecture-20260615-103919`
- 目录备份：`D:\Code\robodrone\backups\project-pre-architecture-20260615-103919`
- 第一阶段提交：`19ae5b7 refactor: introduce domain type boundary`

如果重构失败，优先切回 `pre-architecture-20260615-103919` 对应状态；如果 Git 状态异常，再使用目录备份恢复。

## 注释和文档约定

- 新增或调整的代码注释统一使用中文。
- 注释只解释模块职责、关键约束、危险行为、硬件依赖和非显然逻辑。
- 不写“给变量赋值”这类重复代码表面的注释。
- 架构设计、迁移步骤、验证结果统一写入 Markdown 文档，避免只存在聊天记录里。
- 后续 Git 提交描述统一使用中文，并在本文档记录每一步提交对应的重构意图。

## 提交记录

- `19ae5b7`：引入领域公共类型边界，新增 `drone_types.h`。
- `2ce9388`：恢复运行基线，回退 app 启动层迁移。
- `f74b853`：命名 RTOS 任务资源配置，集中管理任务栈大小和优先级。
- `8d8b33c`：拆分 RTOS 任务创建函数，把 `xTaskCreate()` 调用集中到文件内静态函数。
- 本次提交描述：`重构：拆分 RTOS 全局服务初始化`，拆分 RTOS 全局服务初始化函数。
- 本次提交描述：`重构：拆分 RTOS 设备初始化`，拆分 RTOS 设备初始化函数。
- 本次提交描述：`重构：拆分 RTOS 业务模块初始化`，拆分 RTOS 业务模块初始化函数。
- 本次提交描述：`重构：拆分 RTOS 启动完成日志`，拆分 RTOS 启动完成日志函数。
- 本次提交描述：`重构：新增 app 启动层空壳`，新增 app 层编译边界但暂不接管启动入口。
- 本次提交描述：`重构：增加 app 初始化任务薄封装`，让 `App_InitTask()` 转调 `RTOS_Init()`，但暂不切换入口。
- 本次提交描述：`重构：切换 main 初始化任务入口到 app 层`，让 `main.c` 创建 `App_InitTask`。
- 本次提交描述：`重构：抽出 RTOS 启动流程函数`，新增 `RTOS_RunStartup()` 作为真实启动流程入口。
- 本次提交描述：`重构：由 app 初始化任务运行启动流程`，让 `App_InitTask()` 调用 `RTOS_RunStartup()` 并删除自身。
- 本次提交描述：`重构：让 app 初始化任务调用 App_Init`，形成 `App_InitTask()` 到 `App_Init()` 的 app 层入口关系。
- 本次提交描述：`重构：补齐 app 任务启动编排入口`，让 `App_InitTask()` 调用 `App_StartTasks()`。
- 本次提交描述：`重构：迁移任务启动编排到 app 层`，让 `App_StartTasks()` 负责触发任务创建和启动完成日志。
- 本次提交描述：`重构：迁移初始化编排到 app 层`，让 `App_Init()` 负责触发系统模块初始化。
- 本次提交描述：`重构：搬迁启动实现到 app 层`，把系统初始化、任务创建和启动完成日志实现迁入 `app_init.c`。
- 本次提交描述：`重构：收敛 app 启动接口命名`，让 app 层优先使用 `App_*` 接口，`RTOS_*` 仅作为兼容转发。
- 本次提交描述：`清理：移除 main 中旧 RTOS 初始化注释`，让主入口只保留 app 初始化任务。
- 本次提交描述：`文档：同步学习文档中的 app 启动入口`，把学习文档里的旧 `RTOS_Init` 启动路径更新为 `App_InitTask`。
- 本次提交描述：`重构：迁移启动配置到 app 层`，新增 `app_config.h` 管理地面站串口启动配置。
- 本次提交描述：`清理：移除 RTOS 启动兼容入口`，删除 `rtos_init.c/.h`，启动入口统一收敛到 app 层。

## 第一阶段：领域类型边界

本阶段只做低风险边界整理，不改变运行逻辑。

- 新增 `user/domain/drone_types.h`，集中放置无人机领域公共类型。
- `state_t`、`setpoint_t`、`MotorCtrl`、`DroneParams`、控制模式枚举从控制/通信头文件中抽离。
- `user/abstract/structConfig.h` 保留为兼容入口，内部转发到 `drone_types.h`。
- `user/control/control.h` 只保留控制模块公开 API 和 PID 相关 extern。
- `user/communicate/commander.h` 不再 include `control.h`，通信层不再为了类型依赖控制层。
- `MDK-ARM/project.uvprojx` 增加 `../user/domain` include path，并新增 `domain` 分组。

## 第二阶段：app 启动层回退

曾尝试把 `rtos_init.c` 的初始化和任务创建迁移到 `user/app/app_init.c`，但上板后出现运行卡死、遥控数据不更新的现象。为了恢复运行基线，已回退第二阶段 app 启动层。

- `user/abstract/rtos_init.c` 恢复为完整初始化和任务创建入口。
- `user/abstract/rtos_init.h` 恢复原有宏定义和 `RTOS_Init()` 接口。
- `MDK-ARM/project.uvprojx` 已移除 `../user/app` include path 和 `app` 分组。
- `user/app/app_init.c/.h` 暂时移除，不参与编译。
- 第一阶段 `drone_types.h` 类型边界保留。

## 当前验证重点

- 使用 Keil GUI 编译，要求 0 error。
- 烧录后确认启动日志正常，遥控器数据能持续更新。
- 确认 `Control`、`Receive`、`Send` 任务恢复运行，板子不再卡死。
- 如果恢复正常，后续重新做 app 层时必须更小步迁移：先加空壳，再逐段搬初始化，每段都上板验证。

## 第三阶段：任务资源参数命名

本阶段只整理 `RTOS_Init()` 中 FreeRTOS 任务创建参数，不迁移入口、不调整任务顺序、不改变任务周期。

- 在 `user/abstract/rtos_init.c` 中新增 `TASK_STACK_*` 和 `TASK_PRIO_*` 宏。
- `Alarm`、`usmart`、`ANO_DT`、`Send`、`Receive`、`Control`、`mtf_01`、`WDog` 的任务名、栈大小和优先级保持原值。
- `main.c` 仍然通过 `xTaskCreate(RTOS_Init, "Init", ...)` 启动。
- 当前启动路径仍是 `main.c -> RTOS_Init()`，没有重新启用 `App_InitTask()`。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认遥控器数据仍持续更新。
- 确认 `Receive`、`Send`、`Control` 和 `WDog` 任务仍能运行。

## 第四阶段：文件内拆分任务创建函数

本阶段仍然不恢复 `app` 启动层，只在 `user/abstract/rtos_init.c` 文件内部拆分职责。

- 新增 `static void RTOS_CreateTasks(void)`，集中放置所有 `xTaskCreate()` 调用。
- `RTOS_Init()` 仍然是 `main.c` 创建的初始化任务入口。
- 初始化顺序、任务创建顺序、任务名、栈大小、优先级保持不变。
- 这样先把“初始化流程”和“任务创建清单”分开，后续如果再尝试 app 层迁移，可以按这个函数边界逐步搬迁。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动日志、遥控数据、控制任务和看门狗任务表现不变。

## 第五阶段：文件内拆分全局服务初始化

本阶段继续只在 `user/abstract/rtos_init.c` 文件内部拆分职责，不改变启动入口。

- 新增 `static void RTOS_InitGlobalServices(void)`。
- 集中放置日志初始化、地面站串口开关策略、延时初始化、全局时间定时器启动和 USART1 初始化/静默逻辑。
- `WatchdogGuard_WasIwdgReset()` 的复位原因检查仍保留在 `RTOS_Init()` 最前面。
- `RTOS_Init()` 仍然由 `main.c` 创建，仍然在完成初始化和任务创建后 `vTaskDelete(NULL)` 删除自身。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动日志正常，遥控数据持续更新。
- 尤其确认关闭地面站串口时，无线链路表现不变。

## 第六阶段：文件内拆分设备初始化

本阶段继续只在 `user/abstract/rtos_init.c` 文件内部拆分职责，不改变启动入口。

- 新增 `static void RTOS_InitDevices(void)`。
- 集中放置报警、电调、GPS、陀螺仪、气压计和无线通信初始化。
- `position_init()`、`Control_Init()`、`Motor_Init()`、`Actuator_Init()` 仍留在 `RTOS_Init()` 中，暂不拆业务模块初始化。
- 初始化顺序保持为全局服务初始化之后、位置和控制初始化之前。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认遥控数据仍持续更新。
- 确认报警、电调初始状态、传感器启动日志和无线接收表现不变。

## 第七阶段：文件内拆分业务模块初始化

本阶段继续只在 `user/abstract/rtos_init.c` 文件内部拆分职责，不改变启动入口。

- 新增 `static void RTOS_InitBusinessModules(void)`。
- 集中放置位置、控制、电机、执行器、看门狗长动作退出和 `usmart_init()` 初始化。
- `vTaskDelay(500)` 保持在 `Motor_Init()` 和 `Actuator_Init()` 之间，避免改变动力系统初始化节奏。
- `RTOS_Init()` 现在只保留复位检查、全局服务初始化、设备初始化、业务模块初始化、任务创建、日志输出和删除自身。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认遥控数据仍持续更新。
- 确认控制任务、电机/执行器初始状态和看门狗任务表现不变。

## 第八阶段：文件内拆分启动完成日志

本阶段继续只在 `user/abstract/rtos_init.c` 文件内部拆分职责，不改变启动入口。

- 新增 `static void RTOS_LogStartupComplete(void)`。
- 集中放置启动完成后的剩余堆栈日志和“你好世界”日志。
- `RTOS_Init()` 的主流程进一步收敛为初始化编排、任务创建、启动完成日志和删除自身。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动完成日志仍能打印。
- 确认遥控数据、控制任务和看门狗任务表现不变。

## 第九阶段：新增 app 启动层空壳

本阶段只建立 app 层编译边界，不迁移任何启动逻辑。

- 新增 `user/app/app_init.h` 和 `user/app/app_init.c`。
- 暂时提供 `App_Init()`、`App_StartTasks()`、`App_InitTask(void *param)` 空实现。
- `MDK-ARM/project.uvprojx` 增加 `../user/app` include path 和 `app` 分组。
- `main.c` 仍然创建 `RTOS_Init` 任务，运行路径没有变成 `App_InitTask()`。
- `RTOS_Init()` 仍然保留在 `user/abstract/rtos_init.c` 中并负责真实初始化和任务创建。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动路径和前一阶段一致。
- 确认遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。

## 第十阶段：增加 app 初始化任务薄封装

本阶段只让 app 层具备兼容入口能力，仍然不改变实际启动路径。

- `user/app/app_init.c` include `rtos_init.h`。
- `App_InitTask(void *param)` 内部直接调用 `RTOS_Init(param)`。
- `main.c` 暂时不改，仍然通过 `xTaskCreate(RTOS_Init, "Init", ...)` 启动。
- 这一步用于先验证 app 层能够引用 abstract 层入口，下一步再单独切换 `main.c` 的任务入口。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认行为与上一阶段一致，因为实际入口仍未切换。
- 确认遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。

## 第十一阶段：切换 main 初始化任务入口到 app 层

本阶段第一次让 `main.c` 通过 app 层进入初始化任务，但 app 层仍然只是薄封装。

- `Core/Src/main.c` include 从 `rtos_init.h` 切换为 `app_init.h`。
- 初始化任务从 `xTaskCreate(RTOS_Init, "Init", 250, NULL, 20, NULL)` 切换为 `xTaskCreate(App_InitTask, "Init", 250, NULL, 20, NULL)`。
- `Init` 任务名、栈大小、优先级全部保持不变。
- `App_InitTask()` 当前仍然只调用 `RTOS_Init(param)`，真实初始化逻辑仍留在 `user/abstract/rtos_init.c`。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动日志、遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。
- 如果出现启动卡死，优先回退本阶段提交，因为这是首次改变实际入口路径。

## 第十二阶段：抽出 RTOS 启动流程函数

本阶段把真实启动流程从 `RTOS_Init()` 中抽成可复用函数，但暂不改变 app 层调用方式。

- `user/abstract/rtos_init.h` 新增 `RTOS_RunStartup(void)` 声明。
- `RTOS_RunStartup()` 包含复位原因检查、全局服务初始化、设备初始化、业务模块初始化、任务创建和启动完成日志。
- `RTOS_Init(void *param)` 保留为兼容入口，内部调用 `RTOS_RunStartup()` 后继续 `vTaskDelete(NULL)` 删除自身。
- `App_InitTask()` 当前仍然调用 `RTOS_Init(param)`，所以实际任务生命周期保持不变。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动日志、遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。
- 如果本阶段稳定，下一步再让 `App_InitTask()` 直接调用 `RTOS_RunStartup()` 并在 app 层删除自身。

## 第十三阶段：由 app 初始化任务运行启动流程

本阶段让 app 层真正承担初始化任务入口职责，但真实启动流程仍复用 `RTOS_RunStartup()`。

- `App_InitTask(void *param)` 忽略参数后调用 `RTOS_RunStartup()`。
- `App_InitTask()` 在启动流程完成后调用 `vTaskDelete(NULL)` 删除自身。
- `RTOS_Init()` 保留为兼容入口，内部仍然调用 `RTOS_RunStartup()` 后删除自身。
- `main.c` 仍然创建 `App_InitTask`，`Init` 任务名、栈大小和优先级保持不变。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认 `Init` 任务执行完成后删除自身。
- 确认启动日志、遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。

## 第十四阶段：让 app 初始化任务调用 App_Init

本阶段继续整理 app 层入口关系，不改变真实启动行为。

- `App_Init()` 当前调用 `RTOS_RunStartup()`。
- `App_InitTask(void *param)` 忽略参数后调用 `App_Init()`，再 `vTaskDelete(NULL)` 删除自身。
- 启动路径变为 `main.c -> App_InitTask() -> App_Init() -> RTOS_RunStartup()`。
- `RTOS_Init()` 仍保留兼容入口，便于必要时回退。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动日志、遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。

## 第十五阶段：补齐 app 任务启动编排入口

本阶段只补齐 app 层的任务启动入口形状，不迁移任何任务创建逻辑。

- `App_InitTask()` 现在依次调用 `App_Init()`、`App_StartTasks()`，最后 `vTaskDelete(NULL)` 删除自身。
- `App_StartTasks()` 当前仍为空函数，真实任务创建仍在 `RTOS_RunStartup()` 内部通过 `RTOS_CreateTasks()` 完成。
- 这一步为后续迁移任务创建提供明确落点，但当前运行行为应保持不变。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动日志、遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。

## 第十六阶段：迁移任务启动编排到 app 层

本阶段把任务启动编排迁移到 app 层，但底层任务创建实现仍暂放在 `rtos_init.c`。

- `RTOS_RunStartup()` 只保留复位检查、全局服务初始化、设备初始化和业务模块初始化。
- 新增公开函数 `RTOS_StartTasks()`，封装原来的任务创建清单。
- `RTOS_LogStartupComplete()` 从文件内静态函数调整为公开函数，保持启动完成日志仍在任务创建之后输出。
- `App_StartTasks()` 现在调用 `RTOS_StartTasks()` 和 `RTOS_LogStartupComplete()`。
- `RTOS_Init()` 兼容入口仍按原顺序调用 `RTOS_RunStartup()`、`RTOS_StartTasks()`、`RTOS_LogStartupComplete()`，最后删除自身。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动完成日志仍在任务创建后出现。
- 确认遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。

## 第十七阶段：迁移初始化编排到 app 层

本阶段让 app 层承担初始化编排调用，但底层初始化实现仍暂放在 `rtos_init.c`。

- 新增公开函数 `RTOS_InitSystemModules()`，封装复位原因检查、全局服务初始化、设备初始化和业务模块初始化。
- `App_Init()` 现在调用 `RTOS_InitSystemModules()`。
- `RTOS_RunStartup()` 保留为兼容函数，内部转调 `RTOS_InitSystemModules()`。
- `RTOS_Init()` 兼容入口仍按 `RTOS_RunStartup()`、`RTOS_StartTasks()`、`RTOS_LogStartupComplete()`、`vTaskDelete(NULL)` 执行。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动日志、遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。
- 如果本阶段稳定，后续可以逐步把底层初始化函数从 `rtos_init.c` 移到 `app_init.c` 或更合适的 app 子模块。

## 第十八阶段：搬迁启动实现到 app 层

本阶段跨度更大：把启动实现主体从 `user/abstract/rtos_init.c` 搬到 `user/app/app_init.c`。

- `rtos_init.c` 保留 `RTOS_Init(void *param)` 兼容入口，只按原顺序调用 `RTOS_RunStartup()`、`RTOS_StartTasks()`、`RTOS_LogStartupComplete()`，最后删除自身。
- `app_init.c` 承接全局服务初始化、设备初始化、业务模块初始化、任务创建、启动完成日志和 `usmart_task()`。
- 任务名、任务栈大小、任务优先级和任务创建顺序保持不变。
- `App_InitTask()` 的实际路径仍是 `App_Init()`、`App_StartTasks()`、`vTaskDelete(NULL)`。
- `RTOS_RunStartup()`、`RTOS_InitSystemModules()`、`RTOS_StartTasks()`、`RTOS_LogStartupComplete()` 仍保留为兼容 API，但实现位置迁入 app 层。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后重点确认启动日志、遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。
- 如果本阶段异常，优先回退本阶段提交，因为这是首次把启动实现主体搬入 app 层。

## 第十九阶段：收敛 app 启动接口命名

本阶段整理 app 层命名，减少 app 实现里直接使用 `RTOS_*` 编排名。

- `App_Init()` 调用 `App_InitSystemModules()`。
- `App_StartTasks()` 直接负责创建 FreeRTOS 任务。
- `App_InitTask()` 调用 `App_Init()`、`App_StartTasks()`、`App_LogStartupComplete()`，最后删除自身。
- `RTOS_InitSystemModules()`、`RTOS_StartTasks()`、`RTOS_LogStartupComplete()` 保留为兼容转发函数。
- `rtos_init.h` 中补充中文注释，说明新代码优先使用 `app_init.h` 的 `App_*` 接口。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动日志、遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。
- 如果稳定，后续可以逐步减少外部对 `rtos_init.h` 的依赖。

## 第二十阶段：清理 main 中旧 RTOS 初始化注释

本阶段只清理入口文件里的旧路径痕迹，不改变运行行为。

- `Core/Src/main.c` 保留 `xTaskCreate(App_InitTask, "Init", 250, NULL, 20, NULL)`。
- 移除旧的 `RTOS_Init(NULL)` 注释。
- `main.c` 继续只通过 `app_init.h` 进入 app 启动层。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动日志、遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。

## 第二十一阶段：同步学习文档中的 app 启动入口

本阶段只更新学习文档，不改变代码。

- `docs/learning/00-reading-guide.md` 中的初始化任务入口更新为 `App_InitTask`。
- `docs/learning/01-boot-and-rtos.md` 中的启动链路更新为 `main.c -> App_InitTask -> app_init.c`。
- `docs/learning/02-task-map.md` 中的任务创建位置从 `rtos_init.c` 更新为 `app_init.c`。
- 同步当前 `ENABLE_GCS_SERIAL = 1` 的说明，避免文档仍描述旧的串口任务开关状态。

验证要求：

- 本阶段只改 Markdown，无需上板验证。
- 后续读代码时，以 `app_init.h` 的 `App_*` 接口作为新启动层入口。

## 第二十二阶段：迁移启动配置到 app 层

本阶段把启动编排相关配置从兼容头文件迁移到 app 层。

- 新增 `user/app/app_config.h`。
- `USE_USMART_OR_ANO` 和 `ENABLE_GCS_SERIAL` 从 `rtos_init.h` 移入 `app_config.h`，宏值保持不变。
- `app_init.c` include `app_config.h` 后继续使用这些宏控制地面站串口任务。
- `rtos_init.h` 进一步收敛为兼容入口声明，不再持有 app 启动配置。
- `MDK-ARM/project.uvprojx` 的 app 分组加入 `app_config.h`。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认地面站串口任务选择、遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。

## 第二十三阶段：移除 RTOS 启动兼容入口

本阶段正式删除旧的 RTOS 启动兼容层，启动入口统一收敛到 app 层。

- 删除 `user/abstract/rtos_init.c` 和 `user/abstract/rtos_init.h`。
- `user/app/app_init.c` 不再 include `rtos_init.h`。
- 移除 `RTOS_RunStartup()`、`RTOS_InitSystemModules()`、`RTOS_StartTasks()`、`RTOS_LogStartupComplete()` 兼容转发函数。
- 从 `MDK-ARM/project.uvprojx` 的 `abstract` 分组移除 `rtos_init.c`。
- `main.c` 继续通过 `xTaskCreate(App_InitTask, "Init", 250, NULL, 20, NULL)` 启动。
- 本阶段同时纳入用户对 `ANO_DT.c/.h` 中高频振动任务声明和实现的清理。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动日志、遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。
- 如果出现链接错误，优先检查是否还有源码 include `rtos_init.h` 或引用 `RTOS_*` 启动函数。

## 第二十四阶段：拆分 app 任务创建模块

本阶段只拆分 app 层内部职责，不改变任务创建行为。

- 新增 `user/app/app_tasks.h` 和 `user/app/app_tasks.c`。
- `App_StartTasks()` 改为调用 `App_CreateTasks()`。
- 任务栈大小、优先级、任务名和创建顺序保持不变。
- `usmart_task()` 从 `app_init.c` 移入 `app_tasks.c`，仍保持文件内静态函数。
- `MDK-ARM/project.uvprojx` 的 app 分组加入 `app_tasks.c/.h`。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认 `Alarm`、`Send`、`Receive`、`Control`、`mtf_01`、`WDog` 任务仍正常运行。
- 确认 `ENABLE_GCS_SERIAL` 和 `USE_USMART_OR_ANO` 的任务选择行为不变。

## 第二十五阶段：拆分 app 系统初始化模块

本阶段只拆分 app 层内部职责，不改变系统初始化行为。

- 新增 `user/app/app_system.h` 和 `user/app/app_system.c`。
- `App_InitSystemModules()` 从 `app_init.c` 迁移到 `app_system.c`。
- 全局服务初始化、设备初始化、业务模块初始化仍按原顺序执行。
- `usartDebug` 和 `USART_buf` 随串口初始化逻辑迁移到 `app_system.c`，符号名保持不变，兼容 `usmart_port.c` 的外部引用。
- `app_init.c` 收敛为启动入口编排、任务启动调用和启动完成日志。
- `MDK-ARM/project.uvprojx` 的 app 分组加入 `app_system.c/.h`。
- 提交描述：`重构：拆分 app 系统初始化模块`。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动日志、遥控数据、控制任务、发送任务、接收任务和看门狗任务表现不变。
- 确认电机、电调和电杆初始状态不异常。

## 第二十六阶段：拆分 app 启动诊断模块

本阶段继续收敛 app 层职责，只拆分启动完成日志，不改变启动行为。

- 新增 `user/app/app_diagnostics.h` 和 `user/app/app_diagnostics.c`。
- `App_LogStartupComplete()` 从 `app_init.c` 迁移到 `app_diagnostics.c`。
- `app_init.c` 只保留启动编排、系统初始化调用、任务创建调用和删除自身。
- `MDK-ARM/project.uvprojx` 的 app 分组加入 `app_diagnostics.c/.h`。
- 提交描述：`重构：拆分 app 启动诊断模块`。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动完成日志仍输出 `Free heap` 和 `你好世界`。
- 确认启动、遥控、控制、发送、接收和看门狗任务表现不变。

## 第二十七阶段：拆分领域类型头文件

本阶段只拆分领域类型声明，不改变已有模块 include 关系和运行行为。

- 新增 `vector_types.h`、`flight_mode.h`、`vehicle_state.h`、`actuator_types.h` 和 `drone_params.h`。
- `drone_types.h` 收敛为兼容聚合头，继续向现有模块提供原来的全部类型。
- 现有 `control.h`、`commander.h`、`remotedata.h` 等文件仍可继续 include `drone_types.h`。
- `MDK-ARM/project.uvprojx` 的 domain 分组加入新的领域类型头文件。
- 提交描述：`重构：拆分领域类型头文件`。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认控制、遥控、通信和姿态/位置数据表现不变。
- 后续再逐个模块替换为更精确的领域头文件 include。
## 第二十八阶段：收敛领域类型 include 依赖

本阶段开始使用拆分后的领域类型头文件，只调整头文件依赖，不改变数据结构和运行逻辑。

- `gyro.h` 和 `position.h` 改为直接 include `vector_types.h`。
- `remotedata.h` 改为直接 include `vector_types.h`。
- `commander.h` 改为直接 include `flight_mode.h` 和 `vehicle_state.h`。
- 保留 `drone_types.h` 作为旧模块兼容聚合头。
- 提交描述：`重构：收敛领域类型 include 依赖`。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认遥控数据、控制目标、姿态/位置数据表现不变。
## 第二十九阶段：移除干净源文件中的 structConfig 间接类型依赖

本阶段继续收敛领域类型 include 依赖，只调整干净源文件的头文件引用。

- `gyro.c` 改为直接 include `vector_types.h`。
- `remotedata.c` 改为直接 include `vector_types.h`。
- 不修改函数实现、控制逻辑、通信协议和任务行为。
- 提交描述：`重构：移除源文件中的 structConfig 间接依赖`。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认陀螺仪数据、遥控接收和发送任务表现不变。

## 第三十阶段：移除 structConfig 兼容头文件

本阶段删除已经没有源码引用的历史兼容头文件。

- 删除 `user/abstract/structConfig.h`。
- 从 `MDK-ARM/project.uvprojx` 的 abstract 分组移除 `structConfig.h`。
- 领域类型依赖统一通过 `user/domain` 下的精确头文件或兼容聚合头 `drone_types.h` 获取。
- 提交描述：`重构：移除 structConfig 兼容头文件`。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 如果出现找不到 `structConfig.h`，说明仍有漏掉的旧 include，需要改为对应的 domain 类型头文件。

## 第三十一阶段：迁移遥控业务到 services 层

本阶段开始建立 services 层，将遥控业务从旧 communicate 目录迁出。

- 新增 `user/services` 目录。
- `commander.c/.h` 从 `user/communicate` 迁移到 `user/services`，作为遥控命令服务。
- `remotedata.c/.h` 从 `user/communicate` 迁移到 `user/services`，作为遥控数据服务。
- Keil include path 从 `user/communicate` 调整为 `user/services`。
- Keil 工程分组从 `communicate` 调整为 `services`，并补充对应头文件条目。
- 本阶段只迁移文件位置和职责归属，不修改遥控协议、控制目标生成逻辑和任务行为。
- 提交描述：`重构：迁移遥控业务到 services 层`。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认遥控接收、模式切换、控制目标生成和 Send 任务表现不变。

## 第三十二阶段：迁移无线链路服务到 services 层

本阶段继续建立 services 层，将无线链路封装从旧 abstract 目录迁出。

- `wireless.c/.h` 从 `user/abstract` 迁移到 `user/services`，作为无线链路服务。
- `wireless` 继续调用 `module/RF/nRF24L01P` 驱动，不改变底层芯片访问逻辑。
- Keil 工程 abstract 分组移除 `wireless.c`，services 分组加入 `wireless.c/.h`。
- 本阶段不修改无线协议、收发任务、链路恢复策略和遥控数据处理逻辑。
- 提交描述：`重构：迁移无线链路服务到 services 层`。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认遥控接收、遥控回传、Receive 任务和 Send 任务表现不变。

## 第三十三阶段：迁移告警服务到 services 层

本阶段继续建立 services 层，将电池和告警状态服务从旧 abstract 目录迁出。

- `alarm.c/.h` 从 `user/abstract` 迁移到 `user/services`。
- `alarm` 继续负责电池电压/电流检测、蜂鸣器和 LED 告警状态更新。
- Keil 工程 abstract 分组移除 `alarm.c`，services 分组加入 `alarm.c/.h`。
- 本阶段不修改告警阈值、告警状态机、任务周期和硬件输出行为。
- 提交描述：`重构：迁移告警服务到 services 层`。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认电池检测、蜂鸣器、LED 告警和 Alarm 任务表现不变。

## 第三十四阶段：拆分控制状态刷新模块

本阶段开始深入拆分控制层，但只迁移状态采集相关函数，不修改任何控制算法。

- 新增 `user/control/control_state.c/.h`。
- `refreshState()` 从 `control.c` 迁移到 `control_state.c`，状态刷新顺序保持不变。
- `printState()` 从 `control.c` 迁移到 `control_state.c`，日志内容保持不变。
- `control.c` include `control_state.h` 后继续在 `Control_Task()` 中调用 `refreshState(&state)`。
- Keil 工程 control 分组加入 `control_state.c/.h`。
- 本阶段不修改任务周期、PID 参数、控制模式判断、输出混控、安全保护和硬件动作。
- 提交描述：`重构：拆分控制状态刷新模块`。

验证要求：

- Keil GUI Build/Rebuild 需要 0 error。
- 烧录后确认启动日志正常，遥控数据持续更新。
- 确认 `Control`、`Receive`、`Send`、`WDog` 任务正常运行。
- 确认电机、电调、舵机、电杆初始状态无异常。
