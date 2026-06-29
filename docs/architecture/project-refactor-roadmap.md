# RoboDrone 全项目重构目标路线图

## 1. 文档定位

本文档记录 RoboDrone 工程未来的长期分层重构目标。

它不是一次性执行清单，而是后续每一阶段重构的方向约束。实际落地时仍然坚持小步提交、小步验证，尤其涉及启动、通信、控制、电机、电调和执行器时，必须保持可回退。

`docs/architecture/refactor-notes.md` 继续作为已经完成的重构记录；本文档作为未来目标蓝图。

## 2. 当前工程状态

当前工程已经完成第一轮启动层重构：

- `main.c` 负责 HAL/CubeMX 外设初始化，并创建 `App_InitTask`。
- `user/app` 已承接业务启动编排。
- `user/domain` 已承接公共领域类型。
- `rtos_init.c/.h` 已移除，旧 RTOS 启动兼容入口不再保留。
- `MDK-ARM/project.uvoptx` 是 Keil 用户状态文件，默认不提交。
- 当前工作区可能存在用户未提交改动，后续重构不得混入无关提交。

当前启动路径：

```text
main.c
  -> App_InitTask()
     -> App_Init()
        -> App_InitSystemModules()
     -> App_StartTasks()
     -> App_LogStartupComplete()
     -> vTaskDelete(NULL)
```

## 3. 目标分层

长期目标是让每一层职责清楚、依赖方向稳定、硬件行为可验证。

### 3.1 app 层

职责：

- 启动编排。
- 系统初始化顺序。
- FreeRTOS 任务创建清单。
- 系统级启动配置。

目标形态：

```text
user/app/app_init.c      只保留高层启动入口和编排
user/app/app_config.h    启动配置和编译期开关
user/app/app_tasks.c     FreeRTOS 任务创建、栈大小、优先级
user/app/app_system.c    系统/设备/业务模块初始化编排
```

### 3.2 domain 层

职责：

- 无人机领域公共类型。
- 状态、目标、控制模式、遥控命令等跨模块数据结构。
- 不依赖具体硬件驱动和 FreeRTOS 任务实现。

目标形态：

```text
user/domain/drone_types.h
user/domain/flight_mode.h
user/domain/remote_command.h
user/domain/vehicle_state.h
```

### 3.3 drivers / module / hardware 层

职责：

- 直接访问芯片、外设、传感器和执行器。
- 提供尽量薄的硬件能力接口。
- 不承载业务流程和飞控策略。

当前工程已有 `module`、`hardware`，后续重点是减少 `abstract` 中硬件和业务混合的情况。

目标边界示例：

```text
module/gyro        陀螺仪芯片和姿态原始数据
module/bmp280      气压计驱动
module/bn220       GPS 驱动和 NMEA 解析
module/RF          nRF24L01 芯片访问
module/motor       电机、电调、舵机、执行器驱动
hardware/usart     串口硬件适配
```

### 3.4 services 层

职责：

- 把底层硬件能力包装成业务能力。
- 管理遥控、地面站、无线链路、定位、传感器融合等业务服务。
- 向 app 和 control 提供稳定接口。

未来可考虑新增：

```text
user/services/remote_service.c
user/services/telemetry_service.c
user/services/radio_link_service.c
user/services/navigation_service.c
user/services/sensor_service.c
```

### 3.5 control 层

职责：

- 飞控任务循环。
- 模式选择。
- 姿态控制。
- 位置/光流控制。
- 解锁、安全保护、限幅和输出混控。

目标形态：

```text
user/control/control_task.c
user/control/control_mode.c
user/control/attitude_control.c
user/control/position_control.c
user/control/control_safety.c
user/control/control_mixer.c
```

控制层风险最高，必须最后拆，且先搬函数、不改算法。

### 3.6 infrastructure / tool / sys 层

职责：

- 日志。
- 全局时间。
- 延时。
- 看门狗守护。
- 调试工具。
- PC 侧分析脚本。

目标：

- `sys` 保持嵌入式运行时基础设施。
- `tool` 保持日志、Trace、USMART、Python 分析工具。
- 清理不应纳入固件工程的临时文件、备份文件、虚拟环境和历史数据。

## 4. 重构阶段顺序

推荐顺序如下。

1. 拆 `app_tasks.c/.h`。
   - 从 `app_init.c` 拆出任务创建。
   - 保持任务名、栈大小、优先级、创建顺序不变。

2. 拆 `app_system.c/.h`。
   - 从 `app_init.c` 拆出系统初始化、设备初始化、业务模块初始化。
   - 保持初始化顺序不变。

3. 继续收敛 `domain` 类型。
   - 把飞行模式、遥控命令、状态数据逐步从控制/通信头文件中分离。
   - 避免通信层为了类型依赖 include 控制层。

4. 整理通信层边界。
   - 区分无线芯片驱动、遥控数据、地面站协议。
   - 遥控链路每一步都要上板验证。

5. 整理传感器和定位边界。
   - 梳理 gyro、bmp280、gps、光流和 position 的职责。
   - 分清原始传感器数据、滤波结果、定位状态。

6. 拆 control 层。
   - 先拆任务循环和模式选择。
   - 再拆姿态控制、位置控制、安全保护和混控输出。
   - 不在拆文件阶段修改控制算法。

7. 整理工具和调试资产。
   - 梳理 Python 工具、日志工具、TraceRecorder、备份文件、venv、历史采样数据。
   - 明确哪些进入工程，哪些只作为本地工具保留。

8. 收敛文档和工程配置。
   - Keil 分组和 include path 与源码目录一致。
   - 学习文档同步当前真实入口和任务表。
   - 架构记录和重构目标保持更新。

## 5. 每阶段禁止事项

除非该阶段明确说明，否则禁止：

- 修改任务周期。
- 修改任务优先级。
- 修改任务栈大小。
- 修改任务创建顺序。
- 修改控制算法。
- 修改遥控协议或地面站协议。
- 修改电机、电调、舵机、执行器初始行为。
- 提交 `MDK-ARM/project.uvoptx`。
- 把用户已有未提交改动混入无关重构提交。

## 6. 验证策略

每一阶段至少验证：

- Keil GUI Build/Rebuild 0 error。
- 启动日志正常。
- 遥控数据持续更新。
- `Control`、`Receive`、`Send`、`WDog` 任务正常运行。
- `Init` 任务完成后删除自身。

涉及动力系统时额外验证：

- 电机初始状态不异常。
- 舵机/电杆初始状态不异常。
- 电调没有异常启动或误输出。
- 看门狗仍能正常喂狗。

## 7. 提交和文档规范

- Git commit 描述使用中文。
- 每个阶段单独提交。
- 重构提交尽量只包含同一类改动。
- 文档更新和代码改动可以同阶段提交，但不能混入无关业务改动。
- `docs/architecture/refactor-notes.md` 记录已完成步骤。
- 本文档记录未来目标和阶段路线。

## 8. 下一步建议

下一步优先执行：

```text
重构：拆分 app 任务创建模块
```

目标：新增 `user/app/app_tasks.c` 和 `user/app/app_tasks.h`，把任务创建清单从 `app_init.c` 移出。

本步骤风险低，适合作为 app 层继续拆分的第一步。
