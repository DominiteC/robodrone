# RoboFly 无人机控制器 Agent 指南

## 构建命令

- **主要构建系统**：Keil MDK‑ARM（AC6 工具链）搭配 VSCode 中的 EIDE 扩展。
- **使用 VSCode 任务**（`终端 > 运行任务`）：
  - `build` – 增量构建
  - `rebuild` – 清理并重新构建
  - `clean` – 删除输出文件
  - `flash` – 通过 OpenOCD（CMSIS‑DAP）烧录 STM32
- **命令行替代方案**：若已安装 `eide` CLI，可在 `MDK‑ARM` 目录下运行 `eide build`。
- **生成输出**：ELF、HEX 和 MAP 文件输出到 `MDK‑ARM/build/project/`。
- **工具链位置**：`d:\soft\Keil_v5\ARM\ARMCLANG`（ARM Compiler 6）。

## 代码组织

- **CubeMX 生成代码**：`Core/`、`Drivers/`、`Middlewares/`（FreeRTOS、DSP 库）。配置存储在 `project.ioc` 中；硬件更改应通过该文件进行，并通过 STM32CubeMX 重新生成代码。  
  **切勿在 `USER CODE BEGIN`/`END` 块之外编辑文件**——它们会在重新生成时被覆盖。
- **应用程序代码**：所有自定义逻辑位于 `user/` 目录下：
  - `user/tool/` – 日志记录（lwrb、printf）、USMART 串口调试交互组件。
  - `user/sys/` – 系统工具（全局时间、延时、中断回调）。
  - `user/hardware/` – 外设驱动包装（UART）。
  - `user/module/` – 传感器/执行器驱动（BMP280、JY901P、BN220、MTF‑01、nRF24L01P 等）。
  - `user/abstract/` – 抽象层（陀螺仪、无线通信、报警、定位、ANO_DT 地面站协议）。
  - `user/control/` – PID 控制器、状态切换逻辑、主控制循环。
  - `user/communicate/` – 遥控数据解析和指令器。
- **FreeRTOS 配置**：`Core/Inc/FreeRTOSConfig.h`。关键设置：
  - `configTICK_RATE_HZ = 1000`
  - `configTOTAL_HEAP_SIZE = 15360` 字节
  - `configMAX_PRIORITIES = 56`
- **入口点**：`Core/Src/main.c` 创建 FreeRTOS 任务。

## 约定

- **包含路径**：所有 `user/` 子目录已包含在编译器的包含列表中（参见 `MDK‑ARM/build/project/builder.params`）。
- **宏定义**：`USE_HAL_DRIVER`、`STM32F407xx`、`ARM_MATH_CM4`（启用 CMSIS‑DSP）。
- **硬件特定常量**：引脚映射、定时器分配和外设配置由 CubeMX 设置，应通过 `.ioc` 文件更改，而非直接修改代码。
- **新建模块**：将源文件添加到相应的 `user/` 子文件夹，并视需要更新 EIDE 项目文件（`MDK‑ARM/.eide/eide.yml`）。

## 变更流程

- **先写设计再改代码**：以后新增功能、修复问题、重构模块或调整控制逻辑前，必须先在工程文档中写清楚要解决的问题、准备怎么做、为什么这样设计、会影响哪些模块。
- **设计内容要可执行**：设计说明应包含数据流、任务关系、接口变化、关键参数、风险点、失败保护和硬件验证方法；确认后再按文档实施代码。
- **实现后回写文档**：代码完成后，把实际改动、验证结果和与原设计不一致的地方补回对应文档，避免设计只停留在聊天记录里。

## 编码处理

- **编辑前必须检查编码**：读取或修改任何已有文件前，先确认文件编码、BOM 和换行风格；写回时保持原编码和换行风格，避免因工具默认编码导致中文注释或字符串乱码。
- **避免隐式转码**：不要使用会默认改写编码的命令批量重写源文件；如必须转换编码，应先说明原因并只处理明确需要转换的文件。

## 配置与环境

- **编译器**：ARM Compiler 6 (AC6) 搭配 C99，最低优化（`-O1`），类似 AC5 的警告级别。
- **链接器**：抑制 L6329 警告；输出 ELF 格式；分散加载文件自动生成。
- **调试**：OpenOCD 搭配 CMSIS‑DAP 接口（目标 `stm32f4x`）。
- **未配置 linting、格式化或单元测试套件**。

## 测试

- 代码库中没有自动化测试。
- 通过串口日志（USMART）和地面站遥测在硬件上进行验证。

## 有用参考

- **README.md** – 硬件模块、框架和外部链接的概述。
- **`.pic/`** – 接线图、PID流程图和系统架构图。
- **`MDK‑ARM/.eide/eide.yml`** – 完整的项目结构和构建设置。
- **`MDK‑ARM/build/project/builder.params`** – 确切的编译器标志、包含路径和源文件列表。
- **`Core/Inc/FreeRTOSConfig.h`** – 实时内核调优。

## 重构时同步更新架构文档

本节是上面「变更流程」的补充：只要改动属于**架构重构**（拆模块、改分层、调接口、迁位置、收敛耦合），除了按既有流程走设计-实施-回写三步，还必须**同步更新**以下三份架构文档。三份文档职责互补，缺一会让后续维护者看不到完整图景。

**适用范围**：

- 属于重构的改动：本规则强制。
- 新增功能、修复 bug、调整控制参数：不在本规则强制范围，但建议顺手把任何架构影响补到 `current-architecture.md` 的"架构变更记录"。
- 单纯改注释、改日志、改字符串：不需要动架构文档。

**重构前先读**：

- `docs/architecture/known-issues.md` – 查找本次重构对应的问题编号（F-1~F-9 / R-1~R-6 / S-1~S-4）。所有问题都有"影响 / 证据 / 建议方向 / 优先级"四字段，建议方向不是必须照做，但应作为方案起点。
- `docs/architecture/project-refactor-roadmap.md` – 确认本次重构是否在长期路线图里、是否符合推荐阶段顺序。如果偏离路线图，应在 commit 中说明原因。

**重构完成后必须改的三份文档**：

1. `docs/architecture/refactor-notes.md` – 在文件末尾追加新阶段记录，沿用既有"## 第N阶段：xxx"标题与项目符号格式。每条记录必须包含：
   - 改动文件清单。
   - 是否改变运行逻辑（任务周期、优先级、栈、算法、PID 参数、协议、初始行为）。
   - 验收方式与验证结果。
2. `docs/architecture/current-architecture.md` – 在"架构变更记录"追加日期条目（沿用 `### YYYY-MM-DD：xxx` 格式）。本次新增/移除/迁移的文件要在对应分层小节更新。
3. `docs/architecture/known-issues.md` – 把本次解决的问题从清单中移除或标记完成（保留问题编号作为历史），同时补充新发现的问题。

**禁止事项**：

- 不要把"重构改动"只写进 commit message，不更新三份文档。commit message 是临时记录，文档是长期资产。
- 不要把多个不相关的重构塞进同一个阶段记录。每阶段只做一类改动，验证后再开下一阶段。
- 不要在重构阶段顺手改算法、改任务周期、改协议——这些必须独立提交。

**改动清单模板**（commit message 或阶段记录开头可用）：

```text
本次重构对应 known-issues.md 问题：F-x / R-x / S-x
本次重构阶段：第 N 阶段
改动文件：
  - <file>
  - <file>
修了什么问题：
  - <一句话>
还有什么没做：
  - <遗留项 / 下一步>
三份架构文档同步状态：
  - [x] refactor-notes.md 已追加
  - [x] current-architecture.md 已追加
  - [x] known-issues.md 已更新
```