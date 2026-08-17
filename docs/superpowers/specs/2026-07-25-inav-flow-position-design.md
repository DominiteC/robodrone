# F-5: INAV 风格位置/速度双残差校正 — 设计文档

> 状态：设计阶段，待 F-4 飞行稳定后实施
> 参考：MiniFly `state_estimator.c`（移植自 inav-1.9.0 / PX4 Anton Babushkin）

## 1. 目标

将 `position.c` 从当前的**互补滤波 + P 位置校正**，改为 MiniFly 同款的 **INAV Predict-Correct 框架**：

- Predict 步用 IMU 加速度同时预测位置和速度
- CorrectPos 步用光流位置残差同时修正位置和速度（联动）
- CorrectVel 步用光流速度残差修正速度

## 2. 现状 vs 目标

### 2.1 当前架构（position.c）

```
每轮 position_Update():
  ┌─ IMU 预积分 ─────────────────────────┐
  │ imu_pred_v += acc_h · dt              │  ← 只积速度，不预测位置
  └───────────────────────────────────────┘
  ┌─ 速度融合（光流有效时）─────────────────┐
  │ final_vx = raw_vx · tilt               │  光流速度为主
  │ final_vx += 0.15·(imu_pred_vx - final) │  互补滤波
  │ imu_pred_vx = final_vx                 │  同步防跳变
  └───────────────────────────────────────┘
  ┌─ 位置积分 + P 校正 ────────────────────┐
  │ position += velocity · dt              │  融合速度积分
  │ opflow_sum += raw_vx · dt             │  纯光流独立累积
  │ position += residual · 2.0 · dt        │  P 校正，不动速度
  └───────────────────────────────────────┘
```

**核心问题：位置和速度解耦。**
- 速度偏了 → 位置积分偏 → P 校正只拉位置，不管速度
- 加速度有偏置 → imu_pred_v 持续漂 → 速度被污染 → 位置越来越偏
- 没有位置 → 速度的反向修正通道

### 2.2 目标架构（MiniFly INAV）

```
每轮 position_Update():
  ┌─ Predict ─────────────────────────────┐
  │ pos += vel·dt + (acc-accBias)·dt²/2    │  加速度预测位置
  │ vel += (acc-accBias)·dt                │  加速度预测速度
  └───────────────────────────────────────┘
  ┌─ CorrectVel（光流有效时）───────────────┐
  │ e = opflow_vel - estimator.vel         │  速度残差
  │ vel += e · wV · dt                     │  速度校正
  └───────────────────────────────────────┘
  ┌─ CorrectPos（光流有效时）───────────────┐
  │ e = opflow_pos - estimator.pos         │  位置残差
  │ pos += e · wP · dt                     │  位置校正
  │ vel += wP · e · wP · dt   ← 联动！     │  速度也得到修正
  └───────────────────────────────────────┘
  ┌─ AccBias 估计（可选，低频）─────────────┐
  │ accBias += corr · wAccBias · dt        │  慢速估计零偏
  └───────────────────────────────────────┘
```

### 2.3 关键差异对照表

| 维度 | 当前 | INAV 目标 |
|---|---|---|
| 位置预测 | ❌ 无（纯速度积分） | ✅ `pos += vel·dt + acc·dt²/2` |
| 速度来源 | 互补滤波混合 | Predict 一步到位 + Correct 微调 |
| 位置→速度联动 | ❌ | ✅ `vel += wP² · e · dt` |
| 加速度偏置 | ❌ | ✅ 在线估计 |
| 观测模型 | 光流速度 + 光流位置独立累积 | 光流速度残差 + 光流位置残差 |
| 框架 | 互补滤波 + P 校正 | Predict-Correct（类 Kalman 结构但固定增益） |

## 3. 详细设计

### 3.1 状态变量改动

```c
// === 保留 ===
static float imu_pred_vx, imu_pred_vy, imu_pred_vz;  // IMU 速度预测（失联兜底仍需要）
static float imu_pred_dt_total;                       // 失联累计时间

// === 改名/重新定义 ===
// position.x/y      → estimator.pos_x, estimator.pos_y   (现在是状态，不只是输出)
// velocity.x/y      → estimator.vel_x, estimator.vel_y   (现在是 Predict 更新，不是互补滤波输出)
// opflow_pos_sum_x/y → 保留，用于计算位置残差

// === 新增 ===
static float acc_bias_x, acc_bias_y;   // 加速度零偏估计 (cm/s²)
static float opflow_vel_ref_x, opflow_vel_ref_y; // 本帧光流速度观测
static float opflow_pos_ref_x, opflow_pos_ref_y; // 光流累积位置观测

// === 删除 ===
// IMU_PRED_BLEND_WEIGHT 0.15  ← 不再用互补滤波
// POSITION_FLOW_CORRECTION_GAIN 2.0 ← 替换为 CorrectPos 机制
```

### 3.2 参数定义

```c
/* ---- INAV 校正权重 ---- */
#define INAV_W_OPFLOW_POS   1.0f    /* 位置残差权重, 参考 MiniFly wOpflowP */
#define INAV_W_OPFLOW_VEL   2.0f    /* 速度残差权重, 参考 MiniFly wOpflowV */
#define INAV_W_ACC_BIAS     0.005f  /* 加速度零偏估计权重 (慢速), 参考 MiniFly wAccBias */
#define INAV_ACC_BIAS_MAX   50.0f   /* 加速度零偏上限 (cm/s²), 参考 MiniFly INAV_ACC_BIAS_ACCEPTANCE_VALUE */
```

### 3.3 Predict 步

```c
/* 每轮都执行，不管光流是否有效 */
static void inav_predict_xy(float dt, float acc_x, float acc_y,
                            float pitch_rad, float roll_rad)
{
    /* 重力补偿（复用现有逻辑） */
    float acc_hx = acc_x - IMU_GRAVITY * sinf(pitch_rad);
    float acc_hy = acc_y + IMU_GRAVITY * sinf(roll_rad);

    /* 去偏置 */
    acc_hx -= acc_bias_x;
    acc_hy -= acc_bias_y;

    /* Predict: 加速度同时更新位置和速度 */
    estimator.pos_x += estimator.vel_x * dt + acc_hx * dt * dt * 0.5f;
    estimator.pos_y += estimator.vel_y * dt + acc_hy * dt * dt * 0.5f;
    estimator.vel_x += acc_hx * dt;
    estimator.vel_y += acc_hy * dt;

    /* 速度限幅 */
    estimator.vel_x = limit_velocity(estimator.vel_x);
    estimator.vel_y = limit_velocity(estimator.vel_y);

    /* 同步 IMU 预测速度（失联兜底仍需要） */
    imu_pred_vx = estimator.vel_x;
    imu_pred_vy = estimator.vel_y;
}
```

### 3.4 CorrectVel 步

```c
/* 光流有效时执行，用光流速度修正估计器速度 */
static void inav_correct_vel_xy(float dt)
{
    float e_vx = opflow_vel_ref_x - estimator.vel_x;
    float e_vy = opflow_vel_ref_y - estimator.vel_y;

    estimator.vel_x += e_vx * INAV_W_OPFLOW_VEL * dt;
    estimator.vel_y += e_vy * INAV_W_OPFLOW_VEL * dt;
}
```

### 3.5 CorrectPos 步（含联动速度修正）

```c
/* 光流有效时执行，用光流位置修正估计器位置，同时联动修正速度 */
static void inav_correct_pos_xy(float dt)
{
    float e_px = opflow_pos_ref_x - estimator.pos_x;
    float e_py = opflow_pos_ref_y - estimator.pos_y;

    float ewdt_x = e_px * INAV_W_OPFLOW_POS * dt;
    float ewdt_y = e_py * INAV_W_OPFLOW_POS * dt;

    /* 位置校正 */
    estimator.pos_x += ewdt_x;
    estimator.pos_y += ewdt_y;

    /* 联动：速度也得到修正（wP² · e · dt）*/
    estimator.vel_x += INAV_W_OPFLOW_POS * ewdt_x;
    estimator.vel_y += INAV_W_OPFLOW_POS * ewdt_y;
}
```

### 3.6 加速度偏置估计（可选第一版不做）

```c
/* 用 Z 轴高度残差反推加速度偏置（与 MiniFly 一致） */
static void inav_estimate_acc_bias(float dt, float err_pos_z, float w_baro)
{
    /* 残差 → 偏置修正量 */
    float corr_z = -err_pos_z * w_baro * w_baro;
    float mag_sq = acc_bias_x*acc_bias_x + acc_bias_y*acc_bias_y + corr_z*corr_z;

    if (mag_sq < INAV_ACC_BIAS_MAX * INAV_ACC_BIAS_MAX) {
        /* 需要旋转到机体坐标系（与 MiniFly 一致） */
        // imuTransformVectorEarthToBody(&corr);
        // acc_bias_x += corr.x * INAV_W_ACC_BIAS * dt;
        // acc_bias_y += corr.y * INAV_W_ACC_BIAS * dt;
    }
}
```

> **第一版建议暂不实现加速度偏置估计。** 等 Predict-Correct 框架跑稳后再加，降低首次调试复杂度。

### 3.7 光流观测量的构建

```c
/* 光流速度观测：当前 raw_vx/vy（已有，不变） */
opflow_vel_ref_x = raw_vx * tilt_factor;   /* 含倾斜补偿 */
opflow_vel_ref_y = raw_vy * tilt_factor;

/* 光流位置观测：光流速度独立累积（复用现有 opflow_pos_sum） */
opflow_pos_sum_x += raw_vx * xy_dt;   /* 不用倾斜补偿速度，用原始速度 */
opflow_pos_sum_y += raw_vy * xy_dt;
opflow_pos_ref_x = opflow_pos_sum_x;
opflow_pos_ref_y = opflow_pos_sum_y;
```

### 3.8 主循环伪代码

```c
void position_Update(const float_angle* angle, const float_gyro* gyro)
{
    uint32_t now = getGlobalTime();
    MtfSample s;
    bool have_sample = mtf_01_get_latest_sample(&s, now);
    bool flow_valid = mtf_01_is_flow_usable(now);

    float dt = compute_dt(now, last_update_ms);
    last_update_ms = now;

    /* ===== Step 1: Predict（每轮都跑）===== */
    float acc_x = stcAcc.a[0] * 100.f;  /* m/s² → cm/s² */
    float acc_y = stcAcc.a[1] * 100.f;
    float pitch_rad = stcAngle.Angle[1] * DEG_TO_RAD;
    float roll_rad  = stcAngle.Angle[0] * DEG_TO_RAD;
    inav_predict_xy(dt, acc_x, acc_y, pitch_rad, roll_rad);

    if (have_sample) {
        /* TOF 预处理（保持不变） */
        process_tof(&s);

        if (tof_valid) {
            update_tilt_compensation(angle);
            build_flow_observation(&s);  /* 构建 opflow_vel_ref, opflow_pos_ref */

            if (flow_valid) {
                /* ===== Step 2: CorrectVel ===== */
                inav_correct_vel_xy(dt);

                /* ===== Step 3: CorrectPos ===== */
                inav_correct_pos_xy(dt);

                /* 更新外部接口 */
                velocity.x = estimator.vel_x;
                velocity.y = estimator.vel_y;
                /* position 改为直接暴露 estimator.pos */
            } else {
                /* 光流失效：只用 Predict，不校正 */
                /* imu_pred_v 仍用于失联兜底 */
                decay_velocity_on_timeout();
            }
        } else {
            decay_xy_velocity();
        }
    }
}
```

### 3.9 光流失效处理

| 状态 | Predict | CorrectVel | CorrectPos | 位置 |
|---|---|---|---|---|
| 光流有效 | ✅ | ✅ | ✅ | 完整估计 |
| 光流失效 < 300ms | ✅ | ❌ | ❌ | Predict 维持 |
| 光流失效 > 300ms | ✅ 但速度衰减 | ❌ | ❌ | 逐渐漂移 |
| TOF 失效 | ❌ | ❌ | ❌ | 全部衰减归零 |

光流恢复时的处理：`opflow_pos_sum` 清零、位置从当前 Predict 位置继续（不做跳变），让 CorrectPos 慢慢拉回。

## 4. 与现有 F-4 的关系

当前代码已完成 F-4 的大部分工作（IMU 短时预测 + 光流速度融合）。F-5 在 F-4 之上做三件事：

| F-4 已有 | F-5 改动 |
|---|---|
| `imu_pred_v` 积分 | 改为 Predict 步，同时更新 pos 和 vel |
| 互补滤波融合 | 改为 CorrectVel 残差校正 |
| P 位置残差校正 | 改为 CorrectPos（含联动修正速度） |
| `opflow_pos_sum` | 保留，作为 CorrectPos 的观测输入 |

**F-5 实施前需要 F-4 飞行稳定**（路线图约定），确保基础加速度数据可信。

## 5. 验收标准（与路线图一致）

- 定点模式下手动阶跃目标后位置超调 < 30cm、稳态误差 < 10cm
- 圆周航线圆心漂移 < 50cm
- 光流失联 200ms 内 XY 漂移 < 10cm（保持 F-4 的指标）
- 恢复后无姿态跳变

## 6. 风险

| 风险 | 缓解 |
|---|---|
| `wOpflowP=1.0` 太激进，光流噪声被放大 | 上板后第一版可设 `wOpflowP=0.5`、`wOpflowV=1.0`，逐步调回 |
| Predict 中加速度零偏被积分放大 | 第一版不加 accBias 估计，依赖 F-3 验证后的加速度质量 |
| 光流恢复时位置跳变 | CorrectPos 不重置位置，用残差缓慢拉回 |
| 与现有 PID 控制环耦合 | 不动 PID 参数，`velocity` 和 `position` 接口不变 |

## 7. 改动文件清单

| 文件 | 改动 |
|---|---|
| `project/user/abstract/position.c` | 核心改动：Predict/CorrectVel/CorrectPos 函数、主循环重构 |
| `project/user/abstract/position.h` | 可能新增 `position_GetEstimatorState()` 用于遥测 |
| `project/user/abstract/control_pid.c` 或遥控器命令 | `position_ResetFlowState()` 需同时重置 estimator 和 accBias |

## 8. 显式不做

- 不上 EKF / Kalman 滤波
- 不引入协方差矩阵
- 不动 PID 参数
- 不改变任务周期/优先级/栈
- 不改变 `velocity` 和 `position` 的外部接口语义
