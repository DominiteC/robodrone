# 变形时轮子振荡设计

## 问题

变形（飞行↔陆行）过程中，电杆伸缩时轮子不转，机器人在硬地上靠电杆硬蹭，容易卡住。

## 方案

在电杆伸缩期间，让有刷电机（轮子）以低频来回滚动，把地面摩擦力转换为旋转力，避免卡死。

## 改动范围

仅修改 `user/control/change.c`。

## 新增配置常量

```c
#define WHEEL_OSCILLATE_PWM            800   // 轮子振荡 PWM 幅度（最大 4000 的 20%）
#define WHEEL_OSCILLATE_HALF_PERIOD_MS 300   // 半周期 ms（每个方向持续 300ms，完整来回 600ms）
```

## 新增静态辅助函数

`transformActuatorWithWheels(bool extend, uint16_t duration_ms)`：

1. 调用 `Actuator_Start(1~4, extend)` 启动全部 4 个电杆
2. 循环 `while (elapsed < duration_ms)`：
   - `Motor_Set_PWM(pwm, pwm, pwm, pwm)` 四个轮子同向
   - `vTaskDelay(300ms)`
   - `pwm = -pwm` 反转方向
   - `elapsed += 300`
3. 停止全部电杆，轮子 PWM 归零

## 修改 changeServoMode()

两处替换：
- `Actuator_AllSet(true, 15000)` → `transformActuatorWithWheels(true, 15000)` （陆行→飞行，伸长）
- `Actuator_AllSet(false, 13300)` → `transformActuatorWithWheels(false, 13300)` （飞行→陆行，收缩）

## 参数说明

- PWM=800（20%）：仅用于卸力，不产生实际驱动效果，行走时满油门 PWM 约 3750
- 半周期 300ms：在 500ms-1s 的完整周期范围内取中间值
