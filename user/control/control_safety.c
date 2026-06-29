#include "control_safety.h"

#include <math.h>
#include <stdbool.h>

#include "alarm.h"
#include "commander.h"
#include "control_output.h"
#include "log.h"

void safeCheck(MotorCtrl* ctrl, state_t* state)
{
    static bool safety_triggered = false;

    if (!isfinite(state->angle.roll) || !isfinite(state->angle.pitch) ||
        !isfinite(state->gyro.x) || !isfinite(state->gyro.y) || !isfinite(state->gyro.z) ||
        !isfinite(state->height))
    {
        setCommanderSafetyLatched(true);
        setCommanderKeyFlight(false);
        setCommanderKeyland(false);
        state->isRCLocked = true;

        ctrl->Esc_Percent_1 = 0;
        ctrl->Esc_Percent_2 = 0;
        ctrl->Esc_Percent_3 = 0;
        ctrl->Esc_Percent_4 = 0;

        Alarm_SetMode(ALARM_MODE_ERROR);
        MotorControl(ctrl);
        LOG_ERROR("safeCheck invalid state value, force lock");
        safety_triggered = true;
        return;
    }

    if (getCommanderAttitudeMode() == MODE_AIRPLANE && (fabsf(state->angle.roll) > 45 || fabsf(state->angle.pitch) > 45))
    {
        if (!safety_triggered)
        {
            setCommanderKeyFlight(false);
            setCommanderKeyland(false);

            ctrl->Esc_Percent_1 = 0;
            ctrl->Esc_Percent_2 = 0;
            ctrl->Esc_Percent_3 = 0;
            ctrl->Esc_Percent_4 = 0;

            Alarm_SetMode(ALARM_MODE_ERROR);

            MotorControl(ctrl);

            LOG_ERROR("安全触发：姿态超限 (roll=%.1f°, pitch=%.1f°)", state->angle.roll, state->angle.pitch);
            safety_triggered = true;
        }
    }
    else
    {
        if (safety_triggered)
        {
            LOG_INFO("安全状态恢复：姿态正常");
            safety_triggered = false;
        }
    }
}