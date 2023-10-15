
#include "SpeedControl.h"

float WheelSpeedMeasure(uint32_t deltaPulse, uint32_t deltaTime) {
    return M_PI * (float)(2 * WHEEL_RAD_mm * deltaPulse * 1000) / (WHEEL_PULSE_COUNT * deltaTime);
    }

int32_t WheelSpeedZeroLimiter(int32_t sp, int32_t low, int32_t hi) {
    if ((sp >= hi) || (sp <= low)) {
        return sp;
        }
    else {
        return 0;
        }
    }
