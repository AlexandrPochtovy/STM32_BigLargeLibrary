
#include "SpeedControl.h"

float WheelSpeedMeasure(uint32_t deltaPulse, uint32_t deltaTime) {
    return M_PI * (float)(2 * WHEEL_RAD_mm * deltaPulse * 1000) / (WHEEL_PULSE_COUNT * deltaTime);
    }

/* output value need use as act value for correct hyst processing*/
int32_t WheelSpeedZeroLimiter(int32_t act, int32_t sp, int32_t low, int32_t hi) {
    if (sp > 0) {
        if (sp < low) {
            return 0;
            }
        else if (sp > hi) {
            return sp;
            }
        else {
            if (act) {
                return sp;
                }
            else {
                return 0;
                }
            }
        }
    else if (sp < 0) {

        }
    else {
        return 0;
        }
    }


