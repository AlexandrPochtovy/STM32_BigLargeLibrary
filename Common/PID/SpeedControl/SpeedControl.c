
#include "SpeedControl.h"

float WheelSpeedMeasure(uint32_t deltaPulse, uint32_t deltaTime) {
    return M_PI * (float)(2 * WHEEL_RAD_mm * deltaPulse * 1000) / (WHEEL_PULSE_COUNT * deltaTime);
    }

/* output value need use as act value for correct hyst processing*/
size_t WheelSpeedZeroLimiter(size_t act, size_t sp, size_t low, size_t hi) {
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
    	return act;
        }
    else {
        return 0;
        }
    }


