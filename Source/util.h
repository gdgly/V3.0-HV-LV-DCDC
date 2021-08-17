///////////////////////////////////////////////////////////////////////////////
//
// Utilities Include file
//
///////////////////////////////////////////////////////////////////////////////

#ifndef UTIL_H_
#define UTIL_H_

//
// Included Files
//
#include "driverlib.h"
#include "device.h"


#define IS_CPU_TIME_AFTER(cpu_time, expiry)     (((int32_t)(cpu_time) - (int32_t)(expiry)) > 0L)
#define IS_CPU_TIME_BEFORE(cpu_time, expiry)    (((int32_t)(cpu_time) - (int32_t)(expiry)) < 0L)

#define SECONDS_IN_uS       1000000L
#define MILLISECONDS_IN_uS     1000L

#define SQRT_2      1.414213562373f
#define SQRT_3      1.732050807568f
#define PI          3.141592653589f

#define Q16_TO_FLOAT(x) ((float32_t)(x) / 65535.0f)
#define FLOAT_TO_Q16(x) ((int32_t)((x) * 65535.0f))

#define PWM_PERIOD_IN_COUNTS_UP_COUNTER(freq)       (uint16_t)((DEVICE_SYSCLK_FREQ / (freq)) - 1UL)
#define PWM_PERIOD_IN_COUNTS_UP_DOWN_COUNTER(freq)  (uint16_t)((DEVICE_SYSCLK_FREQ / (freq)) >> 1U)



#endif
