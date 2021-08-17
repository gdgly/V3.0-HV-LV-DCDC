///////////////////////////////////////////////////////////////////////////////
//
// ISR file
//
///////////////////////////////////////////////////////////////////////////////


//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "dcdc.h"



#define DCDC_ISR_LINE_CYCLE_PERIOD_IN_TICKS_MAX  (EPWM1_FREQ / 47U)
#define DCDC_ISR_LINE_CYCLE_PERIOD_IN_TICKS_MIN  (EPWM1_FREQ / 63U)

#define DCDC_ISR_SYNC_SIGNAL_TIMEOUT_IN_uS  (100L * MILLISECONDS_IN_uS)

#define DCDC_ISR_PLL_SLEW_RATE_MAX   10


static uint16_t dcdc_isr_sync_tick_count;
static uint16_t dcdc_isr_sync_tick_total_per_period;
static int32_t dcdc_isr_sync_zero_crossing_timestamp_us;
static bool dcdc_isr_was_positive_semicycle;
static bool dcdc_isr_is_sync_valid;
static int32_t dcdc_isr_sync_tick_timeout_us;

static uint16_t dcdc_isr_output_tick_count;
static uint16_t dcdc_isr_output_tick_total_per_period;
static volatile bool dcdc_isr_is_pll_locked;


//
//
//
static inline void dcdc_isr_profiling_pin_set(void)
{
    // GPIO 33 is bit 1 of GPBDAT
    HWREG(GPIODATA_BASE  + GPIO_O_GPBSET) = (1UL << 1);
}

//
//
//
static inline void dcdc_isr_profiling_pin_clear(void)
{
    // GPIO 33 is bit 1 of GPBDAT
    HWREG(GPIODATA_BASE  + GPIO_O_GPBCLEAR) = (1UL << 1);
}

#pragma CODE_SECTION(dcdc_isr_zero_cross_detection, ".TI.ramfunc")
//
//
// TODO: Verify if we need debouncing
static inline void dcdc_isr_zero_cross_detection(void)
{
    int32_t cpu_time_us = CPUTimer_getTimerCount(CPUTIMER0_BASE);
    if (IS_CPU_TIME_AFTER(cpu_time_us, dcdc_isr_sync_tick_timeout_us))
    {
        dcdc_isr_sync_tick_timeout_us =
                cpu_time_us + DCDC_ISR_SYNC_SIGNAL_TIMEOUT_IN_uS;
        dcdc_isr_is_sync_valid = false;
    }

    dcdc_isr_sync_tick_count++;

    int16_t v_in_raw = (int16_t)dcdc_cla_to_cpu_mem.v_in_raw.cpu;
    bool is_positive_semicycle = (v_in_raw > 0);
    if (is_positive_semicycle && (!dcdc_isr_was_positive_semicycle))
    {
        dcdc_isr_sync_zero_crossing_timestamp_us = cpu_time_us;
        dcdc_isr_sync_tick_total_per_period = dcdc_isr_sync_tick_count;
        dcdc_cpu_to_cla_mem.sync_tick_total_per_period.cpu = dcdc_isr_sync_tick_count;
        dcdc_isr_sync_tick_count = 0U;
        dcdc_isr_is_sync_valid =
                  ((dcdc_isr_sync_tick_total_per_period < DCDC_ISR_LINE_CYCLE_PERIOD_IN_TICKS_MAX)
                && (dcdc_isr_sync_tick_total_per_period > DCDC_ISR_LINE_CYCLE_PERIOD_IN_TICKS_MIN));

        // Valid sync signal, extend timeout.
        dcdc_isr_sync_tick_timeout_us =
                cpu_time_us + DCDC_ISR_SYNC_SIGNAL_TIMEOUT_IN_uS;
    }

    dcdc_isr_was_positive_semicycle = is_positive_semicycle;
}

#pragma CODE_SECTION(dcdc_isr_pll_execute, ".TI.ramfunc")
//
//
//
static inline void dcdc_isr_pll_execute(void)
{
    bool is_pll_locked = false;

    dcdc_isr_output_tick_count++;
    if (dcdc_isr_output_tick_count == dcdc_isr_output_tick_total_per_period)
    {
        dcdc_isr_output_tick_count = 0U;

        if (dcdc_isr_is_sync_valid)
        {
            int16_t total_error;
            int16_t period_error =
                    dcdc_isr_output_tick_total_per_period - dcdc_isr_sync_tick_total_per_period;
            if (period_error > DCDC_ISR_PLL_SLEW_RATE_MAX)
                total_error = DCDC_ISR_PLL_SLEW_RATE_MAX;
            else if (period_error < -DCDC_ISR_PLL_SLEW_RATE_MAX)
                total_error = -DCDC_ISR_PLL_SLEW_RATE_MAX;
            else // The frequency is close enough, adjust the phase.
            {
                int32_t dcdc_isr_output_zero_crossing_timestamp_us =
                        CPUTimer_getTimerCount(CPUTIMER0_BASE);
                int16_t phase_error =
                        ((dcdc_isr_output_zero_crossing_timestamp_us - dcdc_isr_sync_zero_crossing_timestamp_us) * EPWM1_FREQ) / SECONDS_IN_uS;

                // If the output zero crossing is closer to the following sync zero crossing
                // than to the previous one, the phase is negative.
                if (phase_error > (dcdc_isr_sync_tick_total_per_period >> 1U))
                    phase_error -= dcdc_isr_sync_tick_total_per_period;

                // TODO: Depending on the shelf information adjust the phase accordingly
//                phase_error += (dcdc_isr_output_tick_total_per_period / 3);
//                phase_error -= (dcdc_isr_output_tick_total_per_period / 3);
                total_error = period_error + phase_error;
                total_error >>= 1U; // Scale down the total error to make it less aggressive.
                if (total_error > DCDC_ISR_PLL_SLEW_RATE_MAX)
                    total_error = DCDC_ISR_PLL_SLEW_RATE_MAX;
                else if (total_error < -DCDC_ISR_PLL_SLEW_RATE_MAX)
                    total_error = -DCDC_ISR_PLL_SLEW_RATE_MAX;
                else // The total error is small.
                    is_pll_locked = true;
            }

            dcdc_isr_output_tick_total_per_period -= total_error;
        }

        dcdc_isr_is_pll_locked = is_pll_locked;
    }
}



#pragma CODE_SECTION(dcdc_ISR, ".TI.ramfunc")
#pragma INTERRUPT(dcdc_ISR)
#pragma FUNC_CANNOT_INLINE(dcdc_ISR)
//
// ISR triggered by ePWM1 zero
//
__interrupt void dcdc_ISR(void)
{
    dcdc_isr_profiling_pin_set();

    dcdc_isr_zero_cross_detection();
    dcdc_isr_pll_execute();


    // Clear EPWM1 interrupt flag so that next interrupt can come in
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    // Acknowledge the interrupt
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);


    dcdc_isr_profiling_pin_clear();
}

//
//
//
void dcdc_isr_init(void)
{
    dcdc_isr_sync_tick_count = 0U;
    dcdc_isr_sync_tick_total_per_period = 0U;
    dcdc_isr_was_positive_semicycle = true;
    dcdc_isr_is_sync_valid = false;

    dcdc_isr_output_tick_count = 0U;
    dcdc_isr_output_tick_total_per_period =
            (DCDC_ISR_LINE_CYCLE_PERIOD_IN_TICKS_MIN + DCDC_ISR_LINE_CYCLE_PERIOD_IN_TICKS_MAX) >> 1U;
    dcdc_isr_is_pll_locked = false;

    Interrupt_register(INT_EPWM1, &dcdc_ISR);
    Interrupt_enable(INT_EPWM1);
}

//
//
//
bool dcdc_isr_is_pll_locked_get(void)
{
    return dcdc_isr_is_pll_locked;
}
