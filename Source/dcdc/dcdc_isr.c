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



#define DCDC_NEGATIVE_OUTPUT_CURRENT_QUALIFY       0.0f
#define DCDC_NEGATIVE_OUTPUT_CURRENT_QUALIFY_RAW   \
        DCDC_OUTPUT_CURRENT_REVERSE_DEFAULT_CALIBRATION_CALCULATE(DCDC_NEGATIVE_OUTPUT_CURRENT_QUALIFY)
#define DCDC_NEGATIVE_OUTPUT_CURRENT_CUTOFF        2.0f
#define DCDC_NEGATIVE_OUTPUT_CURRENT_CUTOFF_RAW    \
        DCDC_OUTPUT_CURRENT_REVERSE_DEFAULT_CALIBRATION_CALCULATE(DCDC_NEGATIVE_OUTPUT_CURRENT_CUTOFF)



//
//
//
static inline void dcdc_isr_profiling_pin_set(void)
{
    // GPIO34 is bit 2 of GPBDAT
    HWREG(GPIODATA_BASE  + GPIO_O_GPBSET) = (1UL << 2);
}

//
//
//
static inline void dcdc_isr_profiling_pin_clear(void)
{
    // GPIO34 is bit 2 of GPBDAT
    HWREG(GPIODATA_BASE  + GPIO_O_GPBCLEAR) = (1UL << 2);
}

//
//
//
static inline void dcdc_isr_oring_fet_enable(void)
{
    GPIO_writePin(12, 1);
}

//
//
//
static inline void dcdc_isr_oring_fet_disable(void)
{
    GPIO_writePin(12, 0);
}

#pragma CODE_SECTION(dcdc_isr_oring_fet_service, ".TI.ramfunc")
//
//
//
static inline void dcdc_isr_oring_fet_service(void)
{
    int16_t i_out_raw = dcdc_cla_to_cpu_mem.i_out_raw_filtered.cpu;
    if (i_out_raw < DCDC_NEGATIVE_OUTPUT_CURRENT_QUALIFY_RAW)
        dcdc_isr_oring_fet_enable();
    else if (i_out_raw > DCDC_NEGATIVE_OUTPUT_CURRENT_CUTOFF_RAW)
        dcdc_isr_oring_fet_disable();
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

    dcdc_isr_oring_fet_service();


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
    Interrupt_register(INT_EPWM1, &dcdc_ISR);
    Interrupt_enable(INT_EPWM1);
}
