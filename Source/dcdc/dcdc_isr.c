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



#pragma CODE_SECTION(dcdc_ISR, ".TI.ramfunc")
#pragma INTERRUPT(dcdc_ISR)
#pragma FUNC_CANNOT_INLINE(dcdc_ISR)
//
// ISR triggered by ePWM1 zero
//
__interrupt void dcdc_ISR(void)
{
    dcdc_isr_profiling_pin_set();


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
