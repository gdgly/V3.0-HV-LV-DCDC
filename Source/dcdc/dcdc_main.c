///////////////////////////////////////////////////////////////////////////////
//
// Main project file
//
///////////////////////////////////////////////////////////////////////////////


//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "util.h"
#include "dcdc.h"
#include "spi_flash/spi_flash.h"
#include "pers/pers.h"


#define DCDC_STATE_MACHINE_SERVICE_EXPIRY_uS 100L
#define DCDC_SPI_FLASH_SERVICE_EXPIRY_uS     250L
#define DCDC_PERS_SERVICE_EXPIRY_uS          (5L * MILLISECONDS_IN_uS)
#define DCDC_SCI_SERVICE_EXPIRY_uS           (1L * MILLISECONDS_IN_uS)
#define DCDC_TEMPERATURE_FILTERING_EXPIRY_uS (1L * MILLISECONDS_IN_uS)
#define DCDC_LED_SERVICE_EXPIRY_uS           (1L * SECONDS_IN_uS)
#define DCDC_FAN_RPM_SERVICE_EXPIRY_uS       (1L * SECONDS_IN_uS)



uint32_t fan_rpm;


//
//
//
void main(void)
{
    // Initialize peripherals
    dcdc_init();

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;           // Enable Global interrupt INTM
    ERTM;           // Enable Global realtime interrupt DBGM

    int32_t state_machine_service_expiry_time_us =
            CPUTimer_getTimerCount(CPUTIMER0_BASE) + DCDC_STATE_MACHINE_SERVICE_EXPIRY_uS;
    int32_t spi_flash_service_expiry_time_us =
            CPUTimer_getTimerCount(CPUTIMER0_BASE) + DCDC_SPI_FLASH_SERVICE_EXPIRY_uS;
    int32_t pers_service_expiry_time_us =
            CPUTimer_getTimerCount(CPUTIMER0_BASE) + DCDC_PERS_SERVICE_EXPIRY_uS;
    int32_t sci_service_expiry_time_us =
            CPUTimer_getTimerCount(CPUTIMER0_BASE) + DCDC_SCI_SERVICE_EXPIRY_uS;
    int32_t temperature_filtering_expiry_time_us =
            CPUTimer_getTimerCount(CPUTIMER0_BASE) + DCDC_TEMPERATURE_FILTERING_EXPIRY_uS;
    int32_t led_service_expiry_time_us =
            CPUTimer_getTimerCount(CPUTIMER0_BASE) + DCDC_LED_SERVICE_EXPIRY_uS;
    int32_t fan_rpm_service_expiry_time_us =
            CPUTimer_getTimerCount(CPUTIMER0_BASE) + DCDC_FAN_RPM_SERVICE_EXPIRY_uS;

    while (1)
    {
        int32_t cpu_time_us = CPUTimer_getTimerCount(CPUTIMER0_BASE);

        if (IS_CPU_TIME_AFTER(cpu_time_us, state_machine_service_expiry_time_us))
        {
            dcdc_state_machine_service();
            state_machine_service_expiry_time_us += DCDC_STATE_MACHINE_SERVICE_EXPIRY_uS;
        }

        if (IS_CPU_TIME_AFTER(cpu_time_us, spi_flash_service_expiry_time_us))
        {
            spi_flash_service();
            spi_flash_service_expiry_time_us += DCDC_SPI_FLASH_SERVICE_EXPIRY_uS;
        }

        if (IS_CPU_TIME_AFTER(cpu_time_us, pers_service_expiry_time_us))
        {
            pers_service();
            pers_service_expiry_time_us += DCDC_PERS_SERVICE_EXPIRY_uS;
        }

        if (IS_CPU_TIME_AFTER(cpu_time_us, sci_service_expiry_time_us))
        {
            dcdc_sdp_service();
            sci_service_expiry_time_us += DCDC_SCI_SERVICE_EXPIRY_uS;
        }

        if (IS_CPU_TIME_AFTER(cpu_time_us, temperature_filtering_expiry_time_us))
        {
            dcdc_temperatures_filtering_service();
            temperature_filtering_expiry_time_us += DCDC_TEMPERATURE_FILTERING_EXPIRY_uS;
        }

        if (IS_CPU_TIME_AFTER(cpu_time_us, led_service_expiry_time_us))
        {
            GPIO_togglePin(31);
            GPIO_togglePin(34);
            led_service_expiry_time_us += DCDC_LED_SERVICE_EXPIRY_uS;
        }

        if (IS_CPU_TIME_AFTER(cpu_time_us, fan_rpm_service_expiry_time_us))
        {
            fan_rpm = dcdc_fan_rpm_get();
            fan_rpm_service_expiry_time_us += DCDC_FAN_RPM_SERVICE_EXPIRY_uS;
        }

        SysCtl_serviceWatchdog();
    }
}
