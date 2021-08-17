///////////////////////////////////////////////////////////////////////////////
//
// Slow processes file
//
///////////////////////////////////////////////////////////////////////////////


//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "dcdc.h"
#include "ntc/ntc.h"

#define DCDC_BUS_VOLTAGE_REVERSE_CALIBRATION_CALCULATE(v)    (uint16_t)(((v) \
        - DCDC_CALIBRATION_V_BUS_OFFSET_DEFAULT) / DCDC_CALIBRATION_V_BUS_SLOPE_DEFAULT)

#define DCDC_BUS_VOLTAGE_UVP_CUTOFF      200.0f
#define DCDC_BUS_VOLTAGE_UVP_CUTOFF_RAW  \
    DCDC_BUS_VOLTAGE_REVERSE_CALIBRATION_CALCULATE(DCDC_BUS_VOLTAGE_UVP_CUTOFF)
#define DCDC_BUS_VOLTAGE_UVP_QUALIFY     250.0f
#define DCDC_BUS_VOLTAGE_UVP_QUALIFY_RAW  \
    DCDC_BUS_VOLTAGE_REVERSE_CALIBRATION_CALCULATE(DCDC_BUS_VOLTAGE_UVP_QUALIFY)

#define DCDC_BUS_VOLTAGE_OVP_CUTOFF      410.0f
#define DCDC_BUS_VOLTAGE_OVP_CUTOFF_RAW  \
    DCDC_BUS_VOLTAGE_REVERSE_CALIBRATION_CALCULATE(DCDC_BUS_VOLTAGE_OVP_CUTOFF)
#define DCDC_BUS_VOLTAGE_OVP_QUALIFY     390.0f
#define DCDC_BUS_VOLTAGE_OVP_QUALIFY_RAW  \
    DCDC_BUS_VOLTAGE_REVERSE_CALIBRATION_CALCULATE(DCDC_BUS_VOLTAGE_OVP_QUALIFY)


#define DCDC_AMBIENT_TEMPERATURE_OTP_x100_CUTOFF     (70 * 100)
#define DCDC_AMBIENT_TEMPERATURE_OTP_x100_QUALIFY    (60 * 100)

#define DCDC_HEATSINK_TEMPERATURE_OTP_x100_CUTOFF    (100 * 100)
#define DCDC_HEATSINK_TEMPERATURE_OTP_x100_QUALIFY   ( 90 * 100)

#define DCDC_LINE_CYCLE_PERIOD_IN_TICKS_HIGH_CUTOFF  (EPWM1_FREQ / 47U)
#define DCDC_LINE_CYCLE_PERIOD_IN_TICKS_HIGH_QUALIFY (EPWM1_FREQ / 48U)
#define DCDC_LINE_CYCLE_PERIOD_IN_TICKS_LOW_QUALIFY  (EPWM1_FREQ / 62U)
#define DCDC_LINE_CYCLE_PERIOD_IN_TICKS_LOW_CUTOFF   (EPWM1_FREQ / 63U)

#define DCDC_INPUT_VOLTAGE_UVP_CUTOFF_RMS         90.0f
#define DCDC_INPUT_VOLTAGE_UVP_QUALIFY_RMS       100.0f

#define DCDC_INPUT_VOLTAGE_OVP_CUTOFF_RMS        200.0f
#define DCDC_INPUT_VOLTAGE_OVP_QUALIFY_RMS       190.0f

#define DCDC_INPUT_CURRENT_OCP_CUTOFF_RMS         60.0f
#define DCDC_INPUT_CURRENT_OCP_QUALIFY_RMS        55.0f

#define DCDC_INPUT_OCP_RECURRENCE_THRESHOLD  5


static bool dcdc_master_startup_shutdown;
static bool dcdc_open_loop_enable;
static bool dcdc_open_loop_hf_legs_enable;
static bool dcdc_open_loop_lf_leg_enable;
static bool dcdc_open_loop_inrush_protection_enable;

static int16_t dcdc_ambient_temperature_x100;
static int16_t dcdc_heatsink_1_temperature_x100;
static int16_t dcdc_heatsink_2_temperature_x100;
static int16_t dcdc_llc_primary_heatsink_temperature_x100;

static uint16_t dcdc_input_voltage_uvp_cutoff_rms_raw;
static uint16_t dcdc_input_voltage_uvp_qualify_rms_raw;
static uint16_t dcdc_input_voltage_ovp_cutoff_rms_raw;
static uint16_t dcdc_input_voltage_ovp_qualify_rms_raw;
static uint16_t dcdc_input_current_ocp_cutoff_rms_raw;
static uint16_t dcdc_input_current_ocp_qualify_rms_raw;

static int16_t dcdc_input_over_current_recurrence_counter;

static bool dcdc_bus_under_voltage;
static bool dcdc_bus_over_voltage;
static bool dcdc_input_over_current;
static bool dcdc_input_current_rms_over_current;
static bool dcdc_ambient_over_temperature;
static bool dcdc_heatsink_1_over_temperature;
static bool dcdc_heatsink_2_over_temperature;
static bool dcdc_llc_primary_heatsink_over_temperature;
static bool dcdc_input_frequency_out_of_range;
static bool dcdc_input_voltage_rms_under_voltage;
static bool dcdc_input_voltage_rms_over_voltage;

static bool dcdc_non_critical_faults_active;
static bool dcdc_critical_faults_active;

static uint16_t dcdc_bus_under_voltage_counter;
static uint16_t dcdc_bus_over_voltage_counter;
static uint16_t dcdc_input_over_current_counter;
static uint16_t dcdc_input_current_rms_over_current_counter;
static uint16_t dcdc_ambient_over_temperature_counter;
static uint16_t dcdc_heatsink_1_over_temperature_counter;
static uint16_t dcdc_heatsink_2_over_temperature_counter;
static uint16_t dcdc_llc_primary_heatsink_over_temperature_counter;
static uint16_t dcdc_input_frequency_out_of_range_counter;
static uint16_t dcdc_input_voltage_rms_under_voltage_counter;
static uint16_t dcdc_input_voltage_rms_over_voltage_counter;

static int32_t dcdc_inrush_relay_to_close_delay_us;

//
//
//
static void dcdc_pwm_hf_legs_enable_and_unlock(void)
{
    EPWM_clearTripZoneFlag(EPWM1_BASE, EPWM_TZ_FLAG_DCAEVT1);
    EPWM_clearTripZoneFlag(EPWM2_BASE, EPWM_TZ_FLAG_DCAEVT1);
}

//
//
//
static void dcdc_pwm_hf_legs_disable_and_lock(void)
{
    EPWM_forceTripZoneEvent(EPWM1_BASE, EPWM_TZ_FORCE_EVENT_DCAEVT1);
    EPWM_forceTripZoneEvent(EPWM2_BASE, EPWM_TZ_FORCE_EVENT_DCAEVT1);
}

//
//
//
static void dcdc_pwm_lf_leg_enable_and_unlock(void)
{
    EPWM_clearTripZoneFlag(EPWM3_BASE, EPWM_TZ_FLAG_DCAEVT1);
}

//
//
//
static void dcdc_pwm_lf_leg_disable_and_lock(void)
{
    EPWM_forceTripZoneEvent(EPWM3_BASE, EPWM_TZ_FORCE_EVENT_DCAEVT1);
}

//
//
//
static void dcdc_input_relay_close(void)
{
    GPIO_writePin(12, 1);
}

//
//
//
static void dcdc_input_relay_open(void)
{
    GPIO_writePin(12, 0);
}

//
//
//
static void dcdc_primary_fault_signal_set(void)
{
    GPIO_writePin(17, 1);
}

//
//
//
static void dcdc_primary_fault_signal_clear(void)
{
    GPIO_writePin(17, 0);
}

//
//
//
static void dcdc_shutdown_command_service(void)
{
    if (!dcdc_master_startup_shutdown)
    {
        dcdc_pwm_hf_legs_disable_and_lock();
        dcdc_pwm_lf_leg_disable_and_lock();
        dcdc_input_relay_open();
        dcdc_open_loop_enable = false;
        dcdc_open_loop_hf_legs_enable = false;
        dcdc_open_loop_lf_leg_enable = false;
        dcdc_open_loop_inrush_protection_enable = true;
        dcdc_cpu_to_cla_mem.dcdc_state.cpu = DCDC_STATE_SHUTDOWN;
    }
}

//
//
//
static void dcdc_fault_shutdown_service(void)
{
    if (dcdc_critical_faults_active)
    {
        if (dcdc_cpu_to_cla_mem.dcdc_state.cpu
                        > DCDC_STATE_WAITING_FOR_INPUT_VOLTAGE_QUALIFICATION)
                    dcdc_cpu_to_cla_mem.dcdc_state.cpu =
                            DCDC_STATE_WAITING_FOR_CRITICAL_FAULTS_TO_CLEAR;
    }
    else if (dcdc_non_critical_faults_active)
    {
        if (dcdc_cpu_to_cla_mem.dcdc_state.cpu
                > DCDC_STATE_WAITING_FOR_NON_CRITICAL_FAULTS_TO_CLEAR)
            dcdc_cpu_to_cla_mem.dcdc_state.cpu =
                    DCDC_STATE_WAITING_FOR_NON_CRITICAL_FAULTS_TO_CLEAR;
    }
}

//
//
//
static void primary_fault_signal_service(void)
{
    if (dcdc_bus_under_voltage)
        dcdc_primary_fault_signal_set();
    else if (dcdc_cpu_to_cla_mem.dcdc_state.cpu == DCDC_STATE_NORMAL)
        dcdc_primary_fault_signal_clear();
}

//
//
//
static void dcdc_bus_under_voltage_evaluate(void)
{
    uint16_t v_bus_raw = dcdc_cla_to_cpu_mem.v_bus_raw.cpu;
    if (v_bus_raw < DCDC_BUS_VOLTAGE_UVP_CUTOFF_RAW)
    {
        ++dcdc_bus_under_voltage_counter;
        dcdc_bus_under_voltage = true;
    }
    else if (v_bus_raw > DCDC_BUS_VOLTAGE_UVP_QUALIFY_RAW)
        dcdc_bus_under_voltage = false;
}

//
//
//
static bool dcdc_bus_over_voltage_evaluate(void)
{
    uint16_t v_bus_raw = dcdc_cla_to_cpu_mem.v_bus_raw.cpu;
    if (v_bus_raw > DCDC_BUS_VOLTAGE_OVP_CUTOFF_RAW)
    {
        ++dcdc_bus_over_voltage_counter;
        dcdc_bus_over_voltage = true;
    }
    else if (v_bus_raw < DCDC_BUS_VOLTAGE_OVP_QUALIFY_RAW)
        dcdc_bus_over_voltage = false;

    return dcdc_bus_over_voltage;
}

//
//
//
static bool dcdc_input_over_current_evaluate(void)
{
    if (EPWM_getTripZoneFlagStatus(EPWM1_BASE) & EPWM_TZ_FLAG_CBC)
    {
        EPWM_clearTripZoneFlag(EPWM1_BASE, EPWM_TZ_FLAG_CBC);
        dcdc_input_over_current_recurrence_counter++;
    }
    else
        dcdc_input_over_current_recurrence_counter--;

    if (dcdc_input_over_current_recurrence_counter >= DCDC_INPUT_OCP_RECURRENCE_THRESHOLD)
    {
        dcdc_input_over_current_recurrence_counter = DCDC_INPUT_OCP_RECURRENCE_THRESHOLD;
        ++dcdc_input_over_current_counter;
        dcdc_input_over_current = true;
    }
    else if (dcdc_input_over_current_recurrence_counter <= 0)
    {
        dcdc_input_over_current_recurrence_counter = 0;
        dcdc_input_over_current = false;
    }

    return dcdc_input_over_current;
}

//
//
//
static bool dcdc_input_current_rms_over_current_evaluate(void)
{
    uint16_t i_in_raw_rms = dcdc_cla_to_cpu_mem.i_in_raw_rms.cpu;
    if (i_in_raw_rms > dcdc_input_current_ocp_cutoff_rms_raw)
    {
        ++dcdc_input_current_rms_over_current_counter;
        dcdc_input_current_rms_over_current = true;
    }
    else if (i_in_raw_rms < dcdc_input_current_ocp_qualify_rms_raw)
        dcdc_input_current_rms_over_current = false;

    return dcdc_input_current_rms_over_current;
}
//
//
//
static bool dcdc_ambient_over_termperature_evaluate(void)
{
    if (dcdc_ambient_temperature_x100 > DCDC_AMBIENT_TEMPERATURE_OTP_x100_CUTOFF)
    {
        ++dcdc_ambient_over_temperature_counter;
        dcdc_ambient_over_temperature = true;
    }
    else if (dcdc_ambient_temperature_x100 < DCDC_AMBIENT_TEMPERATURE_OTP_x100_QUALIFY)
        dcdc_ambient_over_temperature = false;

    return dcdc_ambient_over_temperature;
}

//
//
//
static bool dcdc_heatsink_1_over_termperature_evaluate(void)
{
    if (dcdc_heatsink_1_temperature_x100 > DCDC_HEATSINK_TEMPERATURE_OTP_x100_CUTOFF)
    {
        ++dcdc_heatsink_1_over_temperature_counter;
        dcdc_heatsink_1_over_temperature = true;
    }
    else if (dcdc_heatsink_1_temperature_x100 < DCDC_HEATSINK_TEMPERATURE_OTP_x100_QUALIFY)
        dcdc_heatsink_1_over_temperature = false;

    return dcdc_heatsink_1_over_temperature;
}

//
//
//
static bool dcdc_heatsink_2_over_termperature_evaluate(void)
{
    if (dcdc_heatsink_2_temperature_x100 > DCDC_HEATSINK_TEMPERATURE_OTP_x100_CUTOFF)
    {
        ++dcdc_heatsink_2_over_temperature_counter;
        dcdc_heatsink_2_over_temperature = true;
    }
    else if (dcdc_heatsink_2_temperature_x100 < DCDC_HEATSINK_TEMPERATURE_OTP_x100_QUALIFY)
        dcdc_heatsink_2_over_temperature = false;

    return dcdc_heatsink_2_over_temperature;
}

//
//
//
static bool dcdc_llc_primary_heatsink_over_termperature_evaluate(void)

{
    if (dcdc_llc_primary_heatsink_temperature_x100 > DCDC_HEATSINK_TEMPERATURE_OTP_x100_CUTOFF)
    {
        ++dcdc_llc_primary_heatsink_over_temperature_counter;
        dcdc_llc_primary_heatsink_over_temperature = true;
    }
    else if (dcdc_llc_primary_heatsink_temperature_x100 < DCDC_HEATSINK_TEMPERATURE_OTP_x100_QUALIFY)
        dcdc_llc_primary_heatsink_over_temperature = false;

    return dcdc_llc_primary_heatsink_over_temperature;
}

//
//
//
static bool dcdc_input_frequency_out_of_range_evaluate(void)
{
    if ((dcdc_cpu_to_cla_mem.sync_tick_total_per_period.cpu > DCDC_LINE_CYCLE_PERIOD_IN_TICKS_HIGH_CUTOFF)
            || (dcdc_cpu_to_cla_mem.sync_tick_total_per_period.cpu < DCDC_LINE_CYCLE_PERIOD_IN_TICKS_LOW_CUTOFF))
    {
        ++dcdc_input_frequency_out_of_range_counter;
        dcdc_input_frequency_out_of_range = true;
    }
    else if ((dcdc_cpu_to_cla_mem.sync_tick_total_per_period.cpu > DCDC_LINE_CYCLE_PERIOD_IN_TICKS_LOW_QUALIFY)
            && (dcdc_cpu_to_cla_mem.sync_tick_total_per_period.cpu < DCDC_LINE_CYCLE_PERIOD_IN_TICKS_HIGH_QUALIFY))

    {
        dcdc_input_frequency_out_of_range = false;
    }

    return dcdc_input_frequency_out_of_range;
}

//
//
//
static bool dcdc_input_voltage_rms_under_voltage_evaluate(void)
{
    uint16_t v_in_raw_rms = dcdc_cla_to_cpu_mem.v_in_raw_rms.cpu;
    if (v_in_raw_rms < dcdc_input_voltage_uvp_cutoff_rms_raw)
    {
        ++dcdc_input_voltage_rms_under_voltage_counter;
        dcdc_input_voltage_rms_under_voltage = true;
    }
    else if (v_in_raw_rms > dcdc_input_voltage_uvp_qualify_rms_raw)
        dcdc_input_voltage_rms_under_voltage = false;

    return dcdc_input_voltage_rms_under_voltage;
}

//
//
//
static bool dcdc_input_voltage_rms_over_voltage_evaluate(void)
{
    uint16_t v_in_raw_rms = dcdc_cla_to_cpu_mem.v_in_raw_rms.cpu;
    if (v_in_raw_rms > dcdc_input_voltage_ovp_cutoff_rms_raw)
    {
        ++dcdc_input_voltage_rms_over_voltage_counter;
        dcdc_input_voltage_rms_over_voltage = true;
    }
    else if (v_in_raw_rms < dcdc_input_voltage_ovp_qualify_rms_raw)
        dcdc_input_voltage_rms_over_voltage = false;

    return dcdc_input_voltage_rms_over_voltage;
}

//
//
//
static void dcdc_faults_service(void)
{
    bool non_critical_faults_active = false;
    bool critical_faults_active = false;

    dcdc_bus_under_voltage_evaluate();

    non_critical_faults_active |= dcdc_bus_over_voltage_evaluate();
    non_critical_faults_active |= dcdc_input_over_current_evaluate();
    non_critical_faults_active |= dcdc_input_current_rms_over_current_evaluate();
    non_critical_faults_active |= dcdc_ambient_over_termperature_evaluate();
    non_critical_faults_active |= dcdc_heatsink_1_over_termperature_evaluate();
    non_critical_faults_active |= dcdc_heatsink_2_over_termperature_evaluate();
    non_critical_faults_active |= dcdc_llc_primary_heatsink_over_termperature_evaluate();
    dcdc_non_critical_faults_active = non_critical_faults_active;

    critical_faults_active |= dcdc_input_frequency_out_of_range_evaluate();
    critical_faults_active |= dcdc_input_voltage_rms_under_voltage_evaluate();
    critical_faults_active |= dcdc_input_voltage_rms_over_voltage_evaluate();
    dcdc_critical_faults_active = critical_faults_active;
}

//
//
//
static void dcdc_shutdown_state_service(void)
{
    if (dcdc_master_startup_shutdown)
    {
        if (dcdc_open_loop_enable)
            dcdc_cpu_to_cla_mem.dcdc_state.cpu = DCDC_STATE_OPEN_LOOP;
        else
            dcdc_cpu_to_cla_mem.dcdc_state.cpu = DCDC_STATE_WAITING_FOR_INPUT_VOLTAGE_QUALIFICATION;
    }
}

//
//
//
static void dcdc_open_loop_state_service(void)
{
    if (dcdc_open_loop_hf_legs_enable)
        dcdc_pwm_hf_legs_enable_and_unlock();
    else
        dcdc_pwm_hf_legs_disable_and_lock();

    if (dcdc_open_loop_lf_leg_enable)
        dcdc_pwm_lf_leg_enable_and_unlock();
    else
        dcdc_pwm_lf_leg_disable_and_lock();

    if (dcdc_open_loop_inrush_protection_enable)
        dcdc_input_relay_open();
    else
        dcdc_input_relay_close();

    // TODO: Determine which are open loop faults
    if (dcdc_non_critical_faults_active)
        dcdc_cpu_to_cla_mem.dcdc_state.cpu = DCDC_STATE_SHUTDOWN;
}

//
//
//
static void dcdc_waiting_for_critical_faults_to_clear_state_service(void)
{
    if (!dcdc_critical_faults_active)
        dcdc_cpu_to_cla_mem.dcdc_state.cpu = DCDC_STATE_WAITING_FOR_INPUT_VOLTAGE_QUALIFICATION;
}

//
//
//
static void dcdc_waiting_for_input_voltage_qualification_state_service(void)
{
    uint16_t v_bus_raw = dcdc_cla_to_cpu_mem.v_bus_raw.cpu;
    uint16_t v_in_raw_rms = dcdc_cla_to_cpu_mem.v_in_raw_rms.cpu;
    // TODO: If v_bus and v_in don't have equivalent signal conditioning, calibrate

    if ((!dcdc_input_frequency_out_of_range)
            && (!dcdc_input_voltage_rms_under_voltage)
            && (v_bus_raw > (uint16_t)((float32_t)v_in_raw_rms * (SQRT_2 * 0.75f))))
    {
        dcdc_input_relay_close();
        dcdc_inrush_relay_to_close_delay_us =
                CPUTimer_getTimerCount(CPUTIMER0_BASE) + MILLISECONDS_IN_uS;
        dcdc_cpu_to_cla_mem.dcdc_state.cpu =
                DCDC_STATE_WAITING_FOR_INRUSH_RELAY_TO_CLOSE_DELAY_TO_EXPIRE;
    }
}

//
//
//
static void dcdc_waiting_for_inrush_relay_to_close_delay_to_expire_state_service(void)
{
    int32_t cpu_time_us = CPUTimer_getTimerCount(CPUTIMER0_BASE);
    if (IS_CPU_TIME_AFTER(cpu_time_us, dcdc_inrush_relay_to_close_delay_us))
        dcdc_cpu_to_cla_mem.dcdc_state.cpu = DCDC_STATE_WAITING_FOR_NON_CRITICAL_FAULTS_TO_CLEAR;
}

//
//
//
static void dcdc_waiting_for_non_critical_faults_to_clear_state_service(void)
{
    if (!dcdc_non_critical_faults_active)
        dcdc_cpu_to_cla_mem.dcdc_state.cpu = DCDC_STATE_WAITING_FOR_SOFT_START_TO_FINISH;
}

//
//
//
static void dcdc_waiting_for_soft_start_to_finish_state_service(void)
{
    // TODO: Soft start
    dcdc_cpu_to_cla_mem.dcdc_state.cpu = DCDC_STATE_NORMAL;
}

//
//
//
static uint16_t dcdc_input_voltage_threshold_reverse_calibration_calculate(float32_t threshold)
{
    float32_t threshold_default_calibrated = (threshold
            - dcdc_factory_s.input_voltage.offset)
            / dcdc_factory_s.input_voltage.slope;
    float32_t threshold_raw = (threshold_default_calibrated
            - DCDC_CALIBRATION_V_IN_OFFSET_DEFAULT)
            / DCDC_CALIBRATION_V_IN_SLOPE_DEFAULT;

    return (uint16_t)threshold_raw;
}

//
//
//
static uint16_t dcdc_input_current_threshold_reverse_calibration_calculate(float32_t threshold)
{
    float32_t threshold_default_calibrated = (threshold
            - dcdc_factory_s.input_current.offset)
            / dcdc_factory_s.input_current.slope;
    float32_t threshold_raw = (threshold_default_calibrated
            - DCDC_CALIBRATION_I_IN_OFFSET_DEFAULT)
            / DCDC_CALIBRATION_I_IN_SLOPE_DEFAULT;

    return (uint16_t)threshold_raw;
}



//
//
//
void dcdc_slow_init(void)
{
    dcdc_cpu_to_cla_mem.dcdc_state.cpu = DCDC_STATE_SHUTDOWN;
    dcdc_master_startup_shutdown = false;
    dcdc_open_loop_enable = false;
    dcdc_open_loop_hf_legs_enable = false;
    dcdc_open_loop_lf_leg_enable = false;
    dcdc_open_loop_inrush_protection_enable = true;

    dcdc_ambient_temperature_x100 = 0;
    dcdc_heatsink_1_temperature_x100 = 0;
    dcdc_heatsink_2_temperature_x100 = 0;
    dcdc_llc_primary_heatsink_temperature_x100 = 0;

    dcdc_input_voltage_thresholds_reverse_calibration_service();
    dcdc_input_current_thresholds_reverse_calibration_service();

    dcdc_input_over_current_recurrence_counter = 0;

    dcdc_bus_under_voltage = false;
    dcdc_bus_over_voltage = false;
    dcdc_input_over_current = false;
    dcdc_input_current_rms_over_current = false;
    dcdc_ambient_over_temperature = false;
    dcdc_heatsink_1_over_temperature = false;
    dcdc_heatsink_2_over_temperature = false;
    dcdc_llc_primary_heatsink_over_temperature = false;
    dcdc_input_frequency_out_of_range = false;
    dcdc_input_voltage_rms_under_voltage = false;
    dcdc_input_voltage_rms_over_voltage = false;

    dcdc_non_critical_faults_active = false;
    dcdc_critical_faults_active = false;

    dcdc_bus_under_voltage_counter = 0U;
    dcdc_bus_over_voltage_counter = 0U;
    dcdc_input_over_current_counter = 0U;
    dcdc_input_current_rms_over_current_counter = 0U;
    dcdc_ambient_over_temperature_counter = 0U;
    dcdc_heatsink_1_over_temperature_counter = 0U;
    dcdc_heatsink_2_over_temperature_counter = 0U;
    dcdc_llc_primary_heatsink_over_temperature_counter = 0U;
    dcdc_input_frequency_out_of_range_counter = 0U;
    dcdc_input_voltage_rms_under_voltage_counter = 0U;
    dcdc_input_voltage_rms_over_voltage_counter = 0U;

    dcdc_pwm_hf_legs_disable_and_lock();
    dcdc_pwm_lf_leg_disable_and_lock();
    dcdc_input_relay_open();
    dcdc_primary_fault_signal_set();
}

//
//
//
void dcdc_state_machine_service(void)
{
    dcdc_faults_service();
    dcdc_shutdown_command_service();
    dcdc_fault_shutdown_service();
    primary_fault_signal_service();

    switch (dcdc_cpu_to_cla_mem.dcdc_state.cpu)
    {
        case DCDC_STATE_SHUTDOWN:
            dcdc_shutdown_state_service();
            break;
        case DCDC_STATE_OPEN_LOOP:
            dcdc_open_loop_state_service();
            break;
        case DCDC_STATE_WAITING_FOR_CRITICAL_FAULTS_TO_CLEAR:
            dcdc_waiting_for_critical_faults_to_clear_state_service();
            break;
        case DCDC_STATE_WAITING_FOR_INPUT_VOLTAGE_QUALIFICATION:
            dcdc_waiting_for_input_voltage_qualification_state_service();
            break;
        case DCDC_STATE_WAITING_FOR_INRUSH_RELAY_TO_CLOSE_DELAY_TO_EXPIRE:
            dcdc_waiting_for_inrush_relay_to_close_delay_to_expire_state_service();
            break;
        case DCDC_STATE_WAITING_FOR_NON_CRITICAL_FAULTS_TO_CLEAR:
            dcdc_waiting_for_non_critical_faults_to_clear_state_service();
            break;
        case DCDC_STATE_WAITING_FOR_SOFT_START_TO_FINISH:
            dcdc_waiting_for_soft_start_to_finish_state_service();
            break;
        case DCDC_STATE_NORMAL:
            // Do nothing
            break;
        default:
            dcdc_cpu_to_cla_mem.dcdc_state.cpu = DCDC_STATE_SHUTDOWN;
            break;
    }
}

//
//
//
uint32_t dcdc_fan_rpm_get(void)
{
    uint32_t rpm = 0UL;
    uint16_t period;
    period = EQEP_getCapturePeriodLatch(EQEP1_BASE);

    uint16_t status = EQEP_getStatus(EQEP1_BASE);
    if (status & EQEP_STS_UNIT_POS_EVNT)
    {
        EQEP_clearStatus(EQEP1_BASE, EQEP_STS_UNIT_POS_EVNT | EQEP_STS_CAP_OVRFLW_ERROR);
        if (period != 0U)
        {
            uint32_t numerator = (DEVICE_SYSCLK_FREQ / 128U) * 60UL;
            rpm = numerator / period;
        }
    }

    return rpm;
}

//
// Should be serviced at around 1ms rate.
// The temperatures are filtered with an alpha = 1/16
// (tao ~= 15.5ms , cutoff frequency = 10.27Hz)
//
void dcdc_temperatures_filtering_service(void)
{
    int16_t temp_x100;
    temp_x100 = ntc_temperature_x100_get(DCDC_ADC_RESULT_AMBIENT_TEMPERATURE);
    dcdc_ambient_temperature_x100 -= ((dcdc_ambient_temperature_x100 - temp_x100) >> 4);

    temp_x100 = ntc_temperature_x100_get(DCDC_ADC_RESULT_DCDC_HEATSINK_1_TEMPERATURE);
    dcdc_heatsink_1_temperature_x100 -= ((dcdc_heatsink_1_temperature_x100 - temp_x100) >> 4);

    temp_x100 = ntc_temperature_x100_get(DCDC_ADC_RESULT_DCDC_HEATSINK_2_TEMPERATURE);
    dcdc_heatsink_2_temperature_x100 -= ((dcdc_heatsink_2_temperature_x100 - temp_x100) >> 4);

    temp_x100 = ntc_temperature_x100_get(DCDC_ADC_RESULT_LLC_PRIMARY_HEATSINK_TEMPERATURE);
    dcdc_llc_primary_heatsink_temperature_x100 -= ((dcdc_llc_primary_heatsink_temperature_x100 - temp_x100) >> 4);
}

//
//
//
void dcdc_input_voltage_thresholds_reverse_calibration_service(void)
{
    dcdc_input_voltage_uvp_cutoff_rms_raw =
            dcdc_input_voltage_threshold_reverse_calibration_calculate(DCDC_INPUT_VOLTAGE_UVP_CUTOFF_RMS);
    dcdc_input_voltage_uvp_qualify_rms_raw =
            dcdc_input_voltage_threshold_reverse_calibration_calculate(DCDC_INPUT_VOLTAGE_UVP_QUALIFY_RMS);
    dcdc_input_voltage_ovp_cutoff_rms_raw =
            dcdc_input_voltage_threshold_reverse_calibration_calculate(DCDC_INPUT_VOLTAGE_OVP_CUTOFF_RMS);
    dcdc_input_voltage_ovp_qualify_rms_raw =
            dcdc_input_voltage_threshold_reverse_calibration_calculate(DCDC_INPUT_VOLTAGE_OVP_QUALIFY_RMS);
}

//
//
//
void dcdc_input_current_thresholds_reverse_calibration_service(void)
{
    dcdc_input_current_ocp_cutoff_rms_raw =
            dcdc_input_current_threshold_reverse_calibration_calculate(DCDC_INPUT_CURRENT_OCP_CUTOFF_RMS);
    dcdc_input_current_ocp_qualify_rms_raw =
            dcdc_input_current_threshold_reverse_calibration_calculate(DCDC_INPUT_CURRENT_OCP_QUALIFY_RMS);
    dcdc_cpu_to_cla_mem.current_setpoint_max_rms_raw = (float32_t)
            dcdc_input_current_threshold_reverse_calibration_calculate(DCDC_INPUT_CURRENT_SETPOINT_MAX_RMS);
}



int16_t dcdc_ambient_temperature_x100_get(void)
{
    return dcdc_ambient_temperature_x100;
}

int16_t dcdc_heatsink_1_temperature_x100_get(void)
{
    return dcdc_heatsink_1_temperature_x100;
}

int16_t dcdc_heatsink_2_temperature_x100_get(void)
{
    return dcdc_heatsink_2_temperature_x100;
}

int16_t dcdc_llc_primary_heatsink_temperature_x100_get(void)
{
    return dcdc_llc_primary_heatsink_temperature_x100;
}

enum dcdc_states dcdc_state_get(void)
{
    return (enum dcdc_states)dcdc_cpu_to_cla_mem.dcdc_state.cpu;
}

uint16_t dcdc_bus_under_voltage_counter_get(void)
{
    return dcdc_bus_under_voltage_counter;
}

uint16_t dcdc_bus_over_voltage_counter_get(void)
{
    return dcdc_bus_over_voltage_counter;
}

uint16_t dcdc_input_over_current_counter_get(void)
{
    return dcdc_input_over_current_counter;
}

uint16_t dcdc_input_current_rms_over_current_counter_get(void)
{
    return dcdc_input_current_rms_over_current_counter;
}

uint16_t dcdc_ambient_over_temperature_counter_get(void)
{
    return dcdc_ambient_over_temperature_counter;
}

uint16_t dcdc_heatsink_1_over_temperature_counter_get(void)
{
    return dcdc_heatsink_1_over_temperature_counter;
}

uint16_t dcdc_heatsink_2_over_temperature_counter_get(void)
{
    return dcdc_heatsink_2_over_temperature_counter;
}

uint16_t dcdc_llc_primary_heatsink_over_temperature_counter_get(void)
{
    return dcdc_llc_primary_heatsink_over_temperature_counter;
}

uint16_t dcdc_input_frequency_out_of_range_counter_get(void)
{
    return dcdc_input_frequency_out_of_range_counter;
}

uint16_t dcdc_input_voltage_rms_under_voltage_counter_get(void)
{
    return dcdc_input_voltage_rms_under_voltage_counter;
}

uint16_t dcdc_input_voltage_rms_over_voltage_counter_get(void)
{
    return dcdc_input_voltage_rms_over_voltage_counter;
}



void dcdc_bus_under_voltage_counter_reset(void)
{
    dcdc_bus_under_voltage_counter = 0U;
}

void dcdc_bus_over_voltage_counter_reset(void)
{
    dcdc_bus_over_voltage_counter = 0U;
}

void dcdc_input_over_current_counter_reset(void)
{
    dcdc_input_over_current_counter = 0U;
}

void dcdc_input_current_rms_over_current_counter_reset(void)
{
    dcdc_input_current_rms_over_current_counter = 0U;
}

void dcdc_ambient_over_temperature_counter_reset(void)
{
    dcdc_ambient_over_temperature_counter = 0U;
}

void dcdc_heatsink_1_over_temperature_counter_reset(void)
{
    dcdc_heatsink_1_over_temperature_counter = 0U;
}

void dcdc_heatsink_2_over_temperature_counter_reset(void)
{
    dcdc_heatsink_2_over_temperature_counter = 0U;
}

void dcdc_llc_primary_heatsink_over_temperature_counter_reset(void)
{
    dcdc_llc_primary_heatsink_over_temperature_counter = 0U;
}

void dcdc_input_frequency_out_of_range_counter_reset(void)
{
    dcdc_input_frequency_out_of_range_counter = 0U;
}

void dcdc_input_voltage_rms_under_voltage_counter_reset(void)
{
    dcdc_input_voltage_rms_under_voltage_counter = 0U;
}

void dcdc_input_voltage_rms_over_voltage_counter_reset(void)
{
    dcdc_input_voltage_rms_over_voltage_counter = 0U;
}

void dcdc_master_startup_set(void)
{
    dcdc_master_startup_shutdown = true;
}

void dcdc_master_shutdown_set(void)
{
    dcdc_master_startup_shutdown = false;
}

bool dcdc_open_loop_enable_set(bool enable)
{
    if (enable)
    {
        if (dcdc_cpu_to_cla_mem.dcdc_state.cpu == DCDC_STATE_SHUTDOWN)
        {
            dcdc_open_loop_enable = true;
            dcdc_master_startup_shutdown = true;
            return true;
        }
    }
    else
    {
        if (dcdc_cpu_to_cla_mem.dcdc_state.cpu == DCDC_STATE_OPEN_LOOP)
        {
            dcdc_open_loop_enable = false;
            dcdc_master_startup_shutdown = false;
            return true;
        }
    }

    return false;
}

bool dcdc_open_loop_hf_legs_enable_set(bool enable)
{
    if (dcdc_open_loop_enable)
    {
        dcdc_open_loop_hf_legs_enable = enable;
        return true;
    }

    return false;
}

bool dcdc_open_loop_lf_leg_enable_set(bool enable)
{
    if (dcdc_open_loop_enable)
    {
        dcdc_open_loop_lf_leg_enable = enable;
        return true;
    }

    return false;
}

bool dcdc_open_loop_inrush_protection_enable_set(bool enable)
{
    if (dcdc_open_loop_enable)
    {
        dcdc_open_loop_inrush_protection_enable = enable;
        return true;
    }

    return false;
}

bool dcdc_open_loop_hf_legs_duty_set(uint16_t duty)
{
    if (dcdc_open_loop_enable)
    {
        if (duty <= 100U)
        {
            dcdc_cpu_to_cla_mem.open_loop_hf_legs_duty.cpu = (uint16_t)duty;
            return true;
        }
    }
    return false;
}
