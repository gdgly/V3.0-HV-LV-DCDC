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



#define DCDC_HEATSINK_TEMPERATURE_OTP_x100_CUTOFF    (100 * 100)
#define DCDC_HEATSINK_TEMPERATURE_OTP_x100_QUALIFY   ( 90 * 100)

#define DCDC_OUTPUT_VOLTAGE_OVP_CUTOFF        60.0f
#define DCDC_OUTPUT_VOLTAGE_OVP_QUALIFY       58.0f

#define DCDC_OUTPUT_CURRENT_OCP_CUTOFF         60.0f
#define DCDC_OUTPUT_CURRENT_OCP_QUALIFY        55.0f

#define DCDC_OUTPUT_OCP_RECURRENCE_THRESHOLD    5


static bool dcdc_master_startup_shutdown;
static bool dcdc_open_loop_enable;
static bool dcdc_open_loop_primary_enable;
static bool dcdc_open_loop_sr_enable;
static bool dcdc_open_loop_active_dummy_load_enable;
static bool dcdc_open_loop_oring_fet_enable;

static int16_t dcdc_llc_secondary_heatsink_1_temperature_x100;
static int16_t dcdc_llc_secondary_heatsink_2_temperature_x100;

static uint16_t dcdc_output_voltage_ovp_cutoff_raw;
static uint16_t dcdc_output_voltage_ovp_qualify_raw;
static uint16_t dcdc_output_current_ocp_cutoff_raw;
static uint16_t dcdc_output_current_ocp_qualify_raw;

static int16_t dcdc_output_over_current_recurrence_counter;

static bool dcdc_output_over_current;
static bool dcdc_output_current_sw_over_current;
static bool dcdc_llc_secondary_heatsink_1_over_temperature;
static bool dcdc_llc_secondary_heatsink_2_over_temperature;
static bool dcdc_output_voltage_over_voltage;

static bool dcdc_non_critical_faults_active;
static bool dcdc_critical_faults_active;

static uint16_t dcdc_output_over_current_counter;
static uint16_t dcdc_output_current_sw_over_current_counter;
static uint16_t dcdc_llc_secondary_heatsink_1_over_temperature_counter;
static uint16_t dcdc_llc_secondary_heatsink_2_over_temperature_counter;
static uint16_t dcdc_output_voltage_over_voltage_counter;

static int32_t dcdc_inrush_relay_to_close_delay_us;



//
//
//
static void dcdc_pwm_primary_enable_and_unlock(void)
{
    EPWM_clearTripZoneFlag(EPWM1_BASE, EPWM_TZ_FLAG_DCAEVT1);
}

//
//
//
static void dcdc_pwm_primary_disable_and_lock(void)
{
    EPWM_forceTripZoneEvent(EPWM1_BASE, EPWM_TZ_FORCE_EVENT_DCAEVT1);
}

//
//
//
static void dcdc_pwm_sr_enable(void)
{
    GPIO_writePin(13, 1);
}

//
//
//
static void dcdc_pwm_sr_disable(void)
{
    GPIO_writePin(13, 0);
}

//
//
//
static void dcdc_active_dummy_load_enable(void)
{
    GPIO_writePin(31, 1);
}

//
//
//
static void dcdc_active_dummy_load_disable(void)
{
    GPIO_writePin(31, 0);
}

//
//
//
static void dcdc_oring_fet_enable(void)
{
    GPIO_writePin(12, 1);
}

//
//
//
static void dcdc_oring_fet_disable(void)
{
    GPIO_writePin(12, 0);
}

//
//
//
static uint32_t dcdc_primary_fault_signal_status_get(void)
{
    return GPIO_readPin(17);
}

//
//
//
static void dcdc_shutdown_command_service(void)
{
    if (!dcdc_master_startup_shutdown)
    {
        dcdc_pwm_primary_disable_and_lock();
        dcdc_pwm_sr_disable();
        dcdc_active_dummy_load_disable();
        dcdc_oring_fet_disable();
        dcdc_open_loop_enable = false;
        dcdc_open_loop_primary_enable = false;
        dcdc_open_loop_sr_enable = false;
        dcdc_open_loop_active_dummy_load_enable = false;
        dcdc_open_loop_oring_fet_enable = false;
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
static bool dcdc_primary_fault_evaluate(void)
{
    uint32_t status = dcdc_primary_fault_signal_status_get();
    return (status != 0UL);
}

//
//
//
static bool dcdc_output_over_current_evaluate(void)
{
    if (EPWM_getTripZoneFlagStatus(EPWM1_BASE) & EPWM_TZ_FLAG_CBC)
    {
        EPWM_clearTripZoneFlag(EPWM1_BASE, EPWM_TZ_FLAG_CBC);
        dcdc_output_over_current_recurrence_counter++;
    }
    else
        dcdc_output_over_current_recurrence_counter--;

    if (dcdc_output_over_current_recurrence_counter >= DCDC_OUTPUT_OCP_RECURRENCE_THRESHOLD)
    {
        dcdc_output_over_current_recurrence_counter = DCDC_OUTPUT_OCP_RECURRENCE_THRESHOLD;
        ++dcdc_output_over_current_counter;
        dcdc_output_over_current = true;
    }
    else if (dcdc_output_over_current_recurrence_counter <= 0)
    {
        dcdc_output_over_current_recurrence_counter = 0;
        dcdc_output_over_current = false;
    }

    return dcdc_output_over_current;
}

//
//
//
static bool dcdc_output_current_sw_over_current_evaluate(void)
{
    uint16_t i_out_raw = DCDC_ADC_RESULT_OUTPUT_CURRENT;
    if (i_out_raw > dcdc_output_current_ocp_cutoff_raw)
    {
        ++dcdc_output_current_sw_over_current_counter;
        dcdc_output_current_sw_over_current = true;
    }
    else if (i_out_raw < dcdc_output_current_ocp_qualify_raw)
        dcdc_output_current_sw_over_current = false;

    return dcdc_output_current_sw_over_current;
}

//
//
//
static bool dcdc_llc_secondary_heatsink_1_over_temperature_evaluate(void)
{
    if (dcdc_llc_secondary_heatsink_1_temperature_x100 > DCDC_HEATSINK_TEMPERATURE_OTP_x100_CUTOFF)
    {
        ++dcdc_llc_secondary_heatsink_1_over_temperature_counter;
        dcdc_llc_secondary_heatsink_1_over_temperature = true;
    }
    else if (dcdc_llc_secondary_heatsink_1_temperature_x100 < DCDC_HEATSINK_TEMPERATURE_OTP_x100_QUALIFY)
        dcdc_llc_secondary_heatsink_1_over_temperature = false;

    return dcdc_llc_secondary_heatsink_1_over_temperature;
}

//
//
//
static bool dcdc_llc_secondary_heatsink_2_over_temperature_evaluate(void)
{
    if (dcdc_llc_secondary_heatsink_2_temperature_x100 > DCDC_HEATSINK_TEMPERATURE_OTP_x100_CUTOFF)
    {
        ++dcdc_llc_secondary_heatsink_2_over_temperature_counter;
        dcdc_llc_secondary_heatsink_2_over_temperature = true;
    }
    else if (dcdc_llc_secondary_heatsink_2_temperature_x100 < DCDC_HEATSINK_TEMPERATURE_OTP_x100_QUALIFY)
        dcdc_llc_secondary_heatsink_2_over_temperature = false;

    return dcdc_llc_secondary_heatsink_2_over_temperature;
}

//
//
//
static bool dcdc_output_voltage_over_voltage_evaluate(void)
{
    uint16_t v_out_1_raw = DCDC_ADC_RESULT_OUTPUT_VOLTAGE_1;
    if (v_out_1_raw > dcdc_output_voltage_ovp_cutoff_raw)
    {
        ++dcdc_output_voltage_over_voltage_counter;
        dcdc_output_voltage_over_voltage = true;
    }
    else if (v_out_1_raw < dcdc_output_voltage_ovp_qualify_raw)
        dcdc_output_voltage_over_voltage = false;

    return dcdc_output_voltage_over_voltage;
}

//
//
//
static void dcdc_faults_service(void)
{
    bool non_critical_faults_active = false;
    bool critical_faults_active = false;

    non_critical_faults_active |= dcdc_output_over_current_evaluate();
    non_critical_faults_active |= dcdc_output_current_sw_over_current_evaluate();
    non_critical_faults_active |= dcdc_llc_secondary_heatsink_1_over_temperature_evaluate();
    non_critical_faults_active |= dcdc_llc_secondary_heatsink_2_over_temperature_evaluate();
    dcdc_non_critical_faults_active = non_critical_faults_active;

    critical_faults_active |= dcdc_primary_fault_evaluate();
    critical_faults_active |= dcdc_output_voltage_over_voltage_evaluate();
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
    if (dcdc_open_loop_primary_enable)
        dcdc_pwm_primary_enable_and_unlock();
    else
        dcdc_pwm_primary_disable_and_lock();

    if (dcdc_open_loop_sr_enable)
        dcdc_pwm_sr_enable();
    else
        dcdc_pwm_sr_disable();

    if (dcdc_open_loop_active_dummy_load_enable)
        dcdc_active_dummy_load_enable();
    else
        dcdc_active_dummy_load_disable();

    if (dcdc_open_loop_oring_fet_enable)
        dcdc_oring_fet_enable();
    else
        dcdc_oring_fet_disable();


    // TODO: Determine which are open loop faults
    if (dcdc_non_critical_faults_active)
        dcdc_master_startup_shutdown = false;
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
    // TODO:
    bool input_voltage_qualified = true;
    if (input_voltage_qualified)
    {
        dcdc_active_dummy_load_disable();
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
static uint16_t dcdc_output_voltage_threshold_reverse_calibration_calculate(float32_t threshold)
{
    float32_t threshold_default_calibrated = (threshold
            - dcdc_factory_s.output_voltage.offset)
            / dcdc_factory_s.output_voltage.slope;
    float32_t threshold_raw = (threshold_default_calibrated
            - DCDC_CALIBRATION_V_OUT_OFFSET_DEFAULT)
            / DCDC_CALIBRATION_V_OUT_SLOPE_DEFAULT;

    return (uint16_t)threshold_raw;
}

//
//
//
static uint16_t dcdc_output_current_threshold_reverse_calibration_calculate(float32_t threshold)
{
    float32_t threshold_default_calibrated = (threshold
            - dcdc_factory_s.output_current.offset)
            / dcdc_factory_s.output_current.slope;
    float32_t threshold_raw = (threshold_default_calibrated
            - DCDC_CALIBRATION_I_OUT_OFFSET_DEFAULT)
            / DCDC_CALIBRATION_I_OUT_SLOPE_DEFAULT;

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
    dcdc_open_loop_primary_enable = false;
    dcdc_open_loop_sr_enable = false;
    dcdc_open_loop_active_dummy_load_enable = false;
    dcdc_open_loop_oring_fet_enable = false;

    dcdc_llc_secondary_heatsink_1_temperature_x100 = 0;
    dcdc_llc_secondary_heatsink_2_temperature_x100 = 0;

    dcdc_output_voltage_thresholds_reverse_calibration_service();
    dcdc_output_current_thresholds_reverse_calibration_service();

    dcdc_output_over_current_recurrence_counter = 0;

    dcdc_output_over_current = false;
    dcdc_output_current_sw_over_current = false;
    dcdc_llc_secondary_heatsink_1_over_temperature = false;
    dcdc_llc_secondary_heatsink_2_over_temperature = false;
    dcdc_output_voltage_over_voltage = false;

    dcdc_non_critical_faults_active = false;
    dcdc_critical_faults_active = false;

    dcdc_output_over_current_counter = 0U;
    dcdc_output_current_sw_over_current_counter = 0U;
    dcdc_llc_secondary_heatsink_1_over_temperature_counter = 0U;
    dcdc_llc_secondary_heatsink_2_over_temperature_counter = 0U;
    dcdc_output_voltage_over_voltage_counter = 0U;

    dcdc_pwm_primary_disable_and_lock();
    dcdc_pwm_sr_disable();
    dcdc_active_dummy_load_disable();
    dcdc_oring_fet_disable();
    dcdc_primary_fault_signal_status_get();
}

//
//
//
void dcdc_state_machine_service(void)
{
    dcdc_faults_service();
    dcdc_shutdown_command_service();
    dcdc_fault_shutdown_service();

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
            dcdc_master_startup_shutdown = false;
            break;
    }
}

//
// Should be serviced at around 1ms rate.
// The temperatures are filtered with an alpha = 1/16
// (tao ~= 15.5ms , cutoff frequency = 10.27Hz)
//
void dcdc_temperatures_filtering_service(void)
{
    int16_t temp_x100;
    temp_x100 = ntc_temperature_x100_get(DCDC_ADC_RESULT_LLC_SECONDARY_HEATSINK_1_TEMPERATURE);
    dcdc_llc_secondary_heatsink_1_temperature_x100 -= ((dcdc_llc_secondary_heatsink_1_temperature_x100 - temp_x100) >> 4);

    temp_x100 = ntc_temperature_x100_get(DCDC_ADC_RESULT_LLC_SECONDARY_HEATSINK_2_TEMPERATURE);
    dcdc_llc_secondary_heatsink_2_temperature_x100 -= ((dcdc_llc_secondary_heatsink_2_temperature_x100 - temp_x100) >> 4);
}

//
//
//
void dcdc_output_voltage_thresholds_reverse_calibration_service(void)
{
    dcdc_output_voltage_ovp_cutoff_raw =
            dcdc_output_voltage_threshold_reverse_calibration_calculate(DCDC_OUTPUT_VOLTAGE_OVP_CUTOFF);
    dcdc_output_voltage_ovp_qualify_raw =
            dcdc_output_voltage_threshold_reverse_calibration_calculate(DCDC_OUTPUT_VOLTAGE_OVP_QUALIFY);
}

//
//
//
void dcdc_output_current_thresholds_reverse_calibration_service(void)
{
    dcdc_output_current_ocp_cutoff_raw =
            dcdc_output_current_threshold_reverse_calibration_calculate(DCDC_OUTPUT_CURRENT_OCP_CUTOFF);
    dcdc_output_current_ocp_qualify_raw =
            dcdc_output_current_threshold_reverse_calibration_calculate(DCDC_OUTPUT_CURRENT_OCP_QUALIFY);
    dcdc_cpu_to_cla_mem.current_setpoint_max_raw = (float32_t)
            dcdc_output_current_threshold_reverse_calibration_calculate(DCDC_OUTPUT_CURRENT_SETPOINT_MAX);
}



int16_t dcdc_llc_secondary_heatsink_1_temperature_x100_get(void)
{
    return dcdc_llc_secondary_heatsink_1_temperature_x100;
}

int16_t dcdc_llc_secondary_heatsink_2_temperature_x100_get(void)
{
    return dcdc_llc_secondary_heatsink_2_temperature_x100;
}

enum dcdc_states dcdc_state_get(void)
{
    return (enum dcdc_states)dcdc_cpu_to_cla_mem.dcdc_state.cpu;
}

uint16_t dcdc_output_over_current_counter_get(void)
{
    return dcdc_output_over_current_counter;
}

uint16_t dcdc_output_current_sw_over_current_counter_get(void)
{
    return dcdc_output_current_sw_over_current_counter;
}

uint16_t dcdc_llc_secondary_heatsink_1_over_temperature_counter_get(void)
{
    return dcdc_llc_secondary_heatsink_1_over_temperature_counter;
}

uint16_t dcdc_llc_secondary_heatsink_2_over_temperature_counter_get(void)
{
    return dcdc_llc_secondary_heatsink_2_over_temperature_counter;
}

uint16_t dcdc_output_voltage_over_voltage_counter_get(void)
{
    return dcdc_output_voltage_over_voltage_counter;
}



void dcdc_output_over_current_counter_reset(void)
{
    dcdc_output_over_current_counter = 0U;
}

void dcdc_output_current_sw_over_current_counter_reset(void)
{
    dcdc_output_current_sw_over_current_counter = 0U;
}

void dcdc_llc_secondary_heatsink_1_over_temperature_counter_reset(void)
{
    dcdc_llc_secondary_heatsink_1_over_temperature_counter = 0U;
}

void dcdc_llc_secondary_heatsink_2_over_temperature_counter_reset(void)
{
    dcdc_llc_secondary_heatsink_2_over_temperature_counter = 0U;
}

void dcdc_output_voltage_over_voltage_counter_reset(void)
{
    dcdc_output_voltage_over_voltage_counter = 0U;
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

bool dcdc_open_loop_primary_enable_set(bool enable)
{
    if (dcdc_open_loop_enable)
    {
        dcdc_open_loop_primary_enable = enable;
        return true;
    }

    return false;
}

bool dcdc_open_loop_sr_enable_set(bool enable)
{
    if (dcdc_open_loop_enable)
    {
        dcdc_open_loop_sr_enable = enable;
        return true;
    }

    return false;
}

bool dcdc_open_loop_active_dummy_load_enable_set(bool enable)
{
    if (dcdc_open_loop_enable)
    {
        dcdc_open_loop_active_dummy_load_enable = enable;
        return true;
    }

    return false;
}

bool dcdc_open_loop_oring_fet_enable_set(bool enable)
{
    if (dcdc_open_loop_enable)
    {
        dcdc_open_loop_oring_fet_enable = enable;
        return true;
    }

    return false;
}

bool dcdc_open_loop_primary_period_set(uint16_t period)
{
    if (dcdc_open_loop_enable)
    {
        if ((period <= EPWM1_PERIOD_MAX)
                && (period >= EPWM1_PERIOD_MIN))
        {
            dcdc_cpu_to_cla_mem.open_loop_primary_period.cpu = (uint16_t)period;
            return true;
        }
    }
    return false;
}
