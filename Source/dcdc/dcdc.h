///////////////////////////////////////////////////////////////////////////////
//
// Include file
//
///////////////////////////////////////////////////////////////////////////////

#ifndef DCDC_H_
#define DCDC_H_

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "util.h"


#define EPWM1_FREQ_MIN      90000UL
#define EPWM1_PERIOD_MAX    PWM_PERIOD_IN_COUNTS_UP_COUNTER(EPWM1_FREQ_MIN)
#define EPWM1_FREQ_MAX      250000UL
#define EPWM1_PERIOD_MIN    PWM_PERIOD_IN_COUNTS_UP_COUNTER(EPWM1_FREQ_MAX)
#define EPWM1_DEADBAND      10U // 100ns

#define EPWM5_FREQ          200000UL // TODO
#define EPWM5_PERIOD        PWM_PERIOD_IN_COUNTS_UP_COUNTER(EPWM5_FREQ)


#define DCDC_CALIBRATION_V_OUT_SLOPE_DEFAULT      1.0f
#define DCDC_CALIBRATION_V_OUT_OFFSET_DEFAULT     0.0f

#define DCDC_CALIBRATION_I_OUT_SLOPE_DEFAULT      1.0f
#define DCDC_CALIBRATION_I_OUT_OFFSET_DEFAULT     0.0f

#define DCDC_OUTPUT_CURRENT_REVERSE_DEFAULT_CALIBRATION_CALCULATE(i)    (int16_t)(((i) \
        - DCDC_CALIBRATION_I_OUT_OFFSET_DEFAULT) / DCDC_CALIBRATION_I_OUT_SLOPE_DEFAULT)


#define DCDC_OUTPUT_CURRENT_SETPOINT_MAX         50.0f



#define DCDC_ADC_RESULT_OUTPUT_VOLTAGE_1                        ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0)
#define DCDC_ADC_RESULT_OUTPUT_CURRENT                          ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1)
#define DCDC_ADC_RESULT_LLC_SECONDARY_HEATSINK_1_TEMPERATURE    ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2)
#define DCDC_ADC_RESULT_OUTPUT_VOLTAGE_2                        ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0)
#define DCDC_ADC_RESULT_LLC_SECONDARY_HEATSINK_2_TEMPERATURE    ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1)
#define DCDC_ADC_RESULT_HW_REVISION                             ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2)
#define DCDC_ADC_RESULT_OUTPUT_CURRENT_EXTERNAL_SETPOINT        ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0)
#define DCDC_ADC_RESULT_SHELF_ID                                ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1)
#define DCDC_ADC_RESULT_SLOT_ID                                 ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2)


enum dcdc_states
{
    DCDC_STATE_SHUTDOWN,
    DCDC_STATE_OPEN_LOOP,
    DCDC_STATE_WAITING_FOR_CRITICAL_FAULTS_TO_CLEAR,
    DCDC_STATE_WAITING_FOR_FAULTS_FROM_PFC_TO_CLEAR,
    DCDC_STATE_WAITING_FOR_NON_CRITICAL_FAULTS_TO_CLEAR,
    DCDC_STATE_WAITING_FOR_SOFT_START_TO_FINISH,
    DCDC_STATE_NORMAL,
    DCDC_STATE_LIGHT_LOAD_OPERATION,
};


union dcdc_cpu_cla_union_t
{
    uint32_t cla;
    uint16_t cpu;
};


struct dcdc_cla_to_cpu
{
    union dcdc_cpu_cla_union_t i_out_raw_filtered;
};

struct dcdc_cpu_to_cla
{
    union dcdc_cpu_cla_union_t dcdc_state;
    union dcdc_cpu_cla_union_t open_loop_primary_period;

    float32_t current_setpoint_max_raw;
    float32_t voltage_loop_gain;
    float32_t current_loop_gain;
};


struct dcdc_calibration_constants
{
    float32_t slope;
    float32_t offset;
};

// Important note: ALWAYS add new elements at the END
// of this strucutre, never in the middle. Also, never
// delete existing elements.
struct dcdc_factory
{
    uint32_t serial_number;
    struct dcdc_calibration_constants output_voltage;
    struct dcdc_calibration_constants output_current;
};


extern void dcdc_init(void);
extern void dcdc_cla_init(void);
extern void dcdc_isr_init(void);
extern void dcdc_slow_init(void);
extern void dcdc_sdp_init(void);

extern void dcdc_state_machine_service(void);
extern void dcdc_sdp_service(void);

extern void dcdc_temperatures_filtering_service(void);
extern void dcdc_output_voltage_thresholds_reverse_calibration_service(void);
extern void dcdc_output_current_thresholds_reverse_calibration_service(void);

// cla tasks
extern void dcdc_cla_task_1(void);
extern void dcdc_cla_initialization_task(void);


extern int16_t dcdc_llc_secondary_heatsink_1_temperature_x100_get(void);
extern int16_t dcdc_llc_secondary_heatsink_2_temperature_x100_get(void);
extern enum dcdc_states dcdc_state_get(void);
extern uint16_t dcdc_output_over_current_counter_get(void);
extern uint16_t dcdc_llc_secondary_heatsink_1_over_temperature_counter_get(void);
extern uint16_t dcdc_llc_secondary_heatsink_2_over_temperature_counter_get(void);
extern uint16_t dcdc_output_voltage_over_voltage_counter_get(void);

extern void dcdc_output_over_current_counter_reset(void);
extern void dcdc_llc_secondary_heatsink_1_over_temperature_counter_reset(void);
extern void dcdc_llc_secondary_heatsink_2_over_temperature_counter_reset(void);
extern void dcdc_output_voltage_over_voltage_counter_reset(void);
extern void dcdc_master_startup_set(void);
extern void dcdc_master_shutdown_set(void);
extern bool dcdc_open_loop_enable_set(bool enable);
extern bool dcdc_open_loop_primary_enable_set(bool enable);
extern bool dcdc_open_loop_sr_enable_set(bool enable);
extern bool dcdc_open_loop_active_dummy_load_enable_set(bool enable);
extern bool dcdc_open_loop_primary_period_set(uint16_t period);


extern volatile struct dcdc_cla_to_cpu dcdc_cla_to_cpu_mem;
extern volatile struct dcdc_cpu_to_cla dcdc_cpu_to_cla_mem;

extern struct dcdc_factory dcdc_factory_s;

#endif
