///////////////////////////////////////////////////////////////////////////////
//
// SDP file
//
///////////////////////////////////////////////////////////////////////////////


//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "dcdc.h"
#include "sdp/sdp.h"
#include "sci.h"
#include "hwid/hwid.h"
#include "slot_shelf_id/slot_shelf_id.h"
#include "pers/pers.h"
#include "version.h"


#define SDP_FACTORY_MODE_VALUE_ENABLED  0x12345678UL

#define DCDC_OUTPUT_VOLTAGE_SLOPE_MIN     0.9f
#define DCDC_OUTPUT_VOLTAGE_SLOPE_MAX     1.1f
#define DCDC_OUTPUT_VOLTAGE_OFFSET_MIN   -10.0f
#define DCDC_OUTPUT_VOLTAGE_OFFSET_MAX    10.0f

#define DCDC_OUTPUT_CURRENT_SLOPE_MIN     0.9f
#define DCDC_OUTPUT_CURRENT_SLOPE_MAX     1.1f
#define DCDC_OUTPUT_CURRENT_OFFSET_MIN   -5.0f
#define DCDC_OUTPUT_CURRENT_OFFSET_MAX    5.0f

#define DCDC_OUTPUT_VOLTAGE_SETPOINT_MIN_x100   (48U * 100U)
#define DCDC_OUTPUT_VOLTAGE_SETPOINT_MAX_x100   (60U * 100U)



static bool sdp_factory_mode_enabled;


// Function prototypes
static uint32_t sdp_fw_build_get(void);
static uint32_t sdp_hw_id_get(void);
static uint32_t sdp_serial_number_get(void);
static uint32_t sdp_shelf_id_get(void);
static uint32_t sdp_slot_id_get(void);
static uint32_t sdp_output_voltage_1_x100_get(void);
static uint32_t sdp_output_current_x100_get(void);
static uint32_t sdp_dcdc_llc_secondary_heatsink_1_temperature_x100_get(void);
static uint32_t sdp_dcdc_llc_secondary_heatsink_2_temperature_x100_get(void);
static uint32_t sdp_output_over_current_counter_get(void);
static uint32_t sdp_llc_secondary_heatsink_1_over_temperature_counter_get(void);
static uint32_t sdp_llc_secondary_heatsink_2_over_temperature_counter_get(void);
static uint32_t sdp_output_over_voltage_counter_get(void);
static uint32_t sdp_dcdc_state_get(void);
static uint32_t sdp_output_voltage_setpoint_x100_get(void);
static uint32_t sdp_voltage_loop_gain_q16_get(void);
static uint32_t sdp_current_loop_gain_q16_get(void);
static uint32_t sdp_output_voltage_slope_q16_get(void);
static uint32_t sdp_output_voltage_offset_q16_get(void);
static uint32_t sdp_output_current_slope_q16_get(void);
static uint32_t sdp_output_current_offset_q16_get(void);

static bool sdp_serial_number_set(uint32_t value);
static bool sdp_output_over_current_counter_set(uint32_t value);
static bool sdp_llc_secondary_heatsink_1_over_temperature_counter_set(uint32_t value);
static bool sdp_llc_secondary_heatsink_2_over_temperature_counter_set(uint32_t value);
static bool sdp_output_over_voltage_counter_set(uint32_t value);
static bool sdp_master_startup_set(uint32_t value);
static bool sdp_master_shutdown_set(uint32_t value);
static bool sdp_open_loop_enable_set(uint32_t value);
static bool sdp_open_loop_primary_enable_set(uint32_t value);
static bool sdp_open_loop_active_dummy_load_enable_set(uint32_t value);
static bool sdp_open_loop_primary_frequency_set(uint32_t value);
static bool sdp_output_voltage_setpoint_x100_set(uint32_t value);
static bool sdp_voltage_loop_gain_q16_set(uint32_t value);
static bool sdp_current_loop_gain_q16_set(uint32_t value);
static bool sdp_enter_factory_mode_set(uint32_t value);
static bool sdp_output_voltage_slope_q16_set(uint32_t value);
static bool sdp_output_voltage_offset_q16_set(uint32_t value);
static bool sdp_output_current_slope_q16_set(uint32_t value);
static bool sdp_output_current_offset_q16_set(uint32_t value);

static const struct sdp_get sdp_get_array[] =
{
    {  0U, &sdp_fw_build_get},
    {  1U, &sdp_hw_id_get},
    {  2U, &sdp_serial_number_get},
    {  3U, &sdp_shelf_id_get},
    {  4U, &sdp_slot_id_get},
    { 11U, &sdp_output_voltage_1_x100_get},
    { 12U, &sdp_output_current_x100_get},
    { 14U, &sdp_dcdc_llc_secondary_heatsink_1_temperature_x100_get},
    { 15U, &sdp_dcdc_llc_secondary_heatsink_2_temperature_x100_get},
    { 22U, &sdp_output_over_current_counter_get},
    { 25U, &sdp_llc_secondary_heatsink_1_over_temperature_counter_get},
    { 26U, &sdp_llc_secondary_heatsink_2_over_temperature_counter_get},
    { 30U, &sdp_output_over_voltage_counter_get},
    { 40U, &sdp_dcdc_state_get},
    { 50U, &sdp_output_voltage_setpoint_x100_get},
    { 51U, &sdp_voltage_loop_gain_q16_get},
    { 52U, &sdp_current_loop_gain_q16_get},
    {101U, &sdp_output_voltage_slope_q16_get},
    {102U, &sdp_output_voltage_offset_q16_get},
    {103U, &sdp_output_current_slope_q16_get},
    {104U, &sdp_output_current_offset_q16_get},
};

static const struct sdp_set sdp_set_array[] =
{
    {  2U, &sdp_serial_number_set},
    { 22U, &sdp_output_over_current_counter_set},
    { 25U, &sdp_llc_secondary_heatsink_1_over_temperature_counter_set},
    { 26U, &sdp_llc_secondary_heatsink_2_over_temperature_counter_set},
    { 30U, &sdp_output_over_voltage_counter_set},
    { 41U, &sdp_master_startup_set},
    { 42U, &sdp_master_shutdown_set},
    { 43U, &sdp_open_loop_enable_set},
    { 44U, &sdp_open_loop_primary_enable_set},
    { 46U, &sdp_open_loop_active_dummy_load_enable_set},
    { 47U, &sdp_open_loop_primary_frequency_set},
    { 50U, &sdp_output_voltage_setpoint_x100_set},
    { 51U, &sdp_voltage_loop_gain_q16_set},
    { 52U, &sdp_current_loop_gain_q16_set},
    {100U, &sdp_enter_factory_mode_set},
    {101U, &sdp_output_voltage_slope_q16_set},
    {102U, &sdp_output_voltage_offset_q16_set},
    {103U, &sdp_output_current_slope_q16_set},
    {104U, &sdp_output_current_offset_q16_set},
};



//
//
//
static uint32_t sdp_fw_build_get(void)
{
    return BUILD;
}

//
//
//
static uint32_t sdp_hw_id_get(void)
{
    return hwid_get(DCDC_ADC_RESULT_HW_REVISION);
}

//
//
//
static uint32_t sdp_serial_number_get(void)
{
    return dcdc_factory_s.serial_number;
}

//
//
//
static uint32_t sdp_shelf_id_get(void)
{
    return slot_shelf_id_get(DCDC_ADC_RESULT_SHELF_ID);
}

//
//
//
static uint32_t sdp_slot_id_get(void)
{
    return slot_shelf_id_get(DCDC_ADC_RESULT_SLOT_ID);
}

//
//
//
static uint32_t sdp_output_voltage_1_x100_get(void)
{
    float32_t v_out_1_raw = (float32_t)DCDC_ADC_RESULT_OUTPUT_VOLTAGE_1;
    float32_t v_out_1_default_calibrated =
            (v_out_1_raw * DCDC_CALIBRATION_V_OUT_SLOPE_DEFAULT)
            + DCDC_CALIBRATION_V_OUT_OFFSET_DEFAULT;
    float32_t v_out_1_calibrated =
            (v_out_1_default_calibrated * dcdc_factory_s.output_voltage.slope)
            + dcdc_factory_s.output_voltage.offset;
    return (uint32_t)(v_out_1_calibrated * 100.0f);
}

//
//
//
static uint32_t sdp_output_current_x100_get(void)
{
    float32_t i_out_raw = (float32_t)DCDC_ADC_RESULT_OUTPUT_CURRENT;
    float32_t i_out_default_calibrated =
            (i_out_raw * DCDC_CALIBRATION_I_OUT_SLOPE_DEFAULT)
            + DCDC_CALIBRATION_I_OUT_OFFSET_DEFAULT;
    float32_t i_out_calibrated =
            (i_out_default_calibrated * dcdc_factory_s.output_current.slope)
            + dcdc_factory_s.output_current.offset;
    return (uint32_t)(i_out_calibrated * 100.0f);
}

//
//
//
static uint32_t sdp_dcdc_llc_secondary_heatsink_1_temperature_x100_get(void)
{
    return dcdc_llc_secondary_heatsink_1_temperature_x100_get();
}

//
//
//
static uint32_t sdp_dcdc_llc_secondary_heatsink_2_temperature_x100_get(void)
{
    return dcdc_llc_secondary_heatsink_2_temperature_x100_get();
}

//
//
//
static uint32_t sdp_output_over_current_counter_get(void)
{
    return dcdc_output_over_current_counter_get();
}

//
//
//
static uint32_t sdp_llc_secondary_heatsink_1_over_temperature_counter_get(void)
{
    return dcdc_llc_secondary_heatsink_1_over_temperature_counter_get();
}

//
//
//
static uint32_t sdp_llc_secondary_heatsink_2_over_temperature_counter_get(void)
{
    return dcdc_llc_secondary_heatsink_2_over_temperature_counter_get();
}

//
//
//
static uint32_t sdp_output_over_voltage_counter_get(void)
{
    return dcdc_output_voltage_over_voltage_counter_get();
}

//
//
//
static uint32_t sdp_dcdc_state_get(void)
{
    return dcdc_state_get();
}

//
//
//
static uint32_t sdp_output_voltage_setpoint_x100_get(void)
{
    return (uint32_t)(dcdc_configuration_s.output_voltage_setpoint * 100.0f);
}

//
//
//
static uint32_t sdp_voltage_loop_gain_q16_get(void)
{
    return FLOAT_TO_Q16(dcdc_cpu_to_cla_mem.voltage_loop_gain);
}

//
//
//
static uint32_t sdp_current_loop_gain_q16_get(void)
{
    return FLOAT_TO_Q16(dcdc_cpu_to_cla_mem.current_loop_gain);
}

//
//
//
static uint32_t sdp_output_voltage_slope_q16_get(void)
{
    return FLOAT_TO_Q16(dcdc_factory_s.output_voltage.slope);
}

//
//
//
static uint32_t sdp_output_voltage_offset_q16_get(void)
{
    return FLOAT_TO_Q16(dcdc_factory_s.output_voltage.offset);
}

//
//
//
static uint32_t sdp_output_current_slope_q16_get(void)
{
    return FLOAT_TO_Q16(dcdc_factory_s.output_current.slope);
}

//
//
//
static uint32_t sdp_output_current_offset_q16_get(void)
{
    return FLOAT_TO_Q16(dcdc_factory_s.output_current.offset);
}


//
//
//
static bool sdp_serial_number_set(uint32_t value)
{
    if (!sdp_factory_mode_enabled)
        return false;

    dcdc_factory_s.serial_number = value;
    return pers_factory_store_initiate();
}

//
//
//
static bool sdp_output_over_current_counter_set(uint32_t value)
{
    dcdc_output_over_current_counter_reset();
    return true;
}

//
//
//
static bool sdp_llc_secondary_heatsink_1_over_temperature_counter_set(uint32_t value)
{
    dcdc_llc_secondary_heatsink_1_over_temperature_counter_reset();
    return true;
}

//
//
//
static bool sdp_llc_secondary_heatsink_2_over_temperature_counter_set(uint32_t value)
{
    dcdc_llc_secondary_heatsink_2_over_temperature_counter_reset();
    return true;
}

//
//
//
static bool sdp_output_over_voltage_counter_set(uint32_t value)
{
    dcdc_output_voltage_over_voltage_counter_reset();
    return true;
}

//
//
//
static bool sdp_master_startup_set(uint32_t value)
{
    dcdc_master_startup_set();
    return true;
}

//
//
//
static bool sdp_master_shutdown_set(uint32_t value)
{
    dcdc_master_shutdown_set();
    return true;
}

//
//
//
static bool sdp_open_loop_enable_set(uint32_t value)
{
    bool enable = (value != 0UL) ? true : false;
    return dcdc_open_loop_enable_set(enable);
}

//
//
//
static bool sdp_open_loop_primary_enable_set(uint32_t value)
{
    bool enable = (value != 0UL) ? true : false;
    return dcdc_open_loop_primary_enable_set(enable);
}

//
//
//
static bool sdp_open_loop_active_dummy_load_enable_set(uint32_t value)
{
    bool enable = (value != 0UL) ? true : false;
    return dcdc_open_loop_active_dummy_load_enable_set(enable);
}

//
//
//
static bool sdp_open_loop_primary_frequency_set(uint32_t value)
{
    uint16_t period = PWM_PERIOD_IN_COUNTS_UP_COUNTER(value);
    return dcdc_open_loop_primary_period_set(period);
}

//
//
//
static bool sdp_output_voltage_setpoint_x100_set(uint32_t value)
{
    if ((value < DCDC_OUTPUT_VOLTAGE_SETPOINT_MIN_x100)
            || (value > DCDC_OUTPUT_VOLTAGE_SETPOINT_MAX_x100))
        return false;

    dcdc_configuration_s.output_voltage_setpoint = (float32_t)value / 100.0f;
    dcdc_output_voltage_thresholds_reverse_calibration_service();
    return pers_configuration_store_initiate();
}

//
//
//
static bool sdp_voltage_loop_gain_q16_set(uint32_t value)
{
    dcdc_cpu_to_cla_mem.voltage_loop_gain = Q16_TO_FLOAT(value);
    return true;
}

//
//
//
static bool sdp_current_loop_gain_q16_set(uint32_t value)
{
    dcdc_cpu_to_cla_mem.current_loop_gain = Q16_TO_FLOAT(value);
    return true;
}

//
// Write SDP_FACTORY_MODE_VALUE_ENABLED to enable.
// Power cycle to disable factory mode
//
static bool sdp_enter_factory_mode_set(uint32_t value)
{
    if (value == SDP_FACTORY_MODE_VALUE_ENABLED)
    {
        sdp_factory_mode_enabled = true;
        return true;
    }

    return false;
}

//
//
//
static bool sdp_output_voltage_slope_q16_set(uint32_t value)
{
    if (!sdp_factory_mode_enabled)
        return false;

    float32_t slope = Q16_TO_FLOAT(value);
    if ((slope < DCDC_OUTPUT_VOLTAGE_SLOPE_MIN)
            || (slope > DCDC_OUTPUT_VOLTAGE_SLOPE_MAX))
        return false;

    dcdc_factory_s.output_voltage.slope = slope;
    dcdc_output_voltage_thresholds_reverse_calibration_service();
    return pers_factory_store_initiate();
}

//
//
//
static bool sdp_output_voltage_offset_q16_set(uint32_t value)
{
    if (!sdp_factory_mode_enabled)
        return false;

    float32_t offset = Q16_TO_FLOAT(value);
    if ((offset < DCDC_OUTPUT_VOLTAGE_OFFSET_MIN)
            || (offset > DCDC_OUTPUT_VOLTAGE_OFFSET_MAX))
        return false;

    dcdc_factory_s.output_voltage.offset = offset;
    dcdc_output_voltage_thresholds_reverse_calibration_service();
    return pers_factory_store_initiate();
}

//
//
//
static bool sdp_output_current_slope_q16_set(uint32_t value)
{
    if (!sdp_factory_mode_enabled)
        return false;

    float32_t slope = Q16_TO_FLOAT(value);
    if ((slope < DCDC_OUTPUT_CURRENT_SLOPE_MIN)
            || (slope > DCDC_OUTPUT_CURRENT_SLOPE_MAX))
        return false;

    dcdc_factory_s.output_current.slope = slope;
    dcdc_output_current_thresholds_reverse_calibration_service();
    return pers_factory_store_initiate();
}

//
//
//
static bool sdp_output_current_offset_q16_set(uint32_t value)
{
    if (!sdp_factory_mode_enabled)
        return false;

    float32_t offset = Q16_TO_FLOAT(value);
    if ((offset < DCDC_OUTPUT_CURRENT_OFFSET_MIN)
            || (offset > DCDC_OUTPUT_CURRENT_OFFSET_MAX))
        return false;

    dcdc_factory_s.output_current.offset = offset;
    dcdc_output_current_thresholds_reverse_calibration_service();
    return pers_factory_store_initiate();
}



//
//
//
void dcdc_sdp_init(void)
{
    sdp_slave_init(sdp_get_array,
                   sizeof(sdp_get_array) / sizeof(sdp_get_array[0]),
                   sdp_set_array,
                   sizeof(sdp_set_array) / sizeof(sdp_set_array[0]));

    sdp_factory_mode_enabled = false;
}

//
//
//
void dcdc_sdp_service(void)
{
    int16_t c;

    if (SCI_getRxFIFOStatus(SCIA_BASE) > SCI_FIFO_RX0)
    {
        c = SCI_readCharNonBlocking(SCIA_BASE);
        sdp_receive_put(c);
    }

    c = sdp_transmit_peek();
    if (c != SDP_NO_CHARACTER)
    {
        if (SCI_getTxFIFOStatus(SCIA_BASE) < SCI_FIFO_TX16)
        {
            SCI_writeCharNonBlocking(SCIA_BASE, c);
            sdp_transmit_done();
        }
    }

    uint16_t sci_rx_status = SCI_getRxStatus(SCIA_BASE);
    if (sci_rx_status & SCI_RXSTATUS_ERROR)
        SCI_performSoftwareReset(SCIA_BASE);
}
