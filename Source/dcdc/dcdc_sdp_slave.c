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
#include "pers/pers.h"
#include "version.h"


#define SDP_FACTORY_MODE_VALUE_ENABLED  0x12345678UL

#define DCDC_INPUT_VOLTAGE_SLOPE_MIN     0.9f
#define DCDC_INPUT_VOLTAGE_SLOPE_MAX     1.1f
#define DCDC_INPUT_VOLTAGE_OFFSET_MIN   -10.0f
#define DCDC_INPUT_VOLTAGE_OFFSET_MAX    10.0f

#define DCDC_INPUT_CURRENT_SLOPE_MIN     0.9f
#define DCDC_INPUT_CURRENT_SLOPE_MAX     1.1f
#define DCDC_INPUT_CURRENT_OFFSET_MIN   -5.0f
#define DCDC_INPUT_CURRENT_OFFSET_MAX    5.0f

static bool sdp_factory_mode_enabled;


// Function prototypes
static uint32_t sdp_fw_build_get(void);
static uint32_t sdp_hw_id_get(void);
static uint32_t sdp_serial_number_get(void);
static uint32_t sdp_bus_voltage_get(void);
static uint32_t sdp_input_voltage_get(void);
static uint32_t sdp_input_current_x100_get(void);
static uint32_t sdp_ambient_temperature_x100_get(void);
static uint32_t sdp_dcdc_heatsink_1_temperature_x100_get(void);
static uint32_t sdp_dcdc_heatsink_2_temperature_x100_get(void);
static uint32_t sdp_llc_primary_heatsink_temperature_x100_get(void);
static uint32_t sdp_fan_speed_get(void);
static uint32_t sdp_bus_under_voltage_counter_get(void);
static uint32_t sdp_bus_over_voltage_counter_get(void);
static uint32_t sdp_input_over_current_counter_get(void);
static uint32_t sdp_input_current_rms_over_current_counter_get(void);
static uint32_t sdp_ambient_over_temperature_counter_get(void);
static uint32_t sdp_heatsink_1_over_temperature_counter_get(void);
static uint32_t sdp_heatsink_2_over_temperature_counter_get(void);
static uint32_t sdp_llc_primary_heatsink_over_temperature_counter_get(void);
static uint32_t sdp_input_frequency_out_of_range_counter_get(void);
static uint32_t sdp_input_voltage_rms_under_voltage_counter_get(void);
static uint32_t sdp_input_voltage_rms_over_voltage_counter_get(void);
static uint32_t sdp_dcdc_state_get(void);
static uint32_t sdp_voltage_loop_gain_get(void);
static uint32_t sdp_current_loop_gain_get(void);
static uint32_t sdp_input_voltage_slope_q16_get(void);
static uint32_t sdp_input_voltage_offset_q16_get(void);
static uint32_t sdp_input_current_slope_q16_get(void);
static uint32_t sdp_input_current_offset_q16_get(void);

static bool sdp_serial_number_set(uint32_t value);
static bool sdp_bus_under_voltage_counter_set(uint32_t value);
static bool sdp_bus_over_voltage_counter_set(uint32_t value);
static bool sdp_input_over_current_counter_set(uint32_t value);
static bool sdp_input_current_rms_over_current_counter_set(uint32_t value);
static bool sdp_ambient_over_temperature_counter_set(uint32_t value);
static bool sdp_heatsink_1_over_temperature_counter_set(uint32_t value);
static bool sdp_heatsink_2_over_temperature_counter_set(uint32_t value);
static bool sdp_llc_primary_heatsink_over_temperature_counter_set(uint32_t value);
static bool sdp_input_frequency_out_of_range_counter_set(uint32_t value);
static bool sdp_input_voltage_rms_under_voltage_counter_set(uint32_t value);
static bool sdp_input_voltage_rms_over_voltage_counter_set(uint32_t value);
static bool sdp_master_startup_set(uint32_t value);
static bool sdp_master_shutdown_set(uint32_t value);
static bool sdp_open_loop_enable_set(uint32_t value);
static bool sdp_open_loop_hf_legs_enable_set(uint32_t value);
static bool sdp_open_loop_lf_leg_enable_set(uint32_t value);
static bool sdp_open_loop_inrush_protection_enable_set(uint32_t value);
static bool sdp_open_loop_hf_legs_duty_set(uint32_t value);
static bool sdp_voltage_loop_gain_set(uint32_t value);
static bool sdp_current_loop_gain_set(uint32_t value);
static bool sdp_enter_factory_mode_set(uint32_t value);
static bool sdp_input_voltage_slope_q16_set(uint32_t value);
static bool sdp_input_voltage_offset_q16_set(uint32_t value);
static bool sdp_input_current_slope_q16_set(uint32_t value);
static bool sdp_input_current_offset_q16_set(uint32_t value);

static const struct sdp_get sdp_get_array[] =
{
    {  0U, &sdp_fw_build_get},
    {  1U, &sdp_hw_id_get},
    {  2U, &sdp_serial_number_get},
    { 10U, &sdp_bus_voltage_get},
    { 11U, &sdp_input_voltage_get},
    { 12U, &sdp_input_current_x100_get},
    { 13U, &sdp_ambient_temperature_x100_get},
    { 14U, &sdp_dcdc_heatsink_1_temperature_x100_get},
    { 15U, &sdp_dcdc_heatsink_2_temperature_x100_get},
    { 16U, &sdp_llc_primary_heatsink_temperature_x100_get},
    { 18U, &sdp_fan_speed_get},
    { 20U, &sdp_bus_under_voltage_counter_get},
    { 21U, &sdp_bus_over_voltage_counter_get},
    { 22U, &sdp_input_over_current_counter_get},
    { 23U, &sdp_input_current_rms_over_current_counter_get},
    { 24U, &sdp_ambient_over_temperature_counter_get},
    { 25U, &sdp_heatsink_1_over_temperature_counter_get},
    { 26U, &sdp_heatsink_2_over_temperature_counter_get},
    { 27U, &sdp_llc_primary_heatsink_over_temperature_counter_get},
    { 28U, &sdp_input_frequency_out_of_range_counter_get},
    { 29U, &sdp_input_voltage_rms_under_voltage_counter_get},
    { 30U, &sdp_input_voltage_rms_over_voltage_counter_get},
    { 40U, &sdp_dcdc_state_get},
    { 51U, &sdp_voltage_loop_gain_get},
    { 52U, &sdp_current_loop_gain_get},
    {101U, &sdp_input_voltage_slope_q16_get},
    {102U, &sdp_input_voltage_offset_q16_get},
    {103U, &sdp_input_current_slope_q16_get},
    {104U, &sdp_input_current_offset_q16_get},
};

static const struct sdp_set sdp_set_array[] =
{
    {  2U, &sdp_serial_number_set},
    { 20U, &sdp_bus_under_voltage_counter_set},
    { 21U, &sdp_bus_over_voltage_counter_set},
    { 22U, &sdp_input_over_current_counter_set},
    { 23U, &sdp_input_current_rms_over_current_counter_set},
    { 24U, &sdp_ambient_over_temperature_counter_set},
    { 25U, &sdp_heatsink_1_over_temperature_counter_set},
    { 26U, &sdp_heatsink_2_over_temperature_counter_set},
    { 27U, &sdp_llc_primary_heatsink_over_temperature_counter_set},
    { 28U, &sdp_input_frequency_out_of_range_counter_set},
    { 29U, &sdp_input_voltage_rms_under_voltage_counter_set},
    { 30U, &sdp_input_voltage_rms_over_voltage_counter_set},
    { 41U, &sdp_master_startup_set},
    { 42U, &sdp_master_shutdown_set},
    { 43U, &sdp_open_loop_enable_set},
    { 44U, &sdp_open_loop_hf_legs_enable_set},
    { 45U, &sdp_open_loop_lf_leg_enable_set},
    { 46U, &sdp_open_loop_inrush_protection_enable_set},
    { 47U, &sdp_open_loop_hf_legs_duty_set},
    { 51U, &sdp_voltage_loop_gain_set},
    { 52U, &sdp_current_loop_gain_set},
    {100U, &sdp_enter_factory_mode_set},
    {101U, &sdp_input_voltage_slope_q16_set},
    {102U, &sdp_input_voltage_offset_q16_set},
    {103U, &sdp_input_current_slope_q16_set},
    {104U, &sdp_input_current_offset_q16_set},
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
static uint32_t sdp_bus_voltage_get(void)
{
    return (uint32_t)(((float32_t)dcdc_cla_to_cpu_mem.v_bus_raw.cpu
            * DCDC_CALIBRATION_V_BUS_SLOPE_DEFAULT)
            + DCDC_CALIBRATION_V_BUS_OFFSET_DEFAULT);
}

//
//
//
static uint32_t sdp_input_voltage_get(void)
{
    float32_t v_in_rms_raw = (float32_t)dcdc_cla_to_cpu_mem.v_in_raw_rms.cpu;
    float32_t v_in_rms_default_calibrated =
            (v_in_rms_raw * DCDC_CALIBRATION_V_IN_SLOPE_DEFAULT)
            + DCDC_CALIBRATION_V_IN_OFFSET_DEFAULT;
    float32_t v_in_rms_calibrated =
            (v_in_rms_default_calibrated * dcdc_factory_s.input_voltage.slope)
            + dcdc_factory_s.input_voltage.offset;
    return (uint32_t)v_in_rms_calibrated;
}

//
//
//
static uint32_t sdp_input_current_x100_get(void)
{
    float32_t i_in_rms_raw = (float32_t)dcdc_cla_to_cpu_mem.i_in_raw_rms.cpu;
    float32_t i_in_rms_default_calibrated_x100 =
            ((i_in_rms_raw * DCDC_CALIBRATION_I_IN_SLOPE_DEFAULT)
            + DCDC_CALIBRATION_I_IN_OFFSET_DEFAULT) * 100.0f;
    float32_t i_in_rms_calibrated_x100 =
            (i_in_rms_default_calibrated_x100 * dcdc_factory_s.input_current.slope)
            + dcdc_factory_s.input_current.offset;
    return (uint32_t)i_in_rms_calibrated_x100;
}

//
//
//
static uint32_t sdp_ambient_temperature_x100_get(void)
{
    return dcdc_ambient_temperature_x100_get();
}

//
//
//
static uint32_t sdp_dcdc_heatsink_1_temperature_x100_get(void)
{
    return dcdc_heatsink_1_temperature_x100_get();
}

//
//
//
static uint32_t sdp_dcdc_heatsink_2_temperature_x100_get(void)
{
    return dcdc_heatsink_2_temperature_x100_get();
}

//
//
//
static uint32_t sdp_llc_primary_heatsink_temperature_x100_get(void)
{
    return dcdc_llc_primary_heatsink_temperature_x100_get();
}

//
//
//
static uint32_t sdp_fan_speed_get(void)
{
    return dcdc_fan_rpm_get();
}

//
//
//
static uint32_t sdp_bus_under_voltage_counter_get(void)
{
    return dcdc_bus_under_voltage_counter_get();
}

//
//
//
static uint32_t sdp_bus_over_voltage_counter_get(void)
{
    return dcdc_bus_over_voltage_counter_get();
}

//
//
//
static uint32_t sdp_input_over_current_counter_get(void)
{
    return dcdc_input_over_current_counter_get();
}

//
//
//
static uint32_t sdp_input_current_rms_over_current_counter_get(void)
{
    return dcdc_input_current_rms_over_current_counter_get();
}

//
//
//
static uint32_t sdp_ambient_over_temperature_counter_get(void)
{
    return dcdc_ambient_over_temperature_counter_get();
}

//
//
//
static uint32_t sdp_heatsink_1_over_temperature_counter_get(void)
{
    return dcdc_heatsink_1_over_temperature_counter_get();
}

//
//
//
static uint32_t sdp_heatsink_2_over_temperature_counter_get(void)
{
    return dcdc_heatsink_2_over_temperature_counter_get();
}

//
//
//
static uint32_t sdp_llc_primary_heatsink_over_temperature_counter_get(void)
{
    return dcdc_llc_primary_heatsink_over_temperature_counter_get();
}

//
//
//
static uint32_t sdp_input_frequency_out_of_range_counter_get(void)
{
    return dcdc_input_frequency_out_of_range_counter_get();
}

//
//
//
static uint32_t sdp_input_voltage_rms_under_voltage_counter_get(void)
{
    return dcdc_input_voltage_rms_under_voltage_counter_get();
}

//
//
//
static uint32_t sdp_input_voltage_rms_over_voltage_counter_get(void)
{
    return dcdc_input_voltage_rms_over_voltage_counter_get();
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
static uint32_t sdp_voltage_loop_gain_get(void)
{
    return FLOAT_TO_Q16(dcdc_cpu_to_cla_mem.voltage_loop_gain);
}

//
//
//
static uint32_t sdp_current_loop_gain_get(void)
{
    return FLOAT_TO_Q16(dcdc_cpu_to_cla_mem.current_loop_gain);
}

//
//
//
static uint32_t sdp_input_voltage_slope_q16_get(void)
{
    return FLOAT_TO_Q16(dcdc_factory_s.input_voltage.slope);
}

//
//
//
static uint32_t sdp_input_voltage_offset_q16_get(void)
{
    return FLOAT_TO_Q16(dcdc_factory_s.input_voltage.offset);
}

//
//
//
static uint32_t sdp_input_current_slope_q16_get(void)
{
    return FLOAT_TO_Q16(dcdc_factory_s.input_current.slope);
}

//
//
//
static uint32_t sdp_input_current_offset_q16_get(void)
{
    return FLOAT_TO_Q16(dcdc_factory_s.input_current.offset);
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
static bool sdp_bus_under_voltage_counter_set(uint32_t value)
{
    dcdc_bus_under_voltage_counter_reset();
    return true;
}

//
//
//
static bool sdp_bus_over_voltage_counter_set(uint32_t value)
{
    dcdc_bus_over_voltage_counter_reset();
    return true;
}

//
//
//
static bool sdp_input_over_current_counter_set(uint32_t value)
{
    dcdc_input_over_current_counter_reset();
    return true;
}

//
//
//
static bool sdp_input_current_rms_over_current_counter_set(uint32_t value)
{
    dcdc_input_current_rms_over_current_counter_reset();
    return true;
}

//
//
//
static bool sdp_ambient_over_temperature_counter_set(uint32_t value)
{
    dcdc_ambient_over_temperature_counter_reset();
    return true;
}

//
//
//
static bool sdp_heatsink_1_over_temperature_counter_set(uint32_t value)
{
    dcdc_heatsink_1_over_temperature_counter_reset();
    return true;
}

//
//
//
static bool sdp_heatsink_2_over_temperature_counter_set(uint32_t value)
{
    dcdc_heatsink_2_over_temperature_counter_reset();
    return true;
}

//
//
//

static bool sdp_llc_primary_heatsink_over_temperature_counter_set(uint32_t value)
{
    dcdc_llc_primary_heatsink_over_temperature_counter_reset();
    return true;
}

//
//
//
static bool sdp_input_frequency_out_of_range_counter_set(uint32_t value)
{
    dcdc_input_frequency_out_of_range_counter_reset();
    return true;
}

//
//
//
static bool sdp_input_voltage_rms_under_voltage_counter_set(uint32_t value)
{
    dcdc_input_voltage_rms_under_voltage_counter_reset();
    return true;
}

//
//
//
static bool sdp_input_voltage_rms_over_voltage_counter_set(uint32_t value)
{
    dcdc_input_voltage_rms_over_voltage_counter_reset();
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
static bool sdp_open_loop_hf_legs_enable_set(uint32_t value)
{
    bool enable = (value != 0UL) ? true : false;
    return dcdc_open_loop_hf_legs_enable_set(enable);
}

//
//
//
static bool sdp_open_loop_lf_leg_enable_set(uint32_t value)
{
    bool enable = (value != 0UL) ? true : false;
    return dcdc_open_loop_lf_leg_enable_set(enable);
}

//
//
//
static bool sdp_open_loop_inrush_protection_enable_set(uint32_t value)
{
    bool enable = (value != 0UL) ? true : false;
    return dcdc_open_loop_inrush_protection_enable_set(enable);
}

//
//
//
static bool sdp_open_loop_hf_legs_duty_set(uint32_t value)
{
    return dcdc_open_loop_hf_legs_duty_set(value);
}

//
//
//
static bool sdp_voltage_loop_gain_set(uint32_t value)
{
    dcdc_cpu_to_cla_mem.voltage_loop_gain = Q16_TO_FLOAT(value);
    return true;
}

//
//
//
static bool sdp_current_loop_gain_set(uint32_t value)
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
static bool sdp_input_voltage_slope_q16_set(uint32_t value)
{
    if (!sdp_factory_mode_enabled)
        return false;

    float32_t slope = Q16_TO_FLOAT(value);
    if ((slope < DCDC_INPUT_VOLTAGE_SLOPE_MIN)
            || (slope > DCDC_INPUT_VOLTAGE_SLOPE_MAX))
        return false;

    dcdc_factory_s.input_voltage.slope = slope;
    dcdc_input_voltage_thresholds_reverse_calibration_service();
    return pers_factory_store_initiate();
}

//
//
//
static bool sdp_input_voltage_offset_q16_set(uint32_t value)
{
    if (!sdp_factory_mode_enabled)
        return false;

    float32_t offset = Q16_TO_FLOAT(value);
    if ((offset < DCDC_INPUT_VOLTAGE_OFFSET_MIN)
            || (offset > DCDC_INPUT_VOLTAGE_OFFSET_MAX))
        return false;

    dcdc_factory_s.input_voltage.offset = offset;
    dcdc_input_voltage_thresholds_reverse_calibration_service();
    return pers_factory_store_initiate();
}

//
//
//
static bool sdp_input_current_slope_q16_set(uint32_t value)
{
    if (!sdp_factory_mode_enabled)
        return false;

    float32_t slope = Q16_TO_FLOAT(value);
    if ((slope < DCDC_INPUT_CURRENT_SLOPE_MIN)
            || (slope > DCDC_INPUT_CURRENT_SLOPE_MAX))
        return false;

    dcdc_factory_s.input_current.slope = slope;
    dcdc_input_current_thresholds_reverse_calibration_service();
    return pers_factory_store_initiate();
}

//
//
//
static bool sdp_input_current_offset_q16_set(uint32_t value)
{
    if (!sdp_factory_mode_enabled)
        return false;

    float32_t offset = Q16_TO_FLOAT(value);
    if ((offset < DCDC_INPUT_CURRENT_OFFSET_MIN)
            || (offset > DCDC_INPUT_CURRENT_OFFSET_MAX))
        return false;

    dcdc_factory_s.input_current.offset = offset;
    dcdc_input_current_thresholds_reverse_calibration_service();
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
