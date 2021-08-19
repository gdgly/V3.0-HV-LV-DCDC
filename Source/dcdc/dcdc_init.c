///////////////////////////////////////////////////////////////////////////////
//
// Initialization file
//
///////////////////////////////////////////////////////////////////////////////


//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "dcdc.h"
#include "clb_sr/clb_config.h"
#include "spi_flash/spi_flash.h"
#include "pers/pers.h"


#define ADC_ACQUISITION_WINDOW_SIZE         8U

// TODO: Set to proper values
#define DCDC_CMPSS_DAC_VALUE_OUTPUT_OCP         3072U
#define DCDC_CMPSS_DAC_VALUE_OUTPUT_HW_OVP      3072U
#define DCDC_CMPSS_DAC_VALUE_OUTPUT_CURRENT_SR   500U

struct dcdc_factory dcdc_factory_s;
struct dcdc_configuration dcdc_configuration_s;



//
// Configure watchdog.
//
static void wdog_init(void)
{
    // Set the watchdog to generate a reset signal
    SysCtl_setWatchdogMode(SYSCTL_WD_MODE_RESET);

    SysCtl_setWatchdogPredivider(SYSCTL_WD_PREDIV_4096);
    SysCtl_setWatchdogPrescaler(SYSCTL_WD_PRESCALE_64);

    // Reset the watchdog counter
    SysCtl_serviceWatchdog();

    // Enable the watchdog
    SysCtl_enableWatchdog();
}

//
//
//
static void pwm_basic_init(uint32_t base, uint16_t period)
{
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(base, period);
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, (period >> 1U)); // 50% duty
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);
    EPWM_setTimeBaseCounter(base, 0U);
    EPWM_setSyncOutPulseMode(base, EPWM_SYNC_OUT_PULSE_DISABLED);
    EPWM_setPhaseShift(base, 0U);
    EPWM_disablePhaseShiftLoad(base);
    EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO);
    // Run freely in emulation mode
    EPWM_setEmulationMode(base, EPWM_EMULATION_FREE_RUN);
}

//
//
//
static void pwm_global_load_config(uint32_t base)
{
    EPWM_setGlobalLoadTrigger(base, EPWM_GL_LOAD_PULSE_CNTR_ZERO);
    EPWM_enableGlobalLoadOneShotMode(base);
    EPWM_enableGlobalLoadRegisters(base, EPWM_GL_REGISTER_TBPRD_TBPRDHR
                                   | EPWM_GL_REGISTER_CMPA_CMPAHR);

    EPWM_enableGlobalLoad(base);
}

//
//
//
static void pwm_deadband_config(uint32_t base, uint16_t deadband)
{
    // The shadow must be configured first. Otherwise, the deadband
    // configuration will be cleared when enabling the shadow!
    EPWM_setDeadBandControlShadowLoadMode(base, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    // Invert only the Falling Edge delayed output (Active High Complementary)
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true);
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_A, false);
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_B, false);
    EPWM_setRisingEdgeDelayCount(base, deadband);
    EPWM_setFallingEdgeDelayCount(base, deadband);
    EPWM_setDeadBandCounterClock(base, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
}

//
//
//
static void pwm_ost_tripzone_config(uint32_t base)
{
    // Configure ePWMxA and ePWMxB to output low on DCAEVT1 TRIP
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_DCAEVT1,
                           EPWM_TZ_ACTION_LOW);
    // Enable DCA as OST
    EPWM_enableTripZoneSignals(base, EPWM_TZ_SIGNAL_DCAEVT1);
}

//
//
//
static void pwm_cbc_tripzone_config(uint32_t base)
{
    // Configure ePWMxA and ePWMxB to output low on DCAEVT2 TRIP (Cycle-by-Cycle)
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_DCAEVT2,
                           EPWM_TZ_ACTION_LOW);
    // Trigger event when DCAL is high
    EPWM_setTripZoneDigitalCompareEventCondition(base,
                                                 EPWM_TZ_DC_OUTPUT_A2,
                                                 EPWM_TZ_EVENT_DCXL_HIGH);
    // Configure DCAL to use TRIP4 as an input
    EPWM_enableDigitalCompareTripCombinationInput(base,
                                                  EPWM_DC_COMBINATIONAL_TRIPIN4,
                                                  EPWM_DC_TYPE_DCAL);
    // Enable DCA as Cycle-by-Cycle
    EPWM_enableTripZoneSignals(base, EPWM_TZ_SIGNAL_DCAEVT2);
    // Configure the DCA path to be unfiltered and asynchronous
    EPWM_setDigitalCompareEventSource(base,
                                      EPWM_DC_MODULE_A,
                                      EPWM_DC_EVENT_2,
                                      EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);



    // Configure ePWMxA and ePWMxB to output low on DCBEVT2 TRIP (Cycle-by-Cycle)
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_DCBEVT2,
                           EPWM_TZ_ACTION_LOW);
    // Trigger event when DCBL is high
    EPWM_setTripZoneDigitalCompareEventCondition(base,
                                                 EPWM_TZ_DC_OUTPUT_B2,
                                                 EPWM_TZ_EVENT_DCXL_HIGH);
    // Configure DCBL to use TRIP5 as an input
    EPWM_enableDigitalCompareTripCombinationInput(base,
                                                  EPWM_DC_COMBINATIONAL_TRIPIN5,
                                                  EPWM_DC_TYPE_DCBL);
    // Enable DCB as Cycle-by-Cycle
    EPWM_enableTripZoneSignals(base, EPWM_TZ_SIGNAL_DCBEVT2);
    // Configure the DCB path to be unfiltered and asynchronous
    EPWM_setDigitalCompareEventSource(base,
                                      EPWM_DC_MODULE_B,
                                      EPWM_DC_EVENT_2,
                                      EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
}

//
// EPWM Initialization
//
static void initEPWM(void)
{
    // Disable sync(Freeze clock to PWM as well)
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);



    // Set up EPWM1
    pwm_basic_init(EPWM1_BASE, EPWM1_PERIOD_MIN);
    pwm_global_load_config(EPWM1_BASE);

    EPWM_setSyncOutPulseMode(EPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

    // Configuring action-qualifiers for EPWM1 to generate
    // complementary outputs on channel A and B
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    pwm_deadband_config(EPWM1_BASE, EPWM1_DEADBAND);
    pwm_ost_tripzone_config(EPWM1_BASE);
    pwm_cbc_tripzone_config(EPWM1_BASE);

    // Enabling EPWM1 interrupt at TBCTR = 0 to trigger ISR
    EPWM_setInterruptSource(EPWM1_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM1_BASE);
    EPWM_setInterruptEventCount(EPWM1_BASE, 1U);



    // Set up EPWM5
    pwm_basic_init(EPWM5_BASE, EPWM5_PERIOD);

    // Configuring action-qualifiers for EPWM5 to generate
    // output on channel A
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
}

//
//
//
static void adc_basic_init(uint32_t base)
{
    // TODO: Change to ADC_REFERENCE_EXTERNAL
    // Enable internal reference on ADCs
    ADC_setVREF(base, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    // Set ADCCLK divider to /2
    ADC_setPrescaler(base, ADC_CLK_DIV_2_0);

    // Set pulse positions to late
    ADC_setInterruptPulseMode(base, ADC_PULSE_END_OF_CONV);

    // Power up the ADC and then delay for 5ms.
    ADC_enableConverter(base);
    DEVICE_DELAY_US(5000);
}

//
// Function to configure and power up ADCA, ADCB, and ADCC.
//
static void initADC(void)
{
    adc_basic_init(ADCA_BASE);
    adc_basic_init(ADCB_BASE);
    adc_basic_init(ADCC_BASE);

    // Configure the SOC to occur on the first zero event of PWM1
    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPB);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);


    // Configure Output Voltage 1 ADC conversion.
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN1, ADC_ACQUISITION_WINDOW_SIZE);
    // Configure Output Current ADC conversion.
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN0, ADC_ACQUISITION_WINDOW_SIZE);
    // Configure LLC Secondary Heatsink 1 Temperature ADC conversion.
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN3, ADC_ACQUISITION_WINDOW_SIZE);

    // Configure Output Voltage 2 ADC conversion.
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN1, ADC_ACQUISITION_WINDOW_SIZE);
    // Configure LLC Secondary Heatsink 2 Temperature ADC conversion.
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN2, ADC_ACQUISITION_WINDOW_SIZE);
    // Configure HW Revision ADC conversion.
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN3, ADC_ACQUISITION_WINDOW_SIZE);

    // Configure Output Current External Setpoint ADC conversion.
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN1, ADC_ACQUISITION_WINDOW_SIZE);
    // Configure Shelf ID ADC conversion.
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN3, ADC_ACQUISITION_WINDOW_SIZE);
    // Configure Slot ID conversion.
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN2, ADC_ACQUISITION_WINDOW_SIZE);


    // Enabling ADCA interrupt to trigger CLA Task 1
    ADC_enableContinuousMode(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
}

//
//
//
static void cmpss_basic_init(uint32_t base, bool inverted_output)
{
    // Enable CMPSS and configure the negative input signal to come from the DAC
    CMPSS_enableModule(base);
    CMPSS_configHighComparator(base, CMPSS_INSRC_DAC
                               | (inverted_output ? CMPSS_INV_INVERTED : 0));

    // Use VDDA as the reference for the DAC
    CMPSS_configDAC(base, CMPSS_DACREF_VDDA
                    | CMPSS_DACVAL_SYSCLK
                    | CMPSS_DACSRC_SHDW);

    // Configure the output signals. CTRIPH will be fed by
    // the asynchronous comparator output.
    CMPSS_configOutputsHigh(base, CMPSS_TRIP_ASYNC_COMP);
}

//
//
//
static void initCMPSS(void)
{
    // Select CMPSS-4 High comparator Positive input 0
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_4, 0);
    cmpss_basic_init(CMPSS4_BASE, false);
    CMPSS_setDACValueHigh(CMPSS4_BASE, DCDC_CMPSS_DAC_VALUE_OUTPUT_OCP);
    // Configure TRIP4 to be CTRIP4H using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(XBAR_TRIP4, XBAR_EPWM_MUX06_CMPSS4_CTRIPH);
    XBAR_enableEPWMMux(XBAR_TRIP4, XBAR_MUX06);


    // Select CMPSS-7 High comparator Positive input 1
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_7, 1);
    cmpss_basic_init(CMPSS7_BASE, false);
    CMPSS_setDACValueHigh(CMPSS7_BASE, DCDC_CMPSS_DAC_VALUE_OUTPUT_HW_OVP);
    // Configure TRIP5 to be CTRIP7H using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(XBAR_TRIP5, XBAR_EPWM_MUX12_CMPSS7_CTRIPH);
    XBAR_enableEPWMMux(XBAR_TRIP5, XBAR_MUX12);


    // Configure comparators to be used for synchronous rectification.

    // Select CMPSS-1 High comparator Positive input 0
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_1, 0);
    cmpss_basic_init(CMPSS1_BASE, false);
    CMPSS_setDACValueHigh(CMPSS1_BASE, DCDC_CMPSS_DAC_VALUE_OUTPUT_CURRENT_SR);


    // Select CMPSS-2 High comparator Positive input 0
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_2, 0);
    cmpss_basic_init(CMPSS2_BASE, false);
    CMPSS_setDACValueHigh(CMPSS2_BASE, DCDC_CMPSS_DAC_VALUE_OUTPUT_CURRENT_SR);


    // Select CMPSS-5 High comparator Positive input 0
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_5, 0);
    cmpss_basic_init(CMPSS5_BASE, false);
    CMPSS_setDACValueHigh(CMPSS5_BASE, DCDC_CMPSS_DAC_VALUE_OUTPUT_CURRENT_SR);


    // Select CMPSS-6 High comparator Positive input 0
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_6, 0);
    cmpss_basic_init(CMPSS6_BASE, false);
    CMPSS_setDACValueHigh(CMPSS6_BASE, DCDC_CMPSS_DAC_VALUE_OUTPUT_CURRENT_SR);
}

//
//
//
static void CLB_init(void)
{
    //CLB1 initialization
    CLB_setOutputMask(CLB1_BASE, (0 << 0), true);
    //CLB1 CLB_IN0 initialization
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_CLB_AUXSIG0);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN0, CLB_GP_IN_MUX_EXTERNAL);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN0, CLB_FILTER_NONE);
    //CLB1 CLB_IN1 initialization
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_CLB_AUXSIG1);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN1, CLB_FILTER_NONE);
    CLB_setGPREG(CLB1_BASE,0);
    CLB_disableCLB(CLB1_BASE);


    //CLB2 initialization
    CLB_setOutputMask(CLB2_BASE, (0 << 0), true);
    //CLB2 CLB_IN0 initialization
    CLB_configLocalInputMux(CLB2_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configGlobalInputMux(CLB2_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_CLB_AUXSIG2);
    CLB_configGPInputMux(CLB2_BASE, CLB_IN0, CLB_GP_IN_MUX_EXTERNAL);
    CLB_selectInputFilter(CLB2_BASE, CLB_IN0, CLB_FILTER_NONE);
    //CLB2 CLB_IN1 initialization
    CLB_configLocalInputMux(CLB2_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configGlobalInputMux(CLB2_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_CLB_AUXSIG3);
    CLB_configGPInputMux(CLB2_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
    CLB_selectInputFilter(CLB2_BASE, CLB_IN1, CLB_FILTER_NONE);
    CLB_setGPREG(CLB2_BASE,0);
    CLB_disableCLB(CLB2_BASE);


    //CLB3 initialization
    CLB_setOutputMask(CLB3_BASE, (0 << 0), true);
    //CLB3 CLB_IN0 initialization
    CLB_configLocalInputMux(CLB3_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configGlobalInputMux(CLB3_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_CLB_AUXSIG4);
    CLB_configGPInputMux(CLB3_BASE, CLB_IN0, CLB_GP_IN_MUX_EXTERNAL);
    CLB_selectInputFilter(CLB3_BASE, CLB_IN0, CLB_FILTER_NONE);
    //CLB3 CLB_IN1 initialization
    CLB_configLocalInputMux(CLB3_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configGlobalInputMux(CLB3_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_CLB_AUXSIG5);
    CLB_configGPInputMux(CLB3_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
    CLB_selectInputFilter(CLB3_BASE, CLB_IN1, CLB_FILTER_NONE);
    CLB_setGPREG(CLB3_BASE,0);
    CLB_disableCLB(CLB3_BASE);


    //CLB4 initialization
    CLB_setOutputMask(CLB4_BASE, (0 << 0), true);
    //CLB4 CLB_IN0 initialization
    CLB_configLocalInputMux(CLB4_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configGlobalInputMux(CLB4_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_CLB_AUXSIG6);
    CLB_configGPInputMux(CLB4_BASE, CLB_IN0, CLB_GP_IN_MUX_EXTERNAL);
    CLB_selectInputFilter(CLB4_BASE, CLB_IN0, CLB_FILTER_NONE);
    //CLB4 CLB_IN1 initialization
    CLB_configLocalInputMux(CLB4_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configGlobalInputMux(CLB4_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_CLB_AUXSIG7);
    CLB_configGPInputMux(CLB4_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
    CLB_selectInputFilter(CLB4_BASE, CLB_IN1, CLB_FILTER_NONE);
    CLB_setGPREG(CLB4_BASE,0);
    CLB_disableCLB(CLB4_BASE);
}

//
//
//
static void CLBXBAR_init(void)
{
    //CLBXBAR0 initialization
    XBAR_setCLBMuxConfig(XBAR_AUXSIG0, XBAR_CLB_MUX00_CMPSS1_CTRIPH);
    XBAR_enableCLBMux(XBAR_AUXSIG0, XBAR_MUX00);

    //CLBXBAR1 initialization
    XBAR_setCLBMuxConfig(XBAR_AUXSIG1, XBAR_CLB_MUX01_INPUTXBAR1);
    XBAR_enableCLBMux(XBAR_AUXSIG1, XBAR_MUX01);

    //CLBXBAR2 initialization
    XBAR_setCLBMuxConfig(XBAR_AUXSIG2, XBAR_CLB_MUX02_CMPSS2_CTRIPH);
    XBAR_enableCLBMux(XBAR_AUXSIG2, XBAR_MUX02);

    //CLBXBAR3 initialization
    XBAR_setCLBMuxConfig(XBAR_AUXSIG3, XBAR_CLB_MUX03_INPUTXBAR2);
    XBAR_enableCLBMux(XBAR_AUXSIG3, XBAR_MUX03);

    //CLBXBAR4 initialization
    XBAR_setCLBMuxConfig(XBAR_AUXSIG4, XBAR_CLB_MUX08_CMPSS5_CTRIPH);
    XBAR_enableCLBMux(XBAR_AUXSIG4, XBAR_MUX08);

    //CLBXBAR5 initialization
    XBAR_setCLBMuxConfig(XBAR_AUXSIG5, XBAR_CLB_MUX09_INPUTXBAR5);
    XBAR_enableCLBMux(XBAR_AUXSIG5, XBAR_MUX09);

    //CLBXBAR6 initialization
    XBAR_setCLBMuxConfig(XBAR_AUXSIG6, XBAR_CLB_MUX10_CMPSS6_CTRIPH);
    XBAR_enableCLBMux(XBAR_AUXSIG6, XBAR_MUX10);

    //CLBXBAR7 initialization
    XBAR_setCLBMuxConfig(XBAR_AUXSIG7, XBAR_CLB_MUX11_INPUTXBAR6);
    XBAR_enableCLBMux(XBAR_AUXSIG7, XBAR_MUX11);
}

//
//
//
static void INPUTXBAR_init(void)
{
    //INPUTXBAR initialization
    XBAR_setInputPin(XBAR_INPUT1, 57);
    XBAR_setInputPin(XBAR_INPUT2, 22);
    XBAR_setInputPin(XBAR_INPUT5, 40);
    XBAR_setInputPin(XBAR_INPUT6, 39);
}

//
//
//
static void OUTPUTXBAR_init(void)
{
    //OUTPUTXBAR0 initialization
    XBAR_setOutputLatchMode(XBAR_OUTPUT3, false);
    XBAR_invertOutputSignal(XBAR_OUTPUT3, false);
    //Mux configuration
    XBAR_setOutputMuxConfig(XBAR_OUTPUT3, XBAR_OUT_MUX01_CLB1_OUT4);
    XBAR_enableOutputMux(XBAR_OUTPUT3, XBAR_MUX01);


    //OUTPUTXBAR1 initialization
    XBAR_setOutputLatchMode(XBAR_OUTPUT4, false);
    XBAR_invertOutputSignal(XBAR_OUTPUT4, false);
    //Mux configuration
    XBAR_setOutputMuxConfig(XBAR_OUTPUT4, XBAR_OUT_MUX05_CLB2_OUT4);
    XBAR_enableOutputMux(XBAR_OUTPUT4, XBAR_MUX05);


    //OUTPUTXBAR2 initialization
    XBAR_setOutputLatchMode(XBAR_OUTPUT5, false);
    XBAR_invertOutputSignal(XBAR_OUTPUT5, false);
    //Mux configuration
    XBAR_setOutputMuxConfig(XBAR_OUTPUT5, XBAR_OUT_MUX09_CLB3_OUT4);
    XBAR_enableOutputMux(XBAR_OUTPUT5, XBAR_MUX09);


    //OUTPUTXBAR3 initialization
    XBAR_setOutputLatchMode(XBAR_OUTPUT8, false);
    XBAR_invertOutputSignal(XBAR_OUTPUT8, false);
    //Mux configuration
    XBAR_setOutputMuxConfig(XBAR_OUTPUT8, XBAR_OUT_MUX13_CLB4_OUT4);
    XBAR_enableOutputMux(XBAR_OUTPUT8, XBAR_MUX13);
}

//
//
//
static void init_tiles(void)
{
    initTILE0(CLB1_BASE);
    initTILE1(CLB2_BASE);
    initTILE2(CLB3_BASE);
    initTILE3(CLB4_BASE);
}

//
//
//
static void CLB_enable(void)
{

    CLB_enableCLB(CLB1_BASE);
    CLB_enableCLB(CLB2_BASE);
    CLB_enableCLB(CLB3_BASE);
    CLB_enableCLB(CLB4_BASE);
}

//
//
//
static void initSCI(void)
{
    SCI_clearOverflowStatus(SCIA_BASE);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
    SCI_resetChannels(SCIA_BASE);

    SCI_setConfig(
            SCIA_BASE, DEVICE_LSPCLK_FREQ, 9600,
            (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));
    SCI_performSoftwareReset(SCIA_BASE);
    SCI_enableFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);
}

//
//
//
static void initSPI(void)
{
    //SPI A initialization (SPI Flash)
    SPI_disableModule(SPIA_BASE);
    // TODO: Do we need to adjust DEVICE_LSPCLK_FREQ for higher speeds?
    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_MASTER, DEVICE_LSPCLK_FREQ / 4, 8);
    SPI_resetTxFIFO(SPIA_BASE);
    SPI_resetRxFIFO(SPIA_BASE);
    SPI_enableFIFO(SPIA_BASE);
    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_STOP_AFTER_TRANSMIT);
    SPI_enableModule(SPIA_BASE);
}

//
//
//
static void setupGPIO(void)
{
    GPIO_setPinConfig(GPIO_0_EPWM1_A);      // High-Frequency Leg 1 PWMs
    GPIO_setPinConfig(GPIO_1_EPWM1_B);

    // Enable GPIO output on GPIO3
    GPIO_writePin(3, 0);                            // Load output latch
    GPIO_setPinConfig(GPIO_3_GPIO3);                // GPIO3 = GPIO3
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);    // GPIO3 = output
    GPIO_setMasterCore(3, GPIO_CORE_CPU1_CLA1);

    GPIO_setPinConfig(GPIO_4_OUTPUTXBAR3);  // SR PWMs
    GPIO_setPinConfig(GPIO_6_OUTPUTXBAR4);  // SR PWMs
    GPIO_setPinConfig(GPIO_7_OUTPUTXBAR5);  // SR PWMs

    // SPI Flash
    GPIO_setPinConfig(GPIO_8_SPIA_SIMO);
    GPIO_setPinConfig(GPIO_9_SPIA_CLK);
    GPIO_setPinConfig(GPIO_10_SPIA_SOMI);

    GPIO_writePin(11, 1);                           // Load output latch
    GPIO_setPinConfig(GPIO_11_GPIO11);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);   // GPIO11 = output

    // Enable GPIO output on GPIO12
    GPIO_writePin(12, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_12_GPIO12);              // GPIO12 = GPIO12
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);   // GPIO12 = output

    // Enable GPIO output on GPIO13
    GPIO_writePin(13, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_13_GPIO13);              // GPIO13 = GPIO13
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);   // GPIO13 = output

    // SPI Debug
    GPIO_setPinConfig(GPIO_14_SPIB_CLK);
    GPIO_setPinConfig(GPIO_15_SPIB_STE);

    GPIO_setPinConfig(GPIO_16_EPWM5_A);     // Auxiliary power PWM

    GPIO_setPinConfig(GPIO_17_OUTPUTXBAR8); // SR PWMs

    GPIO_setPinConfig(GPIO_18_GPIO18_X2);

    // Enable GPIO input on GPIO22
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_22_GPIO22);              // GPIO22 = GPIO22
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_IN);    // GPIO22 = input

    // Enable GPIO input on GPIO23
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_23_GPIO23);              // GPIO23 = GPIO23
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_IN);    // GPIO23 = input

    // Enable GPIO input on GPIO25
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_25_GPIO25);              // GPIO25 = GPIO25
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);    // GPIO25 = input

    GPIO_setPinConfig(GPIO_28_SCIA_RX);
    GPIO_setPinConfig(GPIO_29_SCIA_TX);

    // SPI Debug
    GPIO_setPinConfig(GPIO_30_SPIB_SIMO);

    // Enable GPIO output on GPIO31
    GPIO_writePin(31, 1);                           // Load output latch
    GPIO_setPinConfig(GPIO_31_GPIO31);              // GPIO31 = GPIO31
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_OUT);   // GPIO31 = output

    // Enable GPIO output on GPIO34
    GPIO_writePin(34, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_34_GPIO34);              // GPIO34 = GPIO34
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);   // GPIO34 = output

    GPIO_setPinConfig(GPIO_35_TDI);
    GPIO_setPinConfig(GPIO_37_TDO);

    // Enable GPIO input on GPIO39
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_39_GPIO39);              // GPIO39 = GPIO39
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_IN);    // GPIO39 = input

    // Enable GPIO input on GPIO40
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_40_GPIO40);              // GPIO40 = GPIO40
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);    // GPIO40 = input

    // Enable GPIO input on GPIO57
    GPIO_setPadConfig(57, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_57_GPIO57);              // GPIO57 = GPIO57
    GPIO_setDirectionMode(57, GPIO_DIR_MODE_IN);    // GPIO57 = input

    GPIO_setPinConfig(GPIO_58_CANB_TX);
    GPIO_setPinConfig(GPIO_59_CANB_RX);
}

//
//
//
static void initCPUTimers(void)
{
    // Initialize timer period to maximum
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFFUL);

    // Initialize pre-scale counter to have increments of 1us
    CPUTimer_setPreScaler(CPUTIMER0_BASE, (DEVICE_SYSCLK_FREQ / 1000000UL) - 1U);

    // Make sure timer is stopped
    CPUTimer_stopTimer(CPUTIMER0_BASE);

    // Reload all counter register with period value
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);

    CPUTimer_setEmulationMode(CPUTIMER0_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    // Starts CPU-Timer 0
    CPUTimer_startTimer(CPUTIMER0_BASE);
}

//
//
//
static void factory_defaults_load(void)
{
    dcdc_factory_s.serial_number = 0UL;
    dcdc_factory_s.output_voltage.slope = 1.0f;
    dcdc_factory_s.output_voltage.offset = 0.0f;
    dcdc_factory_s.output_current.slope = 1.0f;
    dcdc_factory_s.output_current.offset = 0.0f;
}

//
//
//
static void configuration_defaults_load(void)
{
    dcdc_configuration_s.output_voltage_setpoint =
            DCDC_OUTPUT_VOLTAGE_SETPOINT_DEFAULT;
}

//
//
//
void dcdc_init(void)
{
    if (HWREG(DEVCFG_BASE + SYSCTL_O_REVID) == 0x0UL)
        while (1); //  Loop forever

    // Initialize device clock and peripherals
    Device_init();
    // All LS, GS, and MSG RAM is initialized with data 0x0 and respective ECC/Parity.
    MemCfg_initSections(MEMCFG_SECT_LSX_ALL
                        | MEMCFG_SECT_GSX_ALL
                        | MEMCFG_SECT_MSGX_ALL);
    // Setup GPIO by disabling pin locks and enabling pullups
    Device_initGPIO();
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    Interrupt_initVectorTable();


    wdog_init();

    initEPWM();
    initADC();
    initCMPSS();
    CLB_init();
    CLBXBAR_init();
    INPUTXBAR_init();
    OUTPUTXBAR_init();
    init_tiles();
    CLB_enable();
    initSCI();
    initSPI();
    setupGPIO();
    initCPUTimers();

    spi_flash_init();

    factory_defaults_load();
    configuration_defaults_load();
    pers_init(&dcdc_factory_s, sizeof(dcdc_factory_s),
              &dcdc_configuration_s, sizeof(dcdc_configuration_s));

    dcdc_sdp_init();
    dcdc_cla_init();
    dcdc_isr_init();
    dcdc_slow_init();

    // Enable sync and clock to PWM
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    EPWM_setGlobalLoadOneShotLatch(EPWM1_BASE);
}
