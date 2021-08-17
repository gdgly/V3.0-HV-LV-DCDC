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
#include "spi_flash/spi_flash.h"
#include "pers/pers.h"


#define ADC_ACQUISITION_WINDOW_SIZE                  8U
#define ADC_EOC_TIME                                21U
#define ADC_CONVERSION_COUNT_BEFORE_INTERRUPT_PULSE  2U
#define ADC_SOC_SHIFT   (EPWM1_PERIOD \
    - (ADC_CONVERSION_COUNT_BEFORE_INTERRUPT_PULSE * (ADC_ACQUISITION_WINDOW_SIZE + ADC_EOC_TIME)))

// TODO: Set to proper values
#define DCDC_CMPSS_DAC_VALUE_OCP_POSITIVE_CYCLE  3072U
#define DCDC_CMPSS_DAC_VALUE_OCP_NEGATIVE_CYCLE  1024U


struct dcdc_factory dcdc_factory_s;



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
}

//
// EPWM Initialization
//
static void initEPWM(void)
{
    // Disable sync(Freeze clock to PWM as well)
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);



    // Set up EPWM1
    pwm_basic_init(EPWM1_BASE, EPWM1_PERIOD);
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

    // Enabling EPWM1 interrupt at TBCTR = 0 to trigger ISR and CLA task
    EPWM_setInterruptSource(EPWM1_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM1_BASE);
    EPWM_setInterruptEventCount(EPWM1_BASE, 1U);



    // Set up EPWM2
    pwm_basic_init(EPWM2_BASE, EPWM2_PERIOD);
    EPWM_enablePhaseShiftLoad(EPWM2_BASE);

    // Configuring action-qualifiers for EPWM2 to generate
    // complementary outputs on channel A and B
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    pwm_deadband_config(EPWM2_BASE, EPWM2_DEADBAND);
    pwm_ost_tripzone_config(EPWM2_BASE);
    pwm_cbc_tripzone_config(EPWM2_BASE);



    // Set up EPWM3
    pwm_basic_init(EPWM3_BASE, EPWM3_PERIOD);

    // Configuring action-qualifiers for EPWM3 to generate
    // complementary outputs on channel A and B
    EPWM_setActionQualifierContSWForceShadowMode(EPWM3_BASE, EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
    EPWM_setActionQualifierContSWForceAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
    EPWM_setActionQualifierContSWForceAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_OUTPUT_LOW);

    pwm_ost_tripzone_config(EPWM3_BASE);


    // Set up EPWM4
    pwm_basic_init(EPWM4_BASE, EPWM4_PERIOD);

    // Configuring action-qualifiers for EPWM4 to generate
    // output on channel A
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);



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
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_B, ADC_SOC_SHIFT);
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPB);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);


    // Configure Input Voltage P ADC conversion.
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN1, ADC_ACQUISITION_WINDOW_SIZE);
    // Configure Input Current ADC conversion.
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN0, ADC_ACQUISITION_WINDOW_SIZE);
    // Configure DCDC Heatsink 1 Temperature ADC conversion.
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN3, ADC_ACQUISITION_WINDOW_SIZE);

    // Configure Input Voltage N ADC conversion.
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN1, ADC_ACQUISITION_WINDOW_SIZE);
    // Configure DCDC Heatsink 2 Temperature ADC conversion.
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN2, ADC_ACQUISITION_WINDOW_SIZE);
    // Configure HW Revision ADC conversion.
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN3, ADC_ACQUISITION_WINDOW_SIZE);

    // Configure Bus Voltage ADC conversion.
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN0, ADC_ACQUISITION_WINDOW_SIZE);
    // Configure ADC Reference Voltage ADC conversion.
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN4, ADC_ACQUISITION_WINDOW_SIZE);
    // Configure Ambient Temperature ADC conversion.
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN1, ADC_ACQUISITION_WINDOW_SIZE);
    // Configure LLC Primary Heatsink Temperature ADC conversion.
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN2, ADC_ACQUISITION_WINDOW_SIZE);
}

//
//
//
static void initCMPSS(void)
{
    // Select CMPSS-1 High comparator Positive input 0
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_1, 0);
    // Select CMPSS-1 Low comparator Positive input 0
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_1, 0);

    // Enable CMPSS and configure the negative input signal to come from the DAC
    CMPSS_enableModule(CMPSS1_BASE);
    CMPSS_configHighComparator(CMPSS1_BASE, CMPSS_INSRC_DAC);
    CMPSS_configLowComparator(CMPSS1_BASE, CMPSS_INSRC_DAC
                              | CMPSS_INV_INVERTED);

    // Use VDDA as the reference for the DAC
    CMPSS_configDAC(CMPSS1_BASE, CMPSS_DACREF_VDDA
                    | CMPSS_DACVAL_SYSCLK
                    | CMPSS_DACSRC_SHDW);

    // Configure the output signals. CTRIPH and CTRIPL will be fed by
    // the asynchronous comparator output.
    CMPSS_configOutputsHigh(CMPSS1_BASE, CMPSS_TRIP_ASYNC_COMP);
    CMPSS_configOutputsLow(CMPSS1_BASE, CMPSS_TRIP_ASYNC_COMP);

    // Configure TRIP4 to be CTRIP1H or CTRIP1L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(XBAR_TRIP4, XBAR_EPWM_MUX00_CMPSS1_CTRIPH_OR_L);
    XBAR_enableEPWMMux(XBAR_TRIP4, XBAR_MUX00);

    CMPSS_setDACValueHigh(CMPSS1_BASE, DCDC_CMPSS_DAC_VALUE_OCP_POSITIVE_CYCLE);
    CMPSS_setDACValueLow(CMPSS1_BASE, DCDC_CMPSS_DAC_VALUE_OCP_NEGATIVE_CYCLE);
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

    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, 9600, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
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
static void initEQEP(void)
{
    EQEP_setDecoderConfig(EQEP1_BASE, (EQEP_CONFIG_1X_RESOLUTION |
                                       EQEP_CONFIG_UP_COUNT |
                                       EQEP_CONFIG_NO_SWAP));
    EQEP_setEmulationMode(EQEP1_BASE, EQEP_EMULATIONMODE_RUNFREE);
    // Enable the unit timer, setting the frequency to 100Hz
    EQEP_enableUnitTimer(EQEP1_BASE, (DEVICE_SYSCLK_FREQ / 100));
    EQEP_setLatchMode(EQEP1_BASE, EQEP_LATCH_UNIT_TIME_OUT);
    EQEP_enableModule(EQEP1_BASE);
    EQEP_setCaptureConfig(EQEP1_BASE, EQEP_CAPTURE_CLK_DIV_128,
                          EQEP_UNIT_POS_EVNT_DIV_1);
    EQEP_enableCapture(EQEP1_BASE);
}

//
//
//
static void setupGPIO(void)
{
    GPIO_setPinConfig(GPIO_0_EPWM1_A);      // High-Frequency Leg 1 PWMs
    GPIO_setPinConfig(GPIO_1_EPWM1_B);

    GPIO_setPinConfig(GPIO_2_EPWM2_A);      // High-Frequency Leg 2 PWMs
    GPIO_setPinConfig(GPIO_3_EPWM2_B);

    GPIO_setPinConfig(GPIO_4_EPWM3_A);      // Low-Frequency Leg PWMs
    GPIO_setPinConfig(GPIO_5_EPWM3_B);

    GPIO_setPinConfig(GPIO_6_EPWM4_A);      // Fan control PWM

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

    GPIO_setPinConfig(GPIO_16_EPWM5_A); // Auxiliary power PWM

    // Enable GPIO output on GPIO17
    GPIO_writePin(17, 1);                           // Load output latch
    GPIO_setPinConfig(GPIO_17_GPIO17);              // GPIO17 = GPIO17
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_OUT);   // GPIO17 = output

    GPIO_setPinConfig(GPIO_18_GPIO18_X2);

    // Enable GPIO outputs on GPIO25
    GPIO_writePin(25, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_25_GPIO25);              // GPIO25 = GPIO25
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_OUT);   // GPIO25 = output
    GPIO_setMasterCore(25, GPIO_CORE_CPU1_CLA1);

    // Enable GPIO outputs on GPIO26
    GPIO_writePin(26, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_26_GPIO26);              // GPIO26 = GPIO26
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_OUT);   // GPIO26 = output

    GPIO_setPinConfig(GPIO_28_SCIA_RX);
    GPIO_setPinConfig(GPIO_29_SCIA_TX);

    // SPI Debug
    GPIO_setPinConfig(GPIO_30_SPIB_SIMO);

    // Enable GPIO outputs on GPIO31
    GPIO_writePin(31, 1);                           // Load output latch
    GPIO_setPinConfig(GPIO_31_GPIO31);              // GPIO31 = GPIO31
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_OUT);   // GPIO31 = output

    // Enable GPIO outputs on GPIO33
    GPIO_writePin(33, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_33_GPIO33);              // GPIO33 = GPIO33
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_OUT);   // GPIO33 = output

    // Enable GPIO outputs on GPIO34
    GPIO_writePin(34, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_34_GPIO34);              // GPIO34 = GPIO34
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);   // GPIO34 = output

    GPIO_setPinConfig(GPIO_35_TDI);
    GPIO_setPinConfig(GPIO_37_TDO);

    GPIO_setPinConfig(GPIO_56_EQEP1_A);
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
    dcdc_factory_s.input_voltage.slope = 1.0f;
    dcdc_factory_s.input_voltage.offset = 0.0f;
    dcdc_factory_s.input_current.slope = 1.0f;
    dcdc_factory_s.input_current.offset = 0.0f;
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
    initSCI();
    initSPI();
    initEQEP();
    setupGPIO();
    initCPUTimers();

    spi_flash_init();

    factory_defaults_load();
    pers_init(&dcdc_factory_s, sizeof(dcdc_factory_s),
              NULL, 0U);

    dcdc_sdp_init();
    dcdc_cla_init();
    dcdc_isr_init();
    dcdc_slow_init();

    // Enable sync and clock to PWM
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}
