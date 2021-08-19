/*
 *  ======== clb.c ========
 *  DO NOT EDIT - This file is generated by the SysConfig tool.
 */

#include "driverlib.h"
#include "device.h"
#include "clb_config.h"
#include "clb.h"


uint16_t TILE0HLCInstr[CLB_NUM_HLC_INSTR + 1] =
{
    TILE0_HLCINSTR_0,
    TILE0_HLCINSTR_1,
    TILE0_HLCINSTR_2,
    TILE0_HLCINSTR_3,
    TILE0_HLCINSTR_4,
    TILE0_HLCINSTR_5,
    TILE0_HLCINSTR_6,
    TILE0_HLCINSTR_7,
    TILE0_HLCINSTR_8,
    TILE0_HLCINSTR_9,
    TILE0_HLCINSTR_10,
    TILE0_HLCINSTR_11,
    TILE0_HLCINSTR_12,
    TILE0_HLCINSTR_13,
    TILE0_HLCINSTR_14,
    TILE0_HLCINSTR_15,
    TILE0_HLCINSTR_16,
    TILE0_HLCINSTR_17,
    TILE0_HLCINSTR_18,
    TILE0_HLCINSTR_19,
    TILE0_HLCINSTR_20,
    TILE0_HLCINSTR_21,
    TILE0_HLCINSTR_22,
    TILE0_HLCINSTR_23,
    TILE0_HLCINSTR_24,
    TILE0_HLCINSTR_25,
    TILE0_HLCINSTR_26,
    TILE0_HLCINSTR_27,
    TILE0_HLCINSTR_28,
    TILE0_HLCINSTR_29,
    TILE0_HLCINSTR_30,
    TILE0_HLCINSTR_31
};


uint16_t TILE1HLCInstr[CLB_NUM_HLC_INSTR + 1] =
{
    TILE1_HLCINSTR_0,
    TILE1_HLCINSTR_1,
    TILE1_HLCINSTR_2,
    TILE1_HLCINSTR_3,
    TILE1_HLCINSTR_4,
    TILE1_HLCINSTR_5,
    TILE1_HLCINSTR_6,
    TILE1_HLCINSTR_7,
    TILE1_HLCINSTR_8,
    TILE1_HLCINSTR_9,
    TILE1_HLCINSTR_10,
    TILE1_HLCINSTR_11,
    TILE1_HLCINSTR_12,
    TILE1_HLCINSTR_13,
    TILE1_HLCINSTR_14,
    TILE1_HLCINSTR_15,
    TILE1_HLCINSTR_16,
    TILE1_HLCINSTR_17,
    TILE1_HLCINSTR_18,
    TILE1_HLCINSTR_19,
    TILE1_HLCINSTR_20,
    TILE1_HLCINSTR_21,
    TILE1_HLCINSTR_22,
    TILE1_HLCINSTR_23,
    TILE1_HLCINSTR_24,
    TILE1_HLCINSTR_25,
    TILE1_HLCINSTR_26,
    TILE1_HLCINSTR_27,
    TILE1_HLCINSTR_28,
    TILE1_HLCINSTR_29,
    TILE1_HLCINSTR_30,
    TILE1_HLCINSTR_31
};


uint16_t TILE2HLCInstr[CLB_NUM_HLC_INSTR + 1] =
{
    TILE2_HLCINSTR_0,
    TILE2_HLCINSTR_1,
    TILE2_HLCINSTR_2,
    TILE2_HLCINSTR_3,
    TILE2_HLCINSTR_4,
    TILE2_HLCINSTR_5,
    TILE2_HLCINSTR_6,
    TILE2_HLCINSTR_7,
    TILE2_HLCINSTR_8,
    TILE2_HLCINSTR_9,
    TILE2_HLCINSTR_10,
    TILE2_HLCINSTR_11,
    TILE2_HLCINSTR_12,
    TILE2_HLCINSTR_13,
    TILE2_HLCINSTR_14,
    TILE2_HLCINSTR_15,
    TILE2_HLCINSTR_16,
    TILE2_HLCINSTR_17,
    TILE2_HLCINSTR_18,
    TILE2_HLCINSTR_19,
    TILE2_HLCINSTR_20,
    TILE2_HLCINSTR_21,
    TILE2_HLCINSTR_22,
    TILE2_HLCINSTR_23,
    TILE2_HLCINSTR_24,
    TILE2_HLCINSTR_25,
    TILE2_HLCINSTR_26,
    TILE2_HLCINSTR_27,
    TILE2_HLCINSTR_28,
    TILE2_HLCINSTR_29,
    TILE2_HLCINSTR_30,
    TILE2_HLCINSTR_31
};


uint16_t TILE3HLCInstr[CLB_NUM_HLC_INSTR + 1] =
{
    TILE3_HLCINSTR_0,
    TILE3_HLCINSTR_1,
    TILE3_HLCINSTR_2,
    TILE3_HLCINSTR_3,
    TILE3_HLCINSTR_4,
    TILE3_HLCINSTR_5,
    TILE3_HLCINSTR_6,
    TILE3_HLCINSTR_7,
    TILE3_HLCINSTR_8,
    TILE3_HLCINSTR_9,
    TILE3_HLCINSTR_10,
    TILE3_HLCINSTR_11,
    TILE3_HLCINSTR_12,
    TILE3_HLCINSTR_13,
    TILE3_HLCINSTR_14,
    TILE3_HLCINSTR_15,
    TILE3_HLCINSTR_16,
    TILE3_HLCINSTR_17,
    TILE3_HLCINSTR_18,
    TILE3_HLCINSTR_19,
    TILE3_HLCINSTR_20,
    TILE3_HLCINSTR_21,
    TILE3_HLCINSTR_22,
    TILE3_HLCINSTR_23,
    TILE3_HLCINSTR_24,
    TILE3_HLCINSTR_25,
    TILE3_HLCINSTR_26,
    TILE3_HLCINSTR_27,
    TILE3_HLCINSTR_28,
    TILE3_HLCINSTR_29,
    TILE3_HLCINSTR_30,
    TILE3_HLCINSTR_31
};



void initTILE0(uint32_t base)
{
    uint16_t i;
    // Output LUT
    CLB_configOutputLUT(base, CLB_OUT0, TILE0_CFG_OUTLUT_0);
    CLB_configOutputLUT(base, CLB_OUT1, TILE0_CFG_OUTLUT_1);
    CLB_configOutputLUT(base, CLB_OUT2, TILE0_CFG_OUTLUT_2);
    CLB_configOutputLUT(base, CLB_OUT3, TILE0_CFG_OUTLUT_3);
    CLB_configOutputLUT(base, CLB_OUT4, TILE0_CFG_OUTLUT_4);
    CLB_configOutputLUT(base, CLB_OUT5, TILE0_CFG_OUTLUT_5);
    CLB_configOutputLUT(base, CLB_OUT6, TILE0_CFG_OUTLUT_6);
    CLB_configOutputLUT(base, CLB_OUT7, TILE0_CFG_OUTLUT_7);
    // AOC
    CLB_configAOC(base, CLB_AOC0, TILE0_OUTPUT_COND_CTR_0);
    CLB_configAOC(base, CLB_AOC1, TILE0_OUTPUT_COND_CTR_1);
    CLB_configAOC(base, CLB_AOC2, TILE0_OUTPUT_COND_CTR_2);
    CLB_configAOC(base, CLB_AOC3, TILE0_OUTPUT_COND_CTR_3);
    CLB_configAOC(base, CLB_AOC4, TILE0_OUTPUT_COND_CTR_4);
    CLB_configAOC(base, CLB_AOC5, TILE0_OUTPUT_COND_CTR_5);
    CLB_configAOC(base, CLB_AOC6, TILE0_OUTPUT_COND_CTR_6);
    CLB_configAOC(base, CLB_AOC7, TILE0_OUTPUT_COND_CTR_7);

    // LUT4
    CLB_selectLUT4Inputs(base, TILE0_CFG_LUT4_IN0, TILE0_CFG_LUT4_IN1, TILE0_CFG_LUT4_IN2, TILE0_CFG_LUT4_IN3);
    CLB_configLUT4Function(base, TILE0_CFG_LUT4_FN10, TILE0_CFG_LUT4_FN2);

    // FSM
    CLB_selectFSMInputs(base, TILE0_CFG_FSM_EXT_IN0, TILE0_CFG_FSM_EXT_IN1, TILE0_CFG_FSM_EXTRA_IN0, TILE0_CFG_FSM_EXTRA_IN1);
    CLB_configFSMNextState(base, TILE0_CFG_FSM_NEXT_STATE_0, TILE0_CFG_FSM_NEXT_STATE_1, TILE0_CFG_FSM_NEXT_STATE_2);
    CLB_configFSMLUTFunction(base, TILE0_CFG_FSM_LUT_FN10, TILE0_CFG_FSM_LUT_FN2);

    // Counters
    CLB_selectCounterInputs(base, TILE0_CFG_COUNTER_RESET, TILE0_CFG_COUNTER_EVENT, TILE0_CFG_COUNTER_MODE_0, TILE0_CFG_COUNTER_MODE_1);
    CLB_configMiscCtrlModes(base, TILE0_CFG_MISC_CONTROL);
    CLB_configCounterLoadMatch(base, CLB_CTR0, TILE0_COUNTER_0_LOAD_VAL, TILE0_COUNTER_0_MATCH1_VAL, TILE0_COUNTER_0_MATCH2_VAL);
    CLB_configCounterLoadMatch(base, CLB_CTR1, TILE0_COUNTER_1_LOAD_VAL, TILE0_COUNTER_1_MATCH1_VAL, TILE0_COUNTER_1_MATCH2_VAL);
    CLB_configCounterLoadMatch(base, CLB_CTR2, TILE0_COUNTER_2_LOAD_VAL, TILE0_COUNTER_2_MATCH1_VAL, TILE0_COUNTER_2_MATCH2_VAL);
    CLB_configCounterTapSelects(base, TILE0_CFG_TAP_SEL);

    // HLC
    CLB_configHLCEventSelect(base, TILE0_HLC_EVENT_SEL);
    CLB_setHLCRegisters(base, TILE0_HLC_R0_INIT, TILE0_HLC_R1_INIT, TILE0_HLC_R2_INIT, TILE0_HLC_R3_INIT);

    for(i = 0; i <= CLB_NUM_HLC_INSTR; i++)
    {
        CLB_programHLCInstruction(base, i, TILE0HLCInstr[i]);
    }
}


void initTILE1(uint32_t base)
{
    uint16_t i;
    // Output LUT
    CLB_configOutputLUT(base, CLB_OUT0, TILE1_CFG_OUTLUT_0);
    CLB_configOutputLUT(base, CLB_OUT1, TILE1_CFG_OUTLUT_1);
    CLB_configOutputLUT(base, CLB_OUT2, TILE1_CFG_OUTLUT_2);
    CLB_configOutputLUT(base, CLB_OUT3, TILE1_CFG_OUTLUT_3);
    CLB_configOutputLUT(base, CLB_OUT4, TILE1_CFG_OUTLUT_4);
    CLB_configOutputLUT(base, CLB_OUT5, TILE1_CFG_OUTLUT_5);
    CLB_configOutputLUT(base, CLB_OUT6, TILE1_CFG_OUTLUT_6);
    CLB_configOutputLUT(base, CLB_OUT7, TILE1_CFG_OUTLUT_7);
    // AOC
    CLB_configAOC(base, CLB_AOC0, TILE1_OUTPUT_COND_CTR_0);
    CLB_configAOC(base, CLB_AOC1, TILE1_OUTPUT_COND_CTR_1);
    CLB_configAOC(base, CLB_AOC2, TILE1_OUTPUT_COND_CTR_2);
    CLB_configAOC(base, CLB_AOC3, TILE1_OUTPUT_COND_CTR_3);
    CLB_configAOC(base, CLB_AOC4, TILE1_OUTPUT_COND_CTR_4);
    CLB_configAOC(base, CLB_AOC5, TILE1_OUTPUT_COND_CTR_5);
    CLB_configAOC(base, CLB_AOC6, TILE1_OUTPUT_COND_CTR_6);
    CLB_configAOC(base, CLB_AOC7, TILE1_OUTPUT_COND_CTR_7);

    // LUT4
    CLB_selectLUT4Inputs(base, TILE1_CFG_LUT4_IN0, TILE1_CFG_LUT4_IN1, TILE1_CFG_LUT4_IN2, TILE1_CFG_LUT4_IN3);
    CLB_configLUT4Function(base, TILE1_CFG_LUT4_FN10, TILE1_CFG_LUT4_FN2);

    // FSM
    CLB_selectFSMInputs(base, TILE1_CFG_FSM_EXT_IN0, TILE1_CFG_FSM_EXT_IN1, TILE1_CFG_FSM_EXTRA_IN0, TILE1_CFG_FSM_EXTRA_IN1);
    CLB_configFSMNextState(base, TILE1_CFG_FSM_NEXT_STATE_0, TILE1_CFG_FSM_NEXT_STATE_1, TILE1_CFG_FSM_NEXT_STATE_2);
    CLB_configFSMLUTFunction(base, TILE1_CFG_FSM_LUT_FN10, TILE1_CFG_FSM_LUT_FN2);

    // Counters
    CLB_selectCounterInputs(base, TILE1_CFG_COUNTER_RESET, TILE1_CFG_COUNTER_EVENT, TILE1_CFG_COUNTER_MODE_0, TILE1_CFG_COUNTER_MODE_1);
    CLB_configMiscCtrlModes(base, TILE1_CFG_MISC_CONTROL);
    CLB_configCounterLoadMatch(base, CLB_CTR0, TILE1_COUNTER_0_LOAD_VAL, TILE1_COUNTER_0_MATCH1_VAL, TILE1_COUNTER_0_MATCH2_VAL);
    CLB_configCounterLoadMatch(base, CLB_CTR1, TILE1_COUNTER_1_LOAD_VAL, TILE1_COUNTER_1_MATCH1_VAL, TILE1_COUNTER_1_MATCH2_VAL);
    CLB_configCounterLoadMatch(base, CLB_CTR2, TILE1_COUNTER_2_LOAD_VAL, TILE1_COUNTER_2_MATCH1_VAL, TILE1_COUNTER_2_MATCH2_VAL);
    CLB_configCounterTapSelects(base, TILE1_CFG_TAP_SEL);

    // HLC
    CLB_configHLCEventSelect(base, TILE1_HLC_EVENT_SEL);
    CLB_setHLCRegisters(base, TILE1_HLC_R0_INIT, TILE1_HLC_R1_INIT, TILE1_HLC_R2_INIT, TILE1_HLC_R3_INIT);

    for(i = 0; i <= CLB_NUM_HLC_INSTR; i++)
    {
        CLB_programHLCInstruction(base, i, TILE1HLCInstr[i]);
    }
}


void initTILE2(uint32_t base)
{
    uint16_t i;
    // Output LUT
    CLB_configOutputLUT(base, CLB_OUT0, TILE2_CFG_OUTLUT_0);
    CLB_configOutputLUT(base, CLB_OUT1, TILE2_CFG_OUTLUT_1);
    CLB_configOutputLUT(base, CLB_OUT2, TILE2_CFG_OUTLUT_2);
    CLB_configOutputLUT(base, CLB_OUT3, TILE2_CFG_OUTLUT_3);
    CLB_configOutputLUT(base, CLB_OUT4, TILE2_CFG_OUTLUT_4);
    CLB_configOutputLUT(base, CLB_OUT5, TILE2_CFG_OUTLUT_5);
    CLB_configOutputLUT(base, CLB_OUT6, TILE2_CFG_OUTLUT_6);
    CLB_configOutputLUT(base, CLB_OUT7, TILE2_CFG_OUTLUT_7);
    // AOC
    CLB_configAOC(base, CLB_AOC0, TILE2_OUTPUT_COND_CTR_0);
    CLB_configAOC(base, CLB_AOC1, TILE2_OUTPUT_COND_CTR_1);
    CLB_configAOC(base, CLB_AOC2, TILE2_OUTPUT_COND_CTR_2);
    CLB_configAOC(base, CLB_AOC3, TILE2_OUTPUT_COND_CTR_3);
    CLB_configAOC(base, CLB_AOC4, TILE2_OUTPUT_COND_CTR_4);
    CLB_configAOC(base, CLB_AOC5, TILE2_OUTPUT_COND_CTR_5);
    CLB_configAOC(base, CLB_AOC6, TILE2_OUTPUT_COND_CTR_6);
    CLB_configAOC(base, CLB_AOC7, TILE2_OUTPUT_COND_CTR_7);

    // LUT4
    CLB_selectLUT4Inputs(base, TILE2_CFG_LUT4_IN0, TILE2_CFG_LUT4_IN1, TILE2_CFG_LUT4_IN2, TILE2_CFG_LUT4_IN3);
    CLB_configLUT4Function(base, TILE2_CFG_LUT4_FN10, TILE2_CFG_LUT4_FN2);

    // FSM
    CLB_selectFSMInputs(base, TILE2_CFG_FSM_EXT_IN0, TILE2_CFG_FSM_EXT_IN1, TILE2_CFG_FSM_EXTRA_IN0, TILE2_CFG_FSM_EXTRA_IN1);
    CLB_configFSMNextState(base, TILE2_CFG_FSM_NEXT_STATE_0, TILE2_CFG_FSM_NEXT_STATE_1, TILE2_CFG_FSM_NEXT_STATE_2);
    CLB_configFSMLUTFunction(base, TILE2_CFG_FSM_LUT_FN10, TILE2_CFG_FSM_LUT_FN2);

    // Counters
    CLB_selectCounterInputs(base, TILE2_CFG_COUNTER_RESET, TILE2_CFG_COUNTER_EVENT, TILE2_CFG_COUNTER_MODE_0, TILE2_CFG_COUNTER_MODE_1);
    CLB_configMiscCtrlModes(base, TILE2_CFG_MISC_CONTROL);
    CLB_configCounterLoadMatch(base, CLB_CTR0, TILE2_COUNTER_0_LOAD_VAL, TILE2_COUNTER_0_MATCH1_VAL, TILE2_COUNTER_0_MATCH2_VAL);
    CLB_configCounterLoadMatch(base, CLB_CTR1, TILE2_COUNTER_1_LOAD_VAL, TILE2_COUNTER_1_MATCH1_VAL, TILE2_COUNTER_1_MATCH2_VAL);
    CLB_configCounterLoadMatch(base, CLB_CTR2, TILE2_COUNTER_2_LOAD_VAL, TILE2_COUNTER_2_MATCH1_VAL, TILE2_COUNTER_2_MATCH2_VAL);
    CLB_configCounterTapSelects(base, TILE2_CFG_TAP_SEL);

    // HLC
    CLB_configHLCEventSelect(base, TILE2_HLC_EVENT_SEL);
    CLB_setHLCRegisters(base, TILE2_HLC_R0_INIT, TILE2_HLC_R1_INIT, TILE2_HLC_R2_INIT, TILE2_HLC_R3_INIT);

    for(i = 0; i <= CLB_NUM_HLC_INSTR; i++)
    {
        CLB_programHLCInstruction(base, i, TILE2HLCInstr[i]);
    }
}


void initTILE3(uint32_t base)
{
    uint16_t i;
    // Output LUT
    CLB_configOutputLUT(base, CLB_OUT0, TILE3_CFG_OUTLUT_0);
    CLB_configOutputLUT(base, CLB_OUT1, TILE3_CFG_OUTLUT_1);
    CLB_configOutputLUT(base, CLB_OUT2, TILE3_CFG_OUTLUT_2);
    CLB_configOutputLUT(base, CLB_OUT3, TILE3_CFG_OUTLUT_3);
    CLB_configOutputLUT(base, CLB_OUT4, TILE3_CFG_OUTLUT_4);
    CLB_configOutputLUT(base, CLB_OUT5, TILE3_CFG_OUTLUT_5);
    CLB_configOutputLUT(base, CLB_OUT6, TILE3_CFG_OUTLUT_6);
    CLB_configOutputLUT(base, CLB_OUT7, TILE3_CFG_OUTLUT_7);
    // AOC
    CLB_configAOC(base, CLB_AOC0, TILE3_OUTPUT_COND_CTR_0);
    CLB_configAOC(base, CLB_AOC1, TILE3_OUTPUT_COND_CTR_1);
    CLB_configAOC(base, CLB_AOC2, TILE3_OUTPUT_COND_CTR_2);
    CLB_configAOC(base, CLB_AOC3, TILE3_OUTPUT_COND_CTR_3);
    CLB_configAOC(base, CLB_AOC4, TILE3_OUTPUT_COND_CTR_4);
    CLB_configAOC(base, CLB_AOC5, TILE3_OUTPUT_COND_CTR_5);
    CLB_configAOC(base, CLB_AOC6, TILE3_OUTPUT_COND_CTR_6);
    CLB_configAOC(base, CLB_AOC7, TILE3_OUTPUT_COND_CTR_7);

    // LUT4
    CLB_selectLUT4Inputs(base, TILE3_CFG_LUT4_IN0, TILE3_CFG_LUT4_IN1, TILE3_CFG_LUT4_IN2, TILE3_CFG_LUT4_IN3);
    CLB_configLUT4Function(base, TILE3_CFG_LUT4_FN10, TILE3_CFG_LUT4_FN2);

    // FSM
    CLB_selectFSMInputs(base, TILE3_CFG_FSM_EXT_IN0, TILE3_CFG_FSM_EXT_IN1, TILE3_CFG_FSM_EXTRA_IN0, TILE3_CFG_FSM_EXTRA_IN1);
    CLB_configFSMNextState(base, TILE3_CFG_FSM_NEXT_STATE_0, TILE3_CFG_FSM_NEXT_STATE_1, TILE3_CFG_FSM_NEXT_STATE_2);
    CLB_configFSMLUTFunction(base, TILE3_CFG_FSM_LUT_FN10, TILE3_CFG_FSM_LUT_FN2);

    // Counters
    CLB_selectCounterInputs(base, TILE3_CFG_COUNTER_RESET, TILE3_CFG_COUNTER_EVENT, TILE3_CFG_COUNTER_MODE_0, TILE3_CFG_COUNTER_MODE_1);
    CLB_configMiscCtrlModes(base, TILE3_CFG_MISC_CONTROL);
    CLB_configCounterLoadMatch(base, CLB_CTR0, TILE3_COUNTER_0_LOAD_VAL, TILE3_COUNTER_0_MATCH1_VAL, TILE3_COUNTER_0_MATCH2_VAL);
    CLB_configCounterLoadMatch(base, CLB_CTR1, TILE3_COUNTER_1_LOAD_VAL, TILE3_COUNTER_1_MATCH1_VAL, TILE3_COUNTER_1_MATCH2_VAL);
    CLB_configCounterLoadMatch(base, CLB_CTR2, TILE3_COUNTER_2_LOAD_VAL, TILE3_COUNTER_2_MATCH1_VAL, TILE3_COUNTER_2_MATCH2_VAL);
    CLB_configCounterTapSelects(base, TILE3_CFG_TAP_SEL);

    // HLC
    CLB_configHLCEventSelect(base, TILE3_HLC_EVENT_SEL);
    CLB_setHLCRegisters(base, TILE3_HLC_R0_INIT, TILE3_HLC_R1_INIT, TILE3_HLC_R2_INIT, TILE3_HLC_R3_INIT);

    for(i = 0; i <= CLB_NUM_HLC_INSTR; i++)
    {
        CLB_programHLCInstruction(base, i, TILE3HLCInstr[i]);
    }
}



