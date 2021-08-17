///////////////////////////////////////////////////////////////////////////////
//
// CLA initialization file
//
///////////////////////////////////////////////////////////////////////////////


//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "dcdc.h"


// Linker Defined variables
extern uint32_t Cla1ProgRunStart;
extern uint32_t Cla1ProgLoadStart;
extern uint32_t Cla1ProgLoadSize;
extern uint32_t Cla1ConstRunStart;
extern uint32_t Cla1ConstLoadStart;
extern uint32_t Cla1ConstLoadSize;

#pragma DATA_SECTION(dcdc_cla_to_cpu_mem, "Cla1ToCpuMsgRAM");
volatile struct dcdc_cla_to_cpu dcdc_cla_to_cpu_mem;

#pragma DATA_SECTION(dcdc_cpu_to_cla_mem, "CpuToCla1MsgRAM");
volatile struct dcdc_cpu_to_cla dcdc_cpu_to_cla_mem;


//
//
//
static void dcdc_cpu_to_cla_struct_init(void)
{
    dcdc_cpu_to_cla_mem.dcdc_state.cpu = DCDC_STATE_SHUTDOWN;
    dcdc_cpu_to_cla_mem.sync_tick_total_per_period.cpu = 0U;
    dcdc_cpu_to_cla_mem.open_loop_hf_legs_duty.cpu = 0U;

    dcdc_cpu_to_cla_mem.voltage_loop_gain = 1.0f;
    dcdc_cpu_to_cla_mem.current_loop_gain = 1.0f;
}




//
// This function will
// - copy over code and const from flash to CLA program and data ram
//   respectively
// - Initialize the task vectors (MVECTx)
// - setup each task's trigger
// - enable each individual task
// - map program and data spaces to the CLA
// - run any one-time initialization task
// Please note that the CLA can only run code and access data that is in RAM.
// the user must assign constants (tables) to FLASH, and copy them over to
// RAM at run-time. They must be copied to a RAM that lies in the address space
// of the CLA, and prior to giving the CLA control over that space
//
void dcdc_cla_init(void)
{
    dcdc_cpu_to_cla_struct_init();


    // Copy the program and constants from FLASH to RAM before configuring the CLA
    memcpy((uint32_t*)&Cla1ProgRunStart, (uint32_t*)&Cla1ProgLoadStart,
           (uint32_t)&Cla1ProgLoadSize);
    memcpy((uint32_t*)&Cla1ConstRunStart, (uint32_t*)&Cla1ConstLoadStart,
           (uint32_t)&Cla1ConstLoadSize);

    // CLA Program will reside in RAMLS0 and RAMLS1
    // and CLA data in RAMLS2
    MemCfg_setCLAMemType(MEMCFG_SECT_LS0, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS1, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS2, MEMCFG_CLA_MEM_DATA);

    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS0, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS1, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS2, MEMCFG_LSRAMMASTER_CPU_CLA1);

//
// Suppressing #770-D conversion from pointer to smaller integer
// The CLA address range is 16 bits so the addresses passed to the MVECT
// registers will be in the lower 64KW address space. Turn the warning
// back on after the MVECTs are assigned addresses
//
#pragma diag_suppress=770

    // Assign the task vectors and set the triggers for task 1 and 8
    CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_1, (uint16_t)&dcdc_cla_task_1);
    CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_8, (uint16_t)&dcdc_cla_initialization_task);
    CLA_setTriggerSource(CLA_TASK_1, CLA_TRIGGER_EPWM1INT);
    CLA_setTriggerSource(CLA_TASK_8, CLA_TRIGGER_SOFTWARE);

#pragma diag_warning=770

    // Enable Tasks 1 and 8
    CLA_enableTasks(CLA1_BASE, (CLA_TASKFLAG_1 | CLA_TASKFLAG_8));

    // Force task 8, the one time initialization task
    CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_8);
}
