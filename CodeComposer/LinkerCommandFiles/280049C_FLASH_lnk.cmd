
MEMORY
{
PAGE 0 :
   /* BEGIN is used for the "boot to Flash" bootloader mode   */

   BEGIN            : origin = 0x080000, length = 0x000002
   RESET            : origin = 0x3FFFC0, length = 0x000002

   /* Flash sectors */
   FLASH_BANK0      : origin = 0x080002, length = 0x00FFFE  // on-chip Flash
   FLASH_BANK1      : origin = 0x090000, length = 0x00FFF0  // on-chip Flash
//   FLASH_BANK1_RSVD : origin = 0x09FFF0, length = 0x000010  // Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory"

PAGE 1 :

   BOOT_RSVD        : origin = 0x000002, length = 0x0000F1  // Part of M0, BOOT rom will use this for stack
   RAMM_STACK       : origin = 0x0000F4, length = 0x000200
   RAMM0_1          : origin = 0x0002F4, length = 0x000504  // on-chip RAM block M1
//   RAMM1_RSVD       : origin = 0x0007F8, length = 0x000008  // Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory"

   RAMLS0_1         : origin = 0x008000, length = 0x001000  // CLA program RAM
   RAMLS2           : origin = 0x009000, length = 0x000800  // CLA data RAM
   RAMLS3           : origin = 0x009800, length = 0x000800  // RAM functions
   RAMLS4           : origin = 0x00A000, length = 0x000800  // RAM functions
   RAMLS5_7         : origin = 0x00A800, length = 0x001800

   RAMGSx           : origin = 0x00C000, length = 0x007FF8
//   RAMGSx_RSVD : origin = 0x013FF8, length = 0x000008     // Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory"

   CLA1_MSGRAMLOW   : origin = 0x001480, length = 0x000080
   CLA1_MSGRAMHIGH  : origin = 0x001500, length = 0x000080
}


SECTIONS
{
   codestart        : > BEGIN,         PAGE = 0, ALIGN(4)
   .text            : >> FLASH_BANK0 | FLASH_BANK1, PAGE = 0, ALIGN(4)
   .cinit           : > FLASH_BANK0 | FLASH_BANK1,  PAGE = 0, ALIGN(4)
   .pinit           : > FLASH_BANK0 | FLASH_BANK1,  PAGE = 0, ALIGN(4)
   .switch          : > FLASH_BANK0 | FLASH_BANK1,  PAGE = 0, ALIGN(4)
   .reset           : > RESET,         PAGE = 0, TYPE = DSECT /* not used, */

   .stack           : > RAMM_STACK, PAGE = 1
   .ebss            : > RAMLS5_7 | RAMM0_1, PAGE = 1
   .esysmem         : > RAMLS5_7 | RAMM0_1, PAGE = 1
   .econst          : > FLASH_BANK0 | FLASH_BANK1,  PAGE = 0, ALIGN(4)

   ramgs            : > RAMGSx,        PAGE = 1

    /* CLA specific sections */
    Cla1Prog        : LOAD = FLASH_BANK0 | FLASH_BANK1,  PAGE = 0
                      RUN = RAMLS0_1, PAGE = 1
                      LOAD_START(_Cla1ProgLoadStart),
                      RUN_START(_Cla1ProgRunStart),
                      LOAD_SIZE(_Cla1ProgLoadSize),
                      ALIGN(4)

   Cla1ToCpuMsgRAM  : > CLA1_MSGRAMLOW,   PAGE = 1
   CpuToCla1MsgRAM  : > CLA1_MSGRAMHIGH,  PAGE = 1

   .scratchpad      : > RAMLS2,           PAGE = 1
   .bss_cla         : > RAMLS2,           PAGE = 1

   Cla1DataRam      : > RAMLS2,           PAGE = 1
   cla_shared       : > RAMLS2,           PAGE = 1

   .const_cla       : LOAD = FLASH_BANK0 | FLASH_BANK1,  PAGE = 0
                      RUN = RAMLS2,       PAGE = 1
                      RUN_START(_Cla1ConstRunStart),
                      LOAD_START(_Cla1ConstLoadStart),
                      LOAD_SIZE(_Cla1ConstLoadSize),
                      ALIGN(4)

    .TI.ramfunc  : {} LOAD = FLASH_BANK0 | FLASH_BANK1,  PAGE = 0
                      RUN = RAMLS3 | RAMLS4, PAGE = 1
                      LOAD_START(_RamfuncsLoadStart),
                      LOAD_SIZE(_RamfuncsLoadSize),
                      LOAD_END(_RamfuncsLoadEnd),
                      RUN_START(_RamfuncsRunStart),
                      RUN_SIZE(_RamfuncsRunSize),
                      RUN_END(_RamfuncsRunEnd),
                      ALIGN(4)
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
