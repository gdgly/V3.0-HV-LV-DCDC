/*
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "board.h"

void Board_init()
{
	EALLOW;

	PinMux_init();
	CLB_init();
	CLBXBAR_init();
	GPIO_init();
	INPUTXBAR_init();
	OUTPUTXBAR_init();

	EDIS;
}

void PinMux_init()
{
	//GPIO57 -> myGPIO0 Pinmux
	GPIO_setPinConfig(GPIO_57_GPIO57);
	//GPIO22_VFBSW -> myGPIO1 Pinmux
	GPIO_setPinConfig(GPIO_22_GPIO22);
	//GPIO40 -> myGPIO2 Pinmux
	GPIO_setPinConfig(GPIO_40_GPIO40);
	//GPIO39 -> myGPIO3 Pinmux
	GPIO_setPinConfig(GPIO_39_GPIO39);
	//OUTPUTXBAR3 -> myOUTPUTXBAR0 Pinmux
	GPIO_setPinConfig(GPIO_4_OUTPUTXBAR3);
	//OUTPUTXBAR4 -> myOUTPUTXBAR1 Pinmux
	GPIO_setPinConfig(GPIO_6_OUTPUTXBAR4);
	//OUTPUTXBAR5 -> myOUTPUTXBAR2 Pinmux
	GPIO_setPinConfig(GPIO_7_OUTPUTXBAR5);
	//OUTPUTXBAR8 -> myOUTPUTXBAR3 Pinmux
	GPIO_setPinConfig(GPIO_17_OUTPUTXBAR8);

}

void CLB_init(){
	//myCLB1 initialization
	CLB_setOutputMask(myCLB1_BASE,
				(0 << 0), true);
	//myCLB1 CLB_IN0 initialization
	CLB_configLocalInputMux(myCLB1_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(myCLB1_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_CLB_AUXSIG0);
	CLB_configGPInputMux(myCLB1_BASE, CLB_IN0, CLB_GP_IN_MUX_EXTERNAL);
	
	CLB_selectInputFilter(myCLB1_BASE, CLB_IN0, CLB_FILTER_NONE);

	//myCLB1 CLB_IN1 initialization
	CLB_configLocalInputMux(myCLB1_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(myCLB1_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_CLB_AUXSIG1);
	CLB_configGPInputMux(myCLB1_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
	
	CLB_selectInputFilter(myCLB1_BASE, CLB_IN1, CLB_FILTER_NONE);

	CLB_setGPREG(myCLB1_BASE,0);
	CLB_disableCLB(myCLB1_BASE);
	//myCLB2 initialization
	CLB_setOutputMask(myCLB2_BASE,
				(0 << 0), true);
	//myCLB2 CLB_IN0 initialization
	CLB_configLocalInputMux(myCLB2_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(myCLB2_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_CLB_AUXSIG2);
	CLB_configGPInputMux(myCLB2_BASE, CLB_IN0, CLB_GP_IN_MUX_EXTERNAL);
	
	CLB_selectInputFilter(myCLB2_BASE, CLB_IN0, CLB_FILTER_NONE);

	//myCLB2 CLB_IN1 initialization
	CLB_configLocalInputMux(myCLB2_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(myCLB2_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_CLB_AUXSIG3);
	CLB_configGPInputMux(myCLB2_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
	
	CLB_selectInputFilter(myCLB2_BASE, CLB_IN1, CLB_FILTER_NONE);

	CLB_setGPREG(myCLB2_BASE,0);
	CLB_disableCLB(myCLB2_BASE);
	//myCLB3 initialization
	CLB_setOutputMask(myCLB3_BASE,
				(0 << 0), true);
	//myCLB3 CLB_IN0 initialization
	CLB_configLocalInputMux(myCLB3_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(myCLB3_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_CLB_AUXSIG4);
	CLB_configGPInputMux(myCLB3_BASE, CLB_IN0, CLB_GP_IN_MUX_EXTERNAL);
	
	CLB_selectInputFilter(myCLB3_BASE, CLB_IN0, CLB_FILTER_NONE);

	//myCLB3 CLB_IN1 initialization
	CLB_configLocalInputMux(myCLB3_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(myCLB3_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_CLB_AUXSIG5);
	CLB_configGPInputMux(myCLB3_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
	
	CLB_selectInputFilter(myCLB3_BASE, CLB_IN1, CLB_FILTER_NONE);

	CLB_setGPREG(myCLB3_BASE,0);
	CLB_disableCLB(myCLB3_BASE);
	//myCLB4 initialization
	CLB_setOutputMask(myCLB4_BASE,
				(0 << 0), true);
	//myCLB4 CLB_IN0 initialization
	CLB_configLocalInputMux(myCLB4_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(myCLB4_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_CLB_AUXSIG6);
	CLB_configGPInputMux(myCLB4_BASE, CLB_IN0, CLB_GP_IN_MUX_EXTERNAL);
	
	CLB_selectInputFilter(myCLB4_BASE, CLB_IN0, CLB_FILTER_NONE);

	//myCLB4 CLB_IN1 initialization
	CLB_configLocalInputMux(myCLB4_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(myCLB4_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_CLB_AUXSIG7);
	CLB_configGPInputMux(myCLB4_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
	
	CLB_selectInputFilter(myCLB4_BASE, CLB_IN1, CLB_FILTER_NONE);

	CLB_setGPREG(myCLB4_BASE,0);
	CLB_disableCLB(myCLB4_BASE);
}

void CLBXBAR_init(){
	//myCLBXBAR0 initialization
		
	XBAR_setCLBMuxConfig(XBAR_AUXSIG0, XBAR_CLB_MUX00_CMPSS1_CTRIPH);
	XBAR_enableCLBMux(XBAR_AUXSIG0, XBAR_MUX00);

	//myCLBXBAR1 initialization
		
	XBAR_setCLBMuxConfig(XBAR_AUXSIG1, XBAR_CLB_MUX01_INPUTXBAR1);
	XBAR_enableCLBMux(XBAR_AUXSIG1, XBAR_MUX01);

	//myCLBXBAR2 initialization
		
	XBAR_setCLBMuxConfig(XBAR_AUXSIG2, XBAR_CLB_MUX02_CMPSS2_CTRIPH);
	XBAR_enableCLBMux(XBAR_AUXSIG2, XBAR_MUX02);

	//myCLBXBAR3 initialization
		
	XBAR_setCLBMuxConfig(XBAR_AUXSIG3, XBAR_CLB_MUX03_INPUTXBAR2);
	XBAR_enableCLBMux(XBAR_AUXSIG3, XBAR_MUX03);

	//myCLBXBAR4 initialization
		
	XBAR_setCLBMuxConfig(XBAR_AUXSIG4, XBAR_CLB_MUX08_CMPSS5_CTRIPH);
	XBAR_enableCLBMux(XBAR_AUXSIG4, XBAR_MUX08);

	//myCLBXBAR5 initialization
		
	XBAR_setCLBMuxConfig(XBAR_AUXSIG5, XBAR_CLB_MUX09_INPUTXBAR5);
	XBAR_enableCLBMux(XBAR_AUXSIG5, XBAR_MUX09);

	//myCLBXBAR6 initialization
		
	XBAR_setCLBMuxConfig(XBAR_AUXSIG6, XBAR_CLB_MUX10_CMPSS6_CTRIPH);
	XBAR_enableCLBMux(XBAR_AUXSIG6, XBAR_MUX10);

	//myCLBXBAR7 initialization
		
	XBAR_setCLBMuxConfig(XBAR_AUXSIG7, XBAR_CLB_MUX11_INPUTXBAR6);
	XBAR_enableCLBMux(XBAR_AUXSIG7, XBAR_MUX11);

}
void GPIO_init(){
		
	//myGPIO0 initialization
	GPIO_setDirectionMode(myGPIO0, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO0, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO0, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO0, GPIO_QUAL_SYNC);
		
	//myGPIO1 initialization
	GPIO_setDirectionMode(myGPIO1, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO1, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO1, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO1, GPIO_QUAL_SYNC);
		
	//myGPIO2 initialization
	GPIO_setDirectionMode(myGPIO2, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO2, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO2, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO2, GPIO_QUAL_SYNC);
		
	//myGPIO3 initialization
	GPIO_setDirectionMode(myGPIO3, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO3, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO3, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO3, GPIO_QUAL_SYNC);
}
void INPUTXBAR_init(){
	
	//myINPUTXBAR initialization
	XBAR_setInputPin(XBAR_INPUT1, 57);
	XBAR_setInputPin(XBAR_INPUT2, 22);
	XBAR_setInputPin(XBAR_INPUT5, 40);
	XBAR_setInputPin(XBAR_INPUT6, 39);
}
void OUTPUTXBAR_init(){
	
	//myOUTPUTXBAR0 initialization
	XBAR_setOutputLatchMode(XBAR_OUTPUT3, false);
	XBAR_invertOutputSignal(XBAR_OUTPUT3, false);
		
	//Mux configuration
	XBAR_setOutputMuxConfig(XBAR_OUTPUT3, XBAR_OUT_MUX01_CLB1_OUT4);
	XBAR_enableOutputMux(XBAR_OUTPUT3, XBAR_MUX01);
	
	//myOUTPUTXBAR1 initialization
	XBAR_setOutputLatchMode(XBAR_OUTPUT4, false);
	XBAR_invertOutputSignal(XBAR_OUTPUT4, false);
		
	//Mux configuration
	XBAR_setOutputMuxConfig(XBAR_OUTPUT4, XBAR_OUT_MUX05_CLB2_OUT4);
	XBAR_enableOutputMux(XBAR_OUTPUT4, XBAR_MUX05);
	
	//myOUTPUTXBAR2 initialization
	XBAR_setOutputLatchMode(XBAR_OUTPUT5, false);
	XBAR_invertOutputSignal(XBAR_OUTPUT5, false);
		
	//Mux configuration
	XBAR_setOutputMuxConfig(XBAR_OUTPUT5, XBAR_OUT_MUX09_CLB3_OUT4);
	XBAR_enableOutputMux(XBAR_OUTPUT5, XBAR_MUX09);
	
	//myOUTPUTXBAR3 initialization
	XBAR_setOutputLatchMode(XBAR_OUTPUT8, false);
	XBAR_invertOutputSignal(XBAR_OUTPUT8, false);
		
	//Mux configuration
	XBAR_setOutputMuxConfig(XBAR_OUTPUT8, XBAR_OUT_MUX13_CLB4_OUT4);
	XBAR_enableOutputMux(XBAR_OUTPUT8, XBAR_MUX13);
}
