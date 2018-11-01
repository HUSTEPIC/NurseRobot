//*****************************************************************************
// pinmux.c
//
// configure the device pins for different signals
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// This file was automatically generated on 7/21/2014 at 3:06:20 PM
// by TI PinMux version 3.0.334
//
//*****************************************************************************


#include "pinmux.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"
#include "prcm.h"

//*****************************************************************************
void
PinMuxConfig(void)
{
    //
    // Enable Peripheral Clocks 
    //
    MAP_PRCMPeripheralClkEnable(PRCM_I2S, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_I2CA0, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);

    //
    // Configure PIN_50 for MCASP0 McAXR1
    //
    MAP_PinTypeI2S(PIN_50, PIN_MODE_6);

#ifdef UART1_PRINTF  
#else
    //
    // Configure PIN_55 for UART0 UART0_TX
    //
    MAP_PinTypeUART(PIN_55, PIN_MODE_3);

    //
    // Configure PIN_57 for UART0 UART0_RX
    //
    MAP_PinTypeUART(PIN_57, PIN_MODE_3);
#endif
    //
    // Configure PIN_63 for MCASP0 McAFSX
    //
    MAP_PinTypeI2S(PIN_63, PIN_MODE_7);

    //
    // Configure PIN_64 for MCASP0 McAXR0
    //
    MAP_PinTypeI2S(PIN_64, PIN_MODE_7);

#ifdef USE_AMOMCU_BOARD 


    //
    // Configure PIN_16 for I2C0 I2C_SCL
    //
    MAP_PinTypeI2C(PIN_16, PIN_MODE_9);

    //
    // Configure PIN_17 for I2C0 I2C_SDA
    //
    MAP_PinTypeI2C(PIN_17, PIN_MODE_9);   

    // 注意， 以下代码非常重要， 原来是PIN_01 与 PIN_02 作为iic ， 
    // 现在用 PIN_16与 PIN_17作为iic， 需要把原来的设置为其他功能例如gpio 口
    
    {
        MAP_PinTypeGPIO(PIN_01, PIN_MODE_0, false);
        MAP_GPIODirModeSet(GPIOA1_BASE, 0x04, GPIO_DIR_MODE_OUT);

        MAP_PinTypeGPIO(PIN_02, PIN_MODE_0, false);
        MAP_GPIODirModeSet(GPIOA1_BASE, 0x08, GPIO_DIR_MODE_OUT);
    }
//    while(1)
//    {
//      MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_2,GPIO_PIN_2);
//      MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_2,0);

//      MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_3,GPIO_PIN_3);
//      MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_3,0);

//    }
//    
#else
	//
    // Configure PIN_01 for I2C0 I2C_SCL
    //
    MAP_PinTypeI2C(PIN_01, PIN_MODE_1);

    //
    // Configure PIN_02 for I2C0 I2C_SDA
    //
    MAP_PinTypeI2C(PIN_02, PIN_MODE_1);

#endif
    //
    // Configure PIN_04 for GPIOInput
    //
    MAP_PinTypeGPIO(PIN_04, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x20, GPIO_DIR_MODE_IN);

    //
    // Configure PIN_15 for GPIOInput
    //
    MAP_PinTypeGPIO(PIN_15, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA2_BASE, 0x40, GPIO_DIR_MODE_IN);

    //
    // Configure PIN_53 for MCASP0 McACLK
    //
    MAP_PinTypeI2S(PIN_53, PIN_MODE_2);
    
 
#if 0
    //
    // Configure PIN_03 for GPIOOutput
    //
    MAP_PinTypeGPIO(PIN_03, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_OUT);    
    
    //while(1)
    {
      MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_4,0);
      MAP_UtilsDelay(200000);
      MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_4,GPIO_PIN_4);
      MAP_UtilsDelay(200000);      
    }
#endif 

#if 0
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);
    //
    // Configure PIN_03 for GPIOOutput
    //
    MAP_PinTypeGPIO(PIN_50, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, 0x01, GPIO_DIR_MODE_OUT);    
    
    while(1)
    {
      MAP_GPIOPinWrite(GPIOA0_BASE,GPIO_PIN_0,0);
      MAP_UtilsDelay(2000);
      MAP_GPIOPinWrite(GPIOA0_BASE,GPIO_PIN_0,GPIO_PIN_0);
      MAP_UtilsDelay(2000);      
    }
#endif  


}

void Pin_reset(void)
{
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);


    MAP_PinTypeGPIO(PIN_03, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_OUT);    
    
    //while(1)
    {
      MAP_UtilsDelay(200000);
      MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_4,0);
      MAP_UtilsDelay(20000000);
      MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_4,GPIO_PIN_4);
      MAP_UtilsDelay(2000000);      
    }
}

void PinMuxConfig_for_Uart1(void)   
{
#ifdef UART1_PRINTF  
    //
    // Enable Peripheral Clocks 
    //
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);

    //-------------------------------------------------
    //
    // Configure PIN_01 for MODE7 UART1_TX
    //
    MAP_PinTypeUART(PIN_01, PIN_MODE_7);
#endif  
}


