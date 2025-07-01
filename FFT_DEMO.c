//#############################################################################
//
// FILE:   empty_driverlib_main.c
//
//! \addtogroup driver_example_list
//! <h1>Empty Project Example</h1> 
//!
//! This example is an empty project setup for Driverlib development.
//!
//
//#############################################################################
//
//
// $Copyright:
// Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "c2000ware_libraries.h"
#include "c2000_freertos.h"
#include "FreeRTOS.h"
#include "dsp.h"
#include "fpu_cfft.h"
#include "fpu_rfft.h"
#include "math.h"

#pragma DATA_SECTION(test_input, "FFT_inputBuffer_1")
#pragma DATA_SECTION(test_output, "FFT_inputBuffer_2")

#define FFT_STAGES 7
#define FFT_SIZE 128
CFFT_F32_STRUCT FFT_obj;
CFFT_F32_STRUCT_Handle FFT_handle = &FFT_obj;

float test_input[FFT_SIZE<<1];
float test_output[FFT_SIZE<<1];
float twiddleFactors[FFT_SIZE];

void fftInit(void);
void signalFilling(void);

//
// Main
//
void main(void)
{

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // PinMux and Peripheral Initialization
    //
    Board_init();

    //
    // C2000Ware Library initialization
    //
    C2000Ware_libraries_init();
    fftInit();
    signalFilling();


    CFFT_f32(FFT_handle);
    CFFT_f32_mag_TMU0(FFT_handle);
    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT;
    ERTM;

    while(1)
    {
        
    }
}
void fftInit(void)
{

    CFFT_f32_setInputPtr(FFT_handle, test_input);
    CFFT_f32_setOutputPtr(FFT_handle, test_output);
    CFFT_f32_setStages(FFT_handle, FFT_STAGES);
    CFFT_f32_setFFTSize(FFT_handle, FFT_SIZE);
    CFFT_f32_setTwiddlesPtr(FFT_handle, twiddleFactors);
    //generate twiddle factor and save to the designated array
    CFFT_f32_sincostable(FFT_handle);

}

void signalFilling(void)
{
    uint32_t i=0;
    for(i=0;i<FFT_SIZE;i++)
    {
        test_input[2*i]=1000.0f*sinf(2*M_PI*43.0f*i/FFT_SIZE)+10*cosf(2*M_PI*25.0f*i/FFT_SIZE);
        test_input[2*i+1]=0.0f;
    }
}
//
// vApplicationStackOverflowHook - Checks run time stack overflow
//
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
   ( void ) pcTaskName;
   ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}

//
// vApplicationMallocFailedHook - Hook function for catching pvPortMalloc() failures
//
void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}

//
// End of File
//
