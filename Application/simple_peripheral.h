/******************************************************************************

 @file  simple_peripheral.h

 @brief This file contains the Simple Peripheral sample application
        definitions and prototypes.

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2013-2024, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

#ifndef SIMPLEPERIPHERAL_H
#define SIMPLEPERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "Hardware/gGo_device_params.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */
// periodic event time interval (in ms) - the interval between BLE refresh/updates. 6 (Strava) to 10 seconds are common.
#define SP_PERIODIC_EVT_PERIOD               750        // 750 ms
#define SP_PERIODIC_EVT_COUNT1               8          // 6000 ms
#define SP_PERIODIC_EVT_COUNT2               4          // 3000 ms

// Task configuration
#define SP_TASK_PRIORITY                     1

#ifndef SP_TASK_STACK_SIZE
#define SP_TASK_STACK_SIZE                   1024
#endif

/***** when NEW_RESET_NVS is defined, the firmware will set snv_internal_80 to the define reset values - IF passcode are not already writtened ***/
#define NEW_RESET_NVS                            1
#ifndef NEW_RESET_NVS
#undef NEW_RESET_NVS
#define HARD_OVERRIDE_NVS                         1   // use hard reset to reset old dashboards to zeros. Old dashboard are dashboard with passcode already writtened
#endif // NEW_NEW_RESET_NVS

#ifndef HARD_OVERRIDE_NVS
#undef HARD_OVERRIDE_NVS
#endif // HARD_OVERRIDE_NVS

#define ZERO_NVS                             1      // when defined -> reset to zeros and override check code
#ifndef ZERO_NVS
#undef ZERO_NVS
#define DUMMY_NVS                            1    // when ZERO_NVS not defined -> reset to dummy data with override check code
#endif // ZERO_NVS

#define SP_ADVERTISING_TIMEOUT               2000  // each tick is in 10 ms, hence 3000 x 10 ms = 30 seconds

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the Simple Peripheral.
 */
extern void SimplePeripheral_createTask(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEPERIPHERAL_H */
