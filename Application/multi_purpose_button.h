/*
 * multi_purpose_button.h
 *
 *  Created on: 4 May 2024
 *      Author: Chee
 */

#ifndef APPLICATION_MULTI_PURPOSE_BUTTON_H_
#define APPLICATION_MULTI_PURPOSE_BUTTON_H_

#ifdef _cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
/* Library Header files */
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* Application Header files */
#include "Hardware/gGo_device_params.h"
#include "Hardware/gGo_debug_config.h"

/*********************************************************************
 * EXTERNAL VARIABLES
 **********************************************************************/
/*********************************************************************
 * CONSTANTS
 *********************************************************************/

// Task configuration
#define MPB_TASK_PRIORITY                3
#ifndef MPB_TASK_STACK_SIZE
#define MPB_TASK_STACK_SIZE              512  //400 //360    // Stack size must be multiples of 8
#endif

//Constants for timing duration
#define MPB_TIMER_OV_TIME_LONG           1000  //Time expressed in ms, overflow time interval for single long press
#define MPB_TIMER_OV_TIME_SHORT          500   //Time expressed in ms, overflow time interval for short press

//Multi-Purpose Button state
#define MPB_WAITING_STATE                0x00
#define MPB_EXECUTING_STATE              0x01

//constants for Multi-Purpose Button processed messageID
#define MPB_SINGLE_LONG_PRESS_MSG        0x01
#define MPB_SINGLE_SHORT_PRESS_MSG       0x02
#define MPB_SINGLE_SHORT_LONG_PRESS_MSG  0x03
#define MPB_DOUBLE_SHORT_PRESS_MSG       0x04
#define MPB_TREBLE_SHORT_PRESS_MSG       0x05
#define MPB_QUADRUPLE_SHORT_PRESS_MSG    0x06
#define MPB_UNDEFINED_MSG                0x00

/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * @Structure mpb_timing_manager_t
 *
 * @brief     It handles the timer from the application
 *
 * @data      timerStart      //A function that can start the timer
 *            timerStop       //A function that can stop the timer
 *            timerSetPeriod  //A function set the period of the timer
 */
typedef void (*mpb_timerStart)(void);
typedef void (*mpb_timerSetPeriod)(uint32_t timerPeriod);
typedef void (*mpb_timerStop)(void);
typedef struct {
    mpb_timerStart timerStart;
    mpb_timerSetPeriod timerSetPeriod;
    mpb_timerStop  timerStop;
}mpb_timerManager_t;

/*********************************************************************
 * @Structure mpbCBs_t
 *
 * @brief     It stores the application callback function(s) memory address(es)
 *
 * @data
 */
//typedef void (*mpbCB_t)(uint8_t messageID);
//typedef struct {
//    mpbCB_t mpbCB_t;
//}mpbCBs_t;
/*********************************************************************
 * FUNCTIONS
 */
/* Task creation function for the multi-purpose button */
extern void mpb_createTask(void);
extern void mpb_registerTimer(mpb_timerManager_t *mpbTimer);//Register in peripherals already
//extern void mpb_registerCBs(mpbCBs_t *mpbCBs);//Register in main function.
extern void mpb_processButtonEvt(uint_fast8_t logicLevel);
extern void mpb_processTimerOv();
extern void mpb_bootAlert(uint16_t duration, uint8_t bootcase);
extern void *mpb_registeropcode(uint8_t *ptr_opcode, uint8_t *ptr_advertiseFlag);
extern void *mpb_powerOnRegister(void);

#ifdef _cplusplus
}
#endif

#endif /* APPLICATION_MULTI_PURPOSE_BUTTON_H_ */
