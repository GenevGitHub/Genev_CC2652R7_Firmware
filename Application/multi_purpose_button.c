/*
 *  multi_purpose_button.c
 *
 *  Created on: 4 May 2024
 *      Author: Chee
 *
 *  multi_purpose_button.c contains all functions associated with the
 *  multi-purpose button.  These functions include:
 *      - Power On and Power Off of the entire system
 *      - toggling light mode, speed mode, speed unit
 *      - activating BLE
 *      - changing control law
 *      - behavior of the button
 *
 *  Clock and interrupt is used to listen to button presses and
 *  the combinations of presses by the user.
 *
 */
/*********************************************************************
* INCLUDES
*/
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

/* Application Header files */
#include "Application/multi_purpose_button.h"
#include "Application/general_purpose_timer.h"
#include "Application/brake_and_throttle.h"
#include "Application/lights.h"
#include "Application/data_analytics.h"
#include "Application/buzzer.h"
#include "Application/power_on_time.h"
#include "Application/led_display.h"

#include "UDHAL/UDHAL_Timer3.h"

/*********************************************************************
* LOCAL VARIABLES
*/

static bool         POWER_ON = 1;               // send pointer to POWER_ON to gpt to register its address

static uint8_t      mpb_buttonState = MPB_WAITING_STATE; //It's a default waiting state!!!!
static uint32_t     timerPeriod;
static uint8_t      risingEdgeCount = 0;    // make this static if not debugging
static uint8_t      fallingEdgeCount = 0;   // make this static if not debugging

mpb_timerManager_t  *mpb_timerManager; //singleButton_timerManager

uint8_t             mpb_buzzerStatus = 0;
static uint8_t      *ptr_dashunit;
static uint8_t      *ptr_dashboardErrorCodeStatus;

void mpb_execute_event(uint8_t messageID);
void mpb_taskFxn(UArg a0, UArg a1);
void mpb_init();

/*********************************************************************
 * @fn      mpb_powerOnRegister
 *
 * @brief   call to return the pointer to POWER_ON to the calling function
 *
 * @param
 *
 * @return  pointer to POWER_ON
 */
void* mpb_powerOnRegister(void){
    return (&POWER_ON);
}


/*********************************************************************
 * @fn      mpb_registeropcode
 *
 * @brief   call by simple peripheral to register opcode and advertiseFlag,
 *          and return the pointer register to POWER_ON to simple peripheral
 *
 * @param   ptr_opcode, ptr_advertiseFlag
 *
 * @return  pointer to mpb_GAPflag
 */
static uint8_t *ptr_GAPopcode;
static uint8_t *ptr_mpb_advertiseFlag;
uint8_t mpb_GAPflag = 0;

extern void *mpb_registeropcode(uint8_t *ptr_opcode, uint8_t *ptr_advertiseFlag){
    ptr_GAPopcode = ptr_opcode;
    ptr_mpb_advertiseFlag = ptr_advertiseFlag;
    return (&mpb_GAPflag);
}

/*********************************************************************
* @fn      mpb_createTask
*
* @brief   Task creation function for the multi-purpose button.
*
* @param   None.
*
* @return  None.
*********************************************************************/
// Task configuration
Task_Struct     mpbTask;
Char            mpbTaskStack[MPB_TASK_STACK_SIZE];

extern void mpb_createTask(void){
    Task_Params mpbTaskParams;

    // Configure task
    Task_Params_init(&mpbTaskParams);
    mpbTaskParams.stack = mpbTaskStack;
    mpbTaskParams.stackSize = MPB_TASK_STACK_SIZE;
    mpbTaskParams.priority = MPB_TASK_PRIORITY;

    Task_construct(&mpbTask, mpb_taskFxn, &mpbTaskParams, NULL);     //

}

/*********************************************************************
* @fn      mpb_taskFxn
*
* @brief   Task creation function for the general purpose timer
*
* @param   None
*
* @return  None
**********************************************************************/
uint8_t         mpb_buttonStatus = 0;
uint8_t         mpb_errorStatus = 0xFF;

void mpb_taskFxn(UArg a0, UArg a1)
{

    mpb_timerManager = UDHAL_TIMER3_mpbTimerRegister();

    mpb_init();

}

/*********************************************************************
* LOCAL FUNCTIONS
*/
/*********************************************************************
 * @fn      mpb_Init
 *
 * @brief   It is used to initialize the library
 *
 * @param   none
 *
 * @return  none
 */
void mpb_init()
{

    gpt_powerOnRegister(&POWER_ON);
    pot_powerOnRegister(&POWER_ON);
    buzzer_buzzerStatusRegister(&mpb_buzzerStatus);
    ptr_dashunit = data_analytics_ptrUnitSelectDash();
    ptr_dashboardErrorCodeStatus = bat_dashboardErrorCodeStatusRegister();

}

/*********************************************************************
 * @fn      mpb_processButtonEvt
 *
 * @brief   It is used to process the data when the button is pressed
 *
 * @param   logicLevel It takes the logic level of the GPIO to decide whether it is a falling edge or rising edge
 *
 *
 * @return  none
 */
void mpb_processButtonEvt(uint_fast8_t logicLevel)
{
    if(logicLevel == 0)
    {
        fallingEdgeCount++;
        /* instruct buzzer to make a single beep on every falling edge */
#ifdef CC2652R7_LAUNCHXL
        mpb_buzzerStatus = 1; // when buzzerStatus == 1, buzzer makes a single beep, 1 == short beep
        buzzer_buttonHandler(mpb_buzzerStatus);
#endif
    }

    if(fallingEdgeCount == 0)    // Ignores the rising edge after a long press
    {
        risingEdgeCount = 0;
        return;
    }

    if(logicLevel == 1)
    {
        risingEdgeCount++;
        /* instruct buzzer to stop beep on every rising edge */
#ifdef CC2652R7_LAUNCHXL
        mpb_buzzerStatus = 0; //  button will always be released (end with a rising edge) after a press. Reset buzzer status to 0 on every release.
        buzzer_buttonHandler(mpb_buzzerStatus);
#endif
    }

    switch(mpb_buttonState)
    {
    case MPB_WAITING_STATE:
        {
            mpb_buttonState = MPB_EXECUTING_STATE;
            timerPeriod = MPB_TIMER_OV_TIME_LONG;
            mpb_timerManager->timerSetPeriod(timerPeriod);
            mpb_timerManager->timerStart();
            break;
        }
    case MPB_EXECUTING_STATE:
        {
            timerPeriod = MPB_TIMER_OV_TIME_SHORT;
            mpb_timerManager->timerStop();
            mpb_timerManager->timerSetPeriod(timerPeriod);
            mpb_timerManager->timerStart();
            break;
        }
    default:
       break;
    }
}

/*********************************************************************
 * @fn      mpb_processTimerOv
 *
 * @brief   when timer3 overflows, this function is used to determine the button event (user command)
 *
 * @param   none
 *
 * @return  none
 */
uint8_t buttonEvent = 0x00;

void mpb_processTimerOv()
{
    // TOGGLE POWER ON/OFF (1 long press)
    if (risingEdgeCount == 0 && fallingEdgeCount == 1){
        buttonEvent = 0x01;
    }
    // Change Light Mode (1 short press)
    else if (risingEdgeCount == 1 && fallingEdgeCount == 1){
        buttonEvent = 0x02;                             //callback -> lightControl_change();
    }
    // TOGGLE BLE Advertising (1 short + 1 long presses)
    else if (risingEdgeCount == 1 && fallingEdgeCount == 2){
        buttonEvent = 0x03;
    }
    // CHANGE SPEED MODE (2 short presses)
    else if (risingEdgeCount == 2 && fallingEdgeCount == 2){
        buttonEvent = 0x04;
    }
    // TOGGLE UNITS METRIC/IMPERIAL (3 short presses)
    else if (risingEdgeCount == 3 && fallingEdgeCount == 3){
        buttonEvent = 0x05;
    }
    // TOGGLE CONTROL LAW (4 short + 1 long presses)
    else if (risingEdgeCount == 4 && fallingEdgeCount == 5){
        buttonEvent = 0x06;
    }
    // DO NOTHING
    else
    {
        buttonEvent = 0x00;
    }

#ifdef CC2652R7_LAUNCHXL
    /* reset buzzer to off here */
    buzzer_buttonHandler(0);
#endif

    timerPeriod = MPB_TIMER_OV_TIME_LONG;     // resets to "SINGLE_BUTTON_TIMER_OV_TIME_LONG" after each overflow
    risingEdgeCount = 0;                         // reset to 0
    fallingEdgeCount = 0;                        // reset to 0
    mpb_buttonState = MPB_WAITING_STATE;          // reset to 0
    mpb_execute_event(buttonEvent);         /* calls motorcontrol_mpbCB(buttonEvent) to execute the buttonEvent */
}


/*********************************************************************
 * @fn      mpb_execute_event
 * @brief   Execute Commands/Actions of user-inputed multi-purpose button Event
 *
 * @param
 *
 * @return  none
 ************************************************************************/

uint8_t messageid = 0;              // for debugging only
uint8_t switch_check = 0;

void mpb_execute_event(uint8_t messageID) {
    messageid = messageID;  // for debugging only

    switch(messageID)
    {
    case MPB_SINGLE_LONG_PRESS_MSG:       // case = 0x01 - toggle Device Power ON / OFF
        {
            switch_check = 1;

            /****  toggle Power ON/OFF or Enter/Exit Sleep Mode ****/
            /**** if Powering On -> switch to Power Off ****/
            if(POWER_ON)
            {

                POWER_ON = 0;
                /*  gracefully disable / exit relevant tasks and put system in sleep mode  */

            }
            /**** if Powering Off -> switch to Power On ****/
            else //if (POWER_ON == 0)
            {

                POWER_ON = 1;
                /* restart firmware  */

            }
        // ICallPlatform_pwrNotify(unsigned int eventType, uintptr_t eventArg, uintptr_t clientArg)
            break;
        }
    case MPB_SINGLE_SHORT_PRESS_MSG:      // case = 0x02 - toggle light modes
        {
            switch_check = 2;

            lights_lightModeChange();

            break;
        }
    case MPB_SINGLE_SHORT_LONG_PRESS_MSG: // case = 0x03 - ADVERT_ENABLE if BLE in waiting state or waiting after timeout state
        {
            switch_check = 3;

            if((*ptr_GAPopcode == GAP_DEVICE_INIT_DONE_EVENT) || ( *ptr_GAPopcode == GAP_LINK_TERMINATED_EVENT))
            {
                if ((mpb_GAPflag != 1 ) && (*ptr_mpb_advertiseFlag != 1))// if not already advertising and if GAPflag is not already 1
                {
                    mpb_GAPflag = 1;
                }
            }

            if (*ptr_mpb_advertiseFlag) // if advertising is 1, reset GAPflag to 0
            {
                mpb_GAPflag = 0;
            }

            break;
        }
    case MPB_DOUBLE_SHORT_PRESS_MSG:      // case = 0x04 - toggle speed modes
        {
            switch_check = 4;
            /**** toggle speed mode ****/
            brake_and_throttle_toggleSpeedMode();

            break;
        }
    case MPB_TREBLE_SHORT_PRESS_MSG:      // case = 0x05 - toggle units
        {
            switch_check = 5;
            if (*ptr_dashunit == IMP_UNIT)
            {
                *ptr_dashunit = SI_UNIT;
            }
            else if (*ptr_dashunit == SI_UNIT)
            {
                *ptr_dashunit = IMP_UNIT;
            }
            /* send UnitSelectDash to dataAnalysis.c */
            data_analytics_changeUnitSelectDash();

            break;
        }
    case MPB_QUADRUPLE_SHORT_PRESS_MSG:      // case = 0x06 - toggle control law
        {
            switch_check = 6;

            /*****  toggles control law  *****/
            uint8_t mpbControlLaw = brake_and_throttle_getControlLaw();
            if (mpbControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
            {
                mpbControlLaw = BRAKE_AND_THROTTLE_DIRECTLAW;
            }
            else
            {
                mpbControlLaw = BRAKE_AND_THROTTLE_NORMALLAW;
            }
            /* send mpbControlLaw to brakeAndThrotte.c */
            brake_and_throttle_setControlLaw(mpbControlLaw);
            led_control_setControlLaw(mpbControlLaw);

            break;
        }
    default:
        {    // case 0x00 and all other cases
            switch_check = 0;
            // mpb_unrecognised_Press_flag = 1;

        break;
        }
    }
}

