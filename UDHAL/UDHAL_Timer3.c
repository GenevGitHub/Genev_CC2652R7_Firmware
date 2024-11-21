/*
 * UDHAL_Timer3.c
 *
 * @brief This library is used for singleButton.h to set, count and detect the button press and press duration
 *  Created on: 4 May 2024
 *      Author: Chee
 */
/*********************************************************************
 * INCLUDES
 */
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/Error.h>

#include "UDHAL/UDHAL_Timer3.h"

//#include "Application/multi_purpose_button.h"

/*********************************************************************
 * LOCAL VARIABLES
 */
static Clock_Handle ClockHandle;
static Clock_Params clkParams;
static uint32_t clockTicks;
static Error_Block eb;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void UDHAL_TIMER3_start();
static void UDHAL_TIMER3_setPeriod(uint32_t clockTimeout);
static void UDHAL_TIMER3_stop();
static void UDHAL_TIMER3_OVClockFxn();
/*********************************************************************
 * Marco
 */
static mpb_timerManager_t mpb_timer =
{
    UDHAL_TIMER3_start,
    UDHAL_TIMER3_setPeriod,
    UDHAL_TIMER3_stop
};


/*********************************************************************
 *
* @fn      UDHAL_TIMER3_init
 *
 * @brief   To initialize the timer
 *
 * @param   None.
 *
 * @return  None.
 */
uint8_t timer3_count = 0;    // for debugging purposes only
void UDHAL_TIMER3_init()
{
    Error_init(&eb);
    clockTicks = MPB_TIMER_OV_TIME_LONG * (1000 / Clock_tickPeriod) - 1;  // -1 ensures Overflow occurs at SINGLE_BUTTON_TIMER_OV_TIME_LONG - not 1 tick after SINGLE_BUTTON_TIMER_OV_TIME_LONG
    ClockHandle = Clock_create(UDHAL_TIMER3_OVClockFxn, clockTicks, &clkParams, &eb);

//    mpb_registerTimer(&mpb_timer); // send pointer to mpb_timer to button.c   // Warning: UDHAL_TIMER3_init() is called by mpb.c, but here, UDHAL_TIMER3_init() also calls a function in mpb.c
                                                                                // Such coding practice (recursive/circular calling) must not be allowed.
    timer3_count++; // for debugging purposes only
}
/*********************************************************************
 *
 * @fn      UDHAL_TIMER3_params_init
 *
 * @brief   To config the timer
 *
 * @param   None.
 *
 * @return  None.
 */
void UDHAL_TIMER3_params_init()
{
    Clock_Params_init(&clkParams);
    clkParams.period =  0;
    clkParams.startFlag = FALSE;
    clkParams.arg = (UArg)0x0000;
    timer3_count++;  // for debugging purposes only
}

/*********************************************************************
 * @fn      UDHAL_TIMER3_mpbTimerRegister
 *
 * @brief   sends the pointer to mpb_timer to the calling function.
 *          It is used to process the button press timing and behavior
 *          when the multi-purpose button is pressed.
 *
 * @param   Nil
 *
 * @return  A set of function pointer contain the timer function
 */
extern mpb_timerManager_t* UDHAL_TIMER3_mpbTimerRegister()
{
    return (&mpb_timer);
}


/*********************************************************************
 * @fn      UDHAL_TIMER3_start
 *
 * @brief   To start the timer for timeout.
 *
 * @param   None.
 *
 * @return  None.
 */
static void UDHAL_TIMER3_start()
{
   // Set the initial timeout
    Clock_start(ClockHandle);
}

/*********************************************************************
 * @fn      UDHAL_TIM3_setPeriod
 *
 * @brief   It is used by Button.c to set the period of the clock, change the configuration to meet the requirement
 *          It is used to set the timer overflow time, change the configuration to meet the requirement
 *
 * @param   clockTimeout is time in milliseconds.
 *
 *
 * @return  none
 */
static void UDHAL_TIMER3_setPeriod(uint32_t clockTimeout)
{
    uint32_t ticks = clockTimeout * (1000 / Clock_tickPeriod) - 1; // -1 ensures Overflow occurs at clockTimeout - not 1 tick after clockTimeout
    Clock_setTimeout(ClockHandle, ticks);
}

/*********************************************************************
 * @fn      UDHAL_TIMER3_stop
 *
 * @brief   To stop the timer for flow control timeout.
 *
 * @param   None.
 *
 * @return  None.
 */
static void UDHAL_TIMER3_stop()
{
    Clock_stop(ClockHandle);
}

/*********************************************************************
 * @fn      UDHAL_TIMER3_OVClockFxn
 *
 * @brief   After timeout (overflow), button pressed sequence is processed at mpb_processTimerOV()
 *
 * @param   none
 *
 * @return  none
 */
static void UDHAL_TIMER3_OVClockFxn()
{
    UDHAL_TIMER3_stop();
    mpb_processTimerOv();
}

/*********************************************************************
 * @fn      UDHAL_TIMER3_clockDelete
 *
 * @brief   Delete Clock
 *
 * @param   none
 *
 * @return  none
 */
extern void UDHAL_TIMER3_clockDelete()
{
    Clock_delete(&ClockHandle);
}


