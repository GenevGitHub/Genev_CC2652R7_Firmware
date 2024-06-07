/*
 * power_on_time.c
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

/*********************************************************************
 * INCLUDES
 */
#include "Application/power_on_time.h"
#include "Application/simple_peripheral.h"  //  need to obtain SP_ADVERTISING_TIMEOUT

#include "Profiles/dashboard_profile.h"
#include "Profiles/battery_profile.h"
#include "Profiles/controller_profile.h"
#include "Profiles/profile_charVal.h"


/*********************************************************************
 * CONSTANTS
 */
#define BITSPERBYTE                 8

/*********************************************************************
 * LOCAL POINTERS
 */
uint8_t     *ptr_pot_advertiseFlag;
uint8       *ptr_pot_charVal;
profileCharVal_t *ptr_pot_profileCharVal;

/*********************************************************************
 * LOCAL VARIABLES
 */
uint16_t    powerOnTimeMinutes = 0;                    // power on time in minutes
uint32_t    pot_count = 0;
uint32_t    powerOnTimeMS = 0;      // power on time in milli-seconds
uint16_t    pot_advertisingTime = 0;
uint16_t    advertisingTimeout = SP_ADVERTISING_TIMEOUT * 10;   // advertising time defined in simple_peripheral.h in milliseconds
uint8_t     pot_advertisingFlag_old = 0;
bool        *ptr_pot_powerOn;

// Task configuration
Task_Struct potTask;

uint8_t potTaskStack[POT_TASK_STACK_SIZE];


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void power_on_time_cal();
static void power_on_time_taskFxn(UArg a0, UArg a1);

/*********************************************************************
 * @fn      power_on_time_createTask
 *
 * @brief   Task creation function for the power on timer.
 *
 *********************************************************************/
void power_on_time_createTask()
{
    Task_Params potTaskParams;

    // Configure task
    Task_Params_init(&potTaskParams);
    potTaskParams.stack = potTaskStack;
    potTaskParams.stackSize = POT_TASK_STACK_SIZE;
    potTaskParams.priority = POT_TASK_PRIORITY;

    Task_construct(&potTask, power_on_time_taskFxn, &potTaskParams, NULL);
}

/*********************************************************************
 * @fn      power_on_time_init
 *
 * @brief   Initialization and initial set up at each Power On.
 *
 * @param   None
 *
 * @return  None
 *********************************************************************/
void power_on_time_init()
{
    ptr_pot_profileCharVal = profile_charVal_profileCharValRegister();
}


/*********************************************************************
 * @fn      power on time_taskFxn
 *
 * @brief   Application task entry point for the General Purpose Timer.
 *
 * @param   a0, a1 - not used.
 *********************************************************************/
static void power_on_time_taskFxn(UArg a0, UArg a1)
{
  /* Initialize application */
    power_on_time_init();

    for (; ;)               /* infinite for loop, starting at 1 and without exit condition */
    {
    /****************  Task timing & delay *******************/
      Task_sleep(POT_TIME * 1000 / Clock_tickPeriod);
      pot_count++;

      /******* Counting of Device On Time  **************************************************/
      powerOnTimeMS  += POT_TIME;
      power_on_time_cal();

      /**** This is the timer for reseting advertiseFlag to zero to coincident
       *    with GAPadv timeout when no link is established   ****/
      if (*ptr_pot_advertiseFlag)
      {
          pot_advertisingTime += POT_TIME;
          /***** counting will stop when advertisingTime is greater than SP_ADVERTISING_TIMEOUT x 10    ****/
          if (pot_advertisingTime > advertisingTimeout)
          {
              /****  set advertiseFlag = 0 to indicate advertising has terminated  ****/
              *ptr_pot_advertiseFlag = 0;
              pot_advertisingTime = 0;
          }
      }
      else
      {
          if (!pot_advertisingTime)
          {
              pot_advertisingTime = 0;
          }
      }

      if (!(*ptr_pot_powerOn))
      {
          /*** break out of FOR loop during power off  ***/
//          break;
      }

    }/* end FOR loop */

}

/*********************************************************************
 * @fn      power_on_time_getPowerOnTime
 *
 * @brief   call this function to retrieve the Power On Time
 *
 * @param   None
 *
 * @return  powerOnTimeMinute
 *********************************************************************/
extern uint16_t power_on_time_getPowerOnTime(){
    return powerOnTimeMinutes;
}


/*********************************************************************
 * @fn      power_on_time__cal
 *
 * @brief   Calculate power on time.
 *
 * @param   None
 *
 * @return  None
 *********************************************************************/
static void power_on_time_cal()
{
    /********************************   Power On Time (in minutes)   *************************************/
    if (( powerOnTimeMS / POWERONTIME_MINUTE_TIME ) > powerOnTimeMinutes )
    {
        powerOnTimeMinutes++;
        /********** Update Service characteristics ************/
        ptr_pot_charVal = (ptr_pot_profileCharVal->ptr_dash_charVal->ptr_powerOnTime);
        profile_setCharVal(ptr_pot_charVal, DASHBOARD_POWER_ON_TIME_LEN, powerOnTimeMinutes);

    }

}

/*********************************************************************
 * @fn      pot_advertiseFlagRegister
 *
 * @brief   register the pointer to adevertiseFlag
 *
 * @params  ptr_advertiseFlag
 *
 * @return  Nil
 */
extern void pot_advertiseFlagRegister(uint8_t *ptr_advertiseFlag)
{
    ptr_pot_advertiseFlag = ptr_advertiseFlag;
}

/*********************************************************************
 * @fn      pot_powerOnRegister
 *
 * @brief   register the pointer to POWER_ON
 *
 * @params  ptr_powerOn
 *
 * @return  Nil
 */
extern void pot_powerOnRegister(bool *ptr_powerOn)
{
    ptr_pot_powerOn = ptr_powerOn;
}