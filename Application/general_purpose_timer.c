/**************************************************************
 * general_purpose_timer.c
 *
 *  Created on: 30 Apr 2024
 *      Author: Chee
 *
 * Description:  generalPurposeTimer is used for:
 *                            -  adc
 *                            -  data Analytics
 *                            -  led Display
 *                            -  buzzer
 *                            -  lights
 *
 **************************************************************/

#include "UDHAL/UDHAL.h"

#include "Hardware/STM32MCP.h"

#include "Application/general_purpose_timer.h"
#include "Application/brake_and_throttle.h"
#include "Application/snv_internal.h"
#include "Application/periodic_communication.h"
#include "Application/ALS_control.h"
#include "Application/lights.h"
#include "Application/data_analytics.h"
#include "Application/led_display.h"
#include "Application/motor_control.h"
#include "Application/buzzer.h"

#include "Profiles/dashboard_profile.h" // API for sending light mode, light status, unit, speed mode to app

// Task configuration
Task_Struct     gptTask;
uint8_t         gptTaskStack[GPT_TASK_STACK_SIZE];

// Variables
static uint8_t  *ptr_gpt_initComplete_flag = GPT_INACTIVE;  // static enables the same variable name to be used in across various files.
static uint8_t  *ptr_gpt_dashboardErrorCodeStatus;
static sysFatalError_t *ptr_sysFatalError;

//uint32_t    gpt_power_on_time_ms;

// Power On Status Variable
static bool     *ptr_POWER_ON;

/* Local Functions declaration */
static void GeneralPurposeTimer_init( void );
static void GeneralPurposeTimer_taskFxn(UArg a0, UArg a1);
static bool gpt_PWR_OFF();

/*********************************************************************
 * @fn      gpt_InitComplFlagRegister
 *
 * @brief   call to assign and register the pointer to initComplete_flag
 *
 * @param   a pointer to initComplete_flag, i.e. ptr_initComplete_flag
 *
 * @return  None
 */
extern void gpt_InitComplFlagRegister(uint8_t *ptr_initComplete_flag)
{
    ptr_gpt_initComplete_flag = ptr_initComplete_flag;
}

/*********************************************************************
 * @fn      gpt_powerOnRegister
 *
 * @brief   call to assign and register the pointer to powerOn
 *
 * @param   a pointer to powerOn, i.e. ptr_powerOn
 *
 * @return  None
 */
extern void gpt_powerOnRegister(bool *ptrpowerOn)
{
    ptr_POWER_ON = ptrpowerOn;
}

uint8_t *ptr_gpt_opcode;
uint8_t *ptr_gpt_advertiseFlag;

extern void gpt_registeropcode(uint8_t *ptr_opcode, uint8_t *ptr_advertiseFlag)
{
    ptr_gpt_opcode = ptr_opcode;
    ptr_gpt_advertiseFlag = ptr_advertiseFlag;
}

/*********************************************************************
 * @fn      GeneralPurposeTimer_createTask
 *
 * @brief   Task creation function for the General Purpose Timer.
 *
 *********************************************************************/
void GeneralPurposeTimer_createTask(void)
{
    Task_Params gptTaskParams;

    // Configure task
    Task_Params_init(&gptTaskParams);
    gptTaskParams.stack = gptTaskStack;
    gptTaskParams.stackSize = GPT_TASK_STACK_SIZE;
    gptTaskParams.priority = GPT_TASK_PRIORITY;

    Task_construct(&gptTask, GeneralPurposeTimer_taskFxn, &gptTaskParams, NULL);
}

/*********************************************************************
 * @fn      GeneralPurposeTimer_taskFxn
 *
 * @brief   Application task entry point for the General Purpose Timer.
 *
 * @param   a0, a1 - not used.
 *********************************************************************/
uint32_t    gpt_counter = 0;
uint32_t    gpt_counter_NDA = 0;
uint32_t    gpt_counter2 = 0;

uint8_t     autoLightMode = 1;
uint8_t     gpt_i2cOpenStatus;

uint8_t     pinConfig = 0xFF;
uint8_t     gpt_buttonStatus = 0;
uint8_t     gpt_ii = 0;
uint8_t     gpt_snvWriteFlag = 0;
uint8_t     N_data_analytics;

static void GeneralPurposeTimer_taskFxn(UArg a0, UArg a1)
{
  /* Initialize application */
  GeneralPurposeTimer_init();
  buzzer_init();


  for (;;)  /* GPT infinite FOR loop, starting at 1 and without exit condition */
  {
  /****************  Task timing & delay *******************/
  /* Task sleep must be positioned at the beginning of the for loop */
      Task_sleep(GPT_TIME * 1000 / Clock_tickPeriod);

      /***** Do nothing until (*ptr_gpt_initComplet_flag) == 1 *****/
      if (*ptr_gpt_initComplete_flag)
      {
          /*******   N = 1  ********************
           * Use for controlling MCU and Motor */
          /* Read rpm every GPT_TIME */
          periodic_communication_MCUSamplingRPM();

          /* Motor Control Section */
          /* Read brake and throttle ADC values */
          if (!(ptr_sysFatalError->ADCfailure) /* && !(ptr_sysFatalError->UARTfailure)*/ )
          {
              /***  Read ADC values and processes brake and throttle input  ***/
              brake_and_throttle_ADC_conversion();
              /***  execute Motor control command due to brake status change  ***/
              motor_control_brakeStatusChg();     // Note STM32MCP function commented out for debugging purposes
              /***  execute Motor control command due to IQ value  ***/
              motor_control_setIQvalue();         // Note STM32MCP function commented out for debugging purposes
          }

          if ((*ptr_gpt_dashboardErrorCodeStatus) == SYS_FATAL_ERROR_PRIORITY)
          {
          /* Critical system error has occurred -> put system in while loop */
              // 0: activate error event for error handling
              // 1: error handling shall allow led display to display the error code
              // 2: in error mode, system shall be put into the designate error handling protocol
              // 3: in error mode, button shall allow user to Power OFF and ON to reset the firmware and restart the system
          }


          /*********************************************************************************
           * Executes after every N_data_analytics sleeps
           *
           * @ GPT_TIME = 0.100 seconds,
           * Must be consistent with data_analytics.c
           *    data_analytics_sampling_time = GPT_TIME * N_data_analytics;
           *                                 = 0.100 seconds x 3  = 0.300 seconds
           * Use to control data_analytics and Ambient light sensor
           ********************************************************************************/
          if (gpt_counter % N_data_analytics == 0) // N = N_data_analytics
          {
              gpt_counter_NDA++;

          /****  Periodic communication and data analytics must be executed
           *     in the same time interval.
           *     The time interval is N_data_analytics x GPT_TIME
           ****************************************************************/
              /* Read / Sample data from MCU */
              periodic_communication_MCUSampling();
              /* Peforms data analytics */
              data_analytics_sampling();
              data_analytics_Main();

              /* Performs Ambient light sensor operations @ every N = 2 */
              if (!(ptr_sysFatalError->I2Cfailure))
              {
                  lights_ALSFxn();
              }

              led_display_changeDashSpeed();

          }

          /***************************************************************
           * Executes after every 5 sleeps
           * do this if gpt_count is divisible by 5. N = 5
           * GPT_TIME = 0.100 seconds, 5 x 0.100 seconds = 0.500 seconds
           **************************************************************/
          if (gpt_counter % 2 == 0)     // N = 2
          {
              gpt_counter2++;

          /*********** Update Led Display if I2C status is normal ****************/              /* led display commands */
              if (!(ptr_sysFatalError->I2Cfailure))
              {
                  led_display_changeLEDPower();
                  led_display_ErrorDisplay();
                  led_display_changeLightMode();
                  led_display_changeLightStatus();

                  led_display_changeSpeedMode(gpt_counter);    // control law is indicated by flashing or non-falshing speed mode indicator

                  led_display_changeUnit();
                  /*****  BLE indicator flashes on and off when advertising, solid light with connected, off when disconnected  *****/
                  led_display_changeBLE(gpt_counter);
                  led_display_changeBatteryStatus(gpt_counter);

              }

              buzzer_ErrorHandler();

              if (!(ptr_sysFatalError->UARTfailure))
              {

              }

          }


          /**** counter only active if (*ptr_gpt_initComplete_flag) == 1  *****/
          gpt_counter++;
//          gpt_power_on_time_ms = gpt_counter * GPT_TIME;  // for comparison purposes only

          /******************************************************************************
           * When instructed to Power Off, the programme enters here to exit for loop
           ******************************************************************************/
          if (gpt_PWR_OFF() == true)  //or alternatively: if (!(*ptr_gpt_powerOn))
          {
              data_analytics();
              data2UDArray();
              gpt_snvWriteFlag = 1;  // flag = 1 allows simple peripheral to execute save snvBuffer to snv and break out FOR loop

//              break;      // break out of GPT infinite FOR loop
          }

      }
  } /* GPT infinite FOR loop */


  /**** When instructed to Power Off and after breaking out of FOR loop,
   *    the programme exits and reaches here.
   *    The following codes and power off procedure are executed
   ***************************************************************************************/

//    STM32MCP_toggleCommunication();
//    Task_sleep(500 * 1000 / Clock_tickPeriod); /*Wait for a while before sending shutdown command!*/
//    STM32MCP_EscooterShutdown(STM32MCP_POWER_OFF);
//    ledControl_deinit(); /*turns off led display*/
//    pinConfig = PINCC26XX_setWakeup(ExternalWakeUpPin); /*The system resets (REBOOTS) automatically*/
//    Power_shutdown(0, 0); /*System enters Shut Down Mode*/
//    while(1)
//    {
//      /**** infinite while loop until woken up ****/
//    }

}

/*********************************************************************
 * @fn      GeneralPurposeTimer_init
 *
 * @brief   Called during initialization
 *          profile initialization/setup.
 *
 **********************************************************************/
void GeneralPurposeTimer_init( void )
{
    ptr_sysFatalError = UDHAL_sysFatalErrorRegister();
    led_display_gptCounterRegister(&gpt_counter);
    ptr_gpt_dashboardErrorCodeStatus = bat_dashboardErrorCodeStatusRegister();

    N_data_analytics = DATA_ANALYTICS_INTERVAL / GPT_TIME;

}


/*********************************************************************
 * @fn      gpt_PWR_OFF
 *
 * @brief   Process whether POWER OFF is true or false
 *
 * @return  return true if *ptr_POWER_ON = 0 (OFF)
 *          return false if *ptr_POWER_ON = 1 (ON)
 **********************************************************************/
bool gpt_PWR_OFF()
{
    if(*ptr_POWER_ON == 0)
    {
        return true;
    }
    else if(*ptr_POWER_ON == 1)
    {
        return false;
    }
    return false;
}

/*********************************************************************
 * @fn      snvWriteFlageRegister
 *
 * @brief   Return the pointer to snvWriteFlag to calling function
 *
 * @return  pointer to snvWriteFlag
 *
 **********************************************************************/
extern void* snvWriteFlageRegister()
{
    return (&gpt_snvWriteFlag);
}
