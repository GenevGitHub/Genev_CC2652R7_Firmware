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
#include "Hardware/ESCOOTER_BOOT.h"
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
static uint8_t  *ptr_gpt_snvWriteComplete_flag = 0;
static uint8_t  *ptr_gpt_dashboardErrorCodePriority;
static sysFatalError_t *ptr_sysFatalError;

// Power On Status Variable
static bool     *ptr_gpt_POWER_ON;

/* Local Functions declaration */
static void GeneralPurposeTimer_init( void );
static void GeneralPurposeTimer_taskFxn(UArg a0, UArg a1);


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
    ptr_gpt_POWER_ON = ptrpowerOn;
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
uint32_t    gpt_counter3 = 0;

uint8_t     autoLightMode = 1;
uint8_t     gpt_i2cOpenStatus;

uint8_t     pinConfig = 0xFF;
uint8_t     gpt_buttonStatus = 0;
uint8_t     gpt_ii = 0;
uint8_t     gpt_snvWriteFlag = 0;
uint8_t     N_data_analytics = 0;
uint8_t     N_2 = 0;
uint8_t     N_3 = 0;
uint8_t     brakeStatusOld = 0;
uint8_t     brakeStatusNew = 0;

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
          /**************************   N = 1  ********************
           * EXECUTED EVERY LOOP
           * N = 1 executes codes at the fundamental time interval unit for general purpose timer - GPT_TIME
           * N = 1 is Use for reading motor rpm, brake and throttle, controlling MCU and Motor */
          /************** Read rpm every GPT_TIME *************************/
          if (!(ptr_sysFatalError->UARTfailure))     // added if statement 20241110
          {
              /*Measures the Motor's speed --> calculates the average RPM*/
              periodic_communication_MCUSamplingRPM();
          }

          /* Motor Control Section */
          /* Read brake and throttle ADC values */
          if (!(ptr_sysFatalError->ADCfailure) /* && !(ptr_sysFatalError->UARTfailure)*/ )
          {
              /***  Read ADC values and processes brake and throttle input  ***/
              brakeStatusNew = brake_and_throttle_ADC_conversion();      // uses ADC
              // execute motor_control_brakeStatusChg() and motor_control_setIQvalue() only UART is normal
              if (!(ptr_sysFatalError->UARTfailure))     // added if statement 20241110
              {
                  /***  execute Motor control command due to IQ value  ***/
                  motor_control_setIQvalue();       // uses UART
              }
          }

          if ((*ptr_gpt_dashboardErrorCodePriority) == SYS_FATAL_ERROR_PRIORITY)
          {
          /* Critical system error has occurred -> put system in while loop */
              // 0: activate error event for error handling
              // 1: error handling shall allow led display to display the error code
              // 2: in error mode, system shall be put into the designate error handling protocol
              // 3: in error mode, button shall allow user to Power OFF and ON to reset the firmware and restart the system

          }

          /*********************************************************************************
           * DATA_ANALYTICS_INTERVAL
           *
           * Executes after every N_data_analytics intervals
           *
           * if GPT_TIME = 0.020 seconds,
           * then
           *    N_data_analytics = DATA_ANALYTICS_INTERVAL / GPT_TIME ;
           *                     = 0.300 / 0.020  = 15
           * N_data_analytics is used to control data_analytics and Ambient light sensor executions
           *
           ********************************************************************************/
          if (gpt_counter % N_data_analytics == 0) // N = N_data_analytics
          {
              gpt_counter_NDA++;
          /****  Periodic communication and data analytics must be executed
           *     in the same time interval.
           *     The time interval is N_data_analytics x GPT_TIME
           ****************************************************************/
              /* Read / Sample data from MCU */
              periodic_communication_MCUSampling(); // periodic communication uses UART when MOTOR_CONNECT
              /* Performs data analytics */
              data_analytics_sampling();
              data_analytics_Main();

              /* Performs Ambient light sensor operations @ every N = 2 */
              if (!(ptr_sysFatalError->I2Cfailure))
              {
                  lights_ALSFxn();                  // ALS is command using i2c
                  led_display_changeDashSpeed();    // led display is command using i2c
              }
          }

          /***************************************************************
           * EXECUTION_INTERVAL_2
           *
           * if GPT_TIME = 0.02 seconds, EXECUTION_INTERVAL_2 = 0.140 seconds, then N_2 = 7
           * The following loop is refreshed/executed every N_2 loops
           *
           **************************************************************/
          if (gpt_counter % N_2 == 0)     // execute only when gpt_counter is an even number
          {
              gpt_counter2++;

          /*********** Update Led Display if I2C status is normal ****************/              /* led display commands */
              if (!(ptr_sysFatalError->I2Cfailure))
              {
                  led_display_changeLEDPower();
                  led_display_ErrorDisplay();
                  led_display_changeLightMode();
                  led_display_changeLightStatus();

                  /*** control law selection is indicated by flashing or non-falshing speed mode indicator
                   *    flashing indicates direct law is selected
                   *    non-flashing indicates normal law is selected
                   ***************************************************************************************/
                  led_display_changeSpeedMode(gpt_counter2);
                  led_display_changeUnit();

                  /*****  BLE indicator flashes on and off when advertising, solid light with connected, off when disconnected  *****/
                  led_display_changeBLE(gpt_counter2);
                  led_display_changeBatteryStatus(gpt_counter2);
              }

              if (!(ptr_sysFatalError->UARTfailure))     // added if statement 20241110
              {
                  /***  execute Motor control command due to brake status change  ***/
                  if (brakeStatusOld != brakeStatusNew)     // Added by Chee 20250213
                  {
                      motor_control_taillightStatusChg();         // called only when tail light status has changed, otherwise, it will not reach here
//                      motor_control_brakeStatusChg();
                      brakeStatusOld = brakeStatusNew;
                  }      // Added by Chee 20250213
              }
              buzzer_ErrorHandler();
          }

          /***************************************************************
           * EXECUTION_INTERVAL_3
           *
           * if GPT_TIME = 0.020 seconds, EXECUTION_INTERVAL_3 = 500 milliseconds, then N_3 = 25
           * The following loop is refreshed/executed every N_3 loops
           *
           **************************************************************/
          if (gpt_counter % N_3 == 0)     // execute only when gpt_counter is an even number
          {
              gpt_counter3++;
              /** MCU fault check **/
              if (!(ptr_sysFatalError->UARTfailure))    // Added by Chee 20250213
              {
                  /*Checks UART stability*/
                  motor_control_uartFaultCheck();   // Moved here by Chee 20250213
                  /*Checks motor controller faults*/
                  STM32MCP_controlEscooterBehavior(ESCOOTER_ERROR_REPORT);   // Moved here by Chee 20250213
              }
          }

          /**** counter only active if (*ptr_gpt_initComplete_flag) == 1  *****/
          gpt_counter++;

          /******************************************************************************
           * When instructed to Power Off, the programme enters here to exit for loop
           ******************************************************************************/
//          if (gpt_PWR_OFF() == true)  //or alternatively:
          if (!(*ptr_gpt_POWER_ON))
          {
              bat_zeroIQ();  /* Send IQ = 0 to Motor controller  */
              data_analytics();
              data2snvBuffer();
              gpt_snvWriteFlag = 1;  // flag = 1 allows simple peripheral to execute save snvBuffer to snv and break out FOR loop
              /*********************************************************************************
               * When instructed to Power Off and after breaking out of FOR loop,
               *    the programme exits and reaches here.
               *    The following codes and power off procedure are executed
               ***************************************************************************************/
              lights_setLightOff();       /* Ensure lights are turned off */
              // Add: STM32 command turn off tail-light and auxiliary light
              led_display_setAllOff();    /* turns off all led lights */
              led_display_deinit();       /* turns off led display */

              STM32MCP_clearMsg();
              STM32MCP_toggleCommunication();
              STM32MCP_controlEscooterBehavior(ESCOOTER_TAIL_LIGHT_OFF);
              STM32MCP_controlEscooterBehavior(ESCOOTER_POWER_OFF);
              STM32MCP_clearMsg();

              Task_sleep(300 * 1000 / Clock_tickPeriod);    // sleep for 300 milliseconds

              break;      // break out of GPT infinite FOR loop
          }
      }
  } /* GPT infinite FOR loop */

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
    ptr_gpt_dashboardErrorCodePriority = bat_dashboardErrorCodePriorityRegister();
    N_data_analytics = DATA_ANALYTICS_INTERVAL / GPT_TIME;
    N_2 = EXECUTION_INTERVAL_2 / GPT_TIME;
    N_3 = EXECUTION_INTERVAL_3 / GPT_TIME;
    periodic_communication_init();
}

/*********************************************************************
 * @fn      gpt_snvWriteFlageRegister
 *
 * @brief   Return the pointer to snvWriteFlag to calling function
 *
 * @return  pointer to snvWriteFlag
 *
 **********************************************************************/
extern void* gpt_snvWriteFlageRegister()
{
    return (&gpt_snvWriteFlag);
}
