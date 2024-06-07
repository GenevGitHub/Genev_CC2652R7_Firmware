/*
 * lights.c
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */
#include "Hardware/gGo_device_params.h"

#include "Application/lights.h"

#include "Application/ALS_control.h"
#include "Application/led_display.h"
#include "UDHAL/UDHAL_PWM.h"

#include "Profiles/controller_profile.h"
#include "Profiles/dashboard_profile.h"
#include "Profiles/battery_profile.h"
#include "Profiles/profile_charVal.h"

/*********************************************************************
 * LOCAL POINTERS
 */
static void (*lightModeArray[3])(void) = {lights_MODE_OFF, lights_MODE_ON, lights_MODE_AUTO};

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8_t     light_mode;
uint8_t     light_mode_Index;
uint8_t     light_status = LIGHT_STATUS_INITIAL;
uint8_t     lightStatusNew = LIGHT_STATUS_INITIAL;

uint16_t    lightControl_pwmPeriod = 1000;
uint16_t    lightControl_pwmDuty = 500;
uint8_t     lightControl_pwmOpenStatus = 0;

profileCharVal_t    *ptr_lights_profileCharVal;
static uint8_t      *ptr_charVal;

/*********************************************************************
 * LOCAL FUNCTIONS
 *
 * void lights_MODE_AUTO();
 * void lights_MODE_OFF();
 * void lights_MODE_ON();
 */
/*********************************************************************
* @fn      lights_taskFxn
*
* @brief   Task creation function for the light Control.
*
* @param   None.
*
* @return  None.
**********************************************************************/
extern void lights_ALSFxn() {
    /* ********************  light sensor control (in minutes)  *******************************
     *****************************************************************************************/
    if (light_mode == 2) {
    /*************   Calls I2C to measure ADC and calculates lux  ****************/
        lightStatusNew = ALS_control_calculateLux();
    /************   Convert ALS_Data to bit-wise  ******************/
    /***   lights_MODE_AUTO updates the light mode indicator on LED display and App *********/
        lights_MODE_AUTO();
    }
}

/*********************************************************************
 * @fn      lightControl_init
 *
 * @brief   Initialization and initial set up at each Power On.  AUTO mode at every POWER ON
 *
 * @param   None
 *
 * @return  None
 *********************************************************************/
void lights_init( uint8_t lights_i2cOpenStatus, uint8_t lightmodeInit )
{
    ptr_lights_profileCharVal = profile_charVal_profileCharValRegister();

    ALS_control_init();

    /** I2C must be initiated before lightControl */
    /** At every POWER ON (SYSTEM START-UP), "IF I2C communication is SUCCESSFUL", the light control is in auto mode (LIGHT_MODE_INITIAL), light is off (LIGHT_STATUS_INITIAL).
    ** "IF I2C communication is NOT successful, we have to disable auto mode. */
    if (lights_i2cOpenStatus == 1)
    {
        light_mode = lightmodeInit; // data_analytics_getLightmodeInit();  // LIGHT_MODE_AUTO;  // if i2c started successfully, light mode default is AUTO
        light_mode_Index = 2;               // if i2c started successfully, AUTO light mode is available
    }
    else if (lights_i2cOpenStatus == 0)
    {
        light_mode = LIGHT_MODE_OFF;        // if i2c did not start successfully, light mode default is OFF
        light_mode_Index = 1;               // if i2c did not start successfully, AUTO light mode will not be available - only ON or OFF light mode is available
    }
    light_status = LIGHT_STATUS_INITIAL;
    lightStatusNew = LIGHT_STATUS_INITIAL;
    led_display_setLightMode( light_mode );
    led_display_setLightStatus( light_status );

}

/*********************************************************************
 * @fn      lights_MODE_AUTO
 *
 * @brief   Execute AUTO light mode routine
 *
 * @param   light_status
 *
 * @return  light_status
 *********************************************************************/

void lights_MODE_AUTO()
{
    if (light_status != lightStatusNew)
    {
        light_status = lightStatusNew;
        lights_statusChg();
    }
}

/*********************************************************************
 * @fn      lights_MODE_OFF
 *
 * @brief   Execute OFF light mode routine
 *
 * @param   None
 *
 * @return  None
 *********************************************************************/
void lights_MODE_OFF()
{
    if (light_status != LIGHT_STATUS_OFF)
    {
        light_status = LIGHT_STATUS_OFF;
        lights_statusChg();
    }
}

/*********************************************************************
 * @fn      lights_MODE_ON
 *
 * @brief   Execute ON light mode routine
 *
 * @param   None
 *
 * @return  None
 *********************************************************************/
void lights_MODE_ON()
{
    if (light_status != LIGHT_STATUS_ON)
    {
        light_status = LIGHT_STATUS_ON;
        lights_statusChg();
    }
}

/*********************************************************************
 * @fn      lights_statusChg
 *
 * @brief   Notify light status change
 *
 * @param   None
 *
 * @return  None
 *********************************************************************/

void lights_statusChg(void)
{
    /* switch LED display brightness depending on light status */
    uint16_t lights_PWMDuty;
    uint8_t ledPower;
    switch(light_status)
    {
    case LIGHT_STATUS_ON:
            {
                /* when light status is on, led power set to led_power_light_on  */
                ledPower = LED_POWER_LIGHT_ON;
    #ifdef CC2652R7_LAUNCHXL
                lights_PWMDuty = LIGHT_PWM_DUTY;
    #endif
                break;
            }
    case LIGHT_STATUS_OFF:
            {
                /* when light status is of led power set to led_power_light_off  */
                ledPower = LED_POWER_LIGHT_OFF;
    #ifdef CC2652R7_LAUNCHXL
                lights_PWMDuty = 0;
    #endif
                break;
            }
    default:
        break;
    }

    #ifdef CC2652R7_LAUNCHXL
    UDHAL_PWM_setHLDutyAndPeriod(lights_PWMDuty);
    #endif

    /* updates light status Characteristic Value -> Mobile App */
    /******  Dashboard services
    *************************************/
    /********** Example Only: Update characteristics Value in dashboard Profile  ************/
//    ptr_charVal = (ptr_lights_DCVArray->ptr_lightStatus );

    ptr_charVal = (ptr_lights_profileCharVal->ptr_dash_charVal->ptr_lightStatus);
    profile_setCharVal(ptr_charVal, DASHBOARD_LIGHT_STATUS_LEN, light_status);       // where is application_setCharVal(address, length, payload)????

    led_display_setLEDPower(ledPower);
    led_display_setLightStatus(light_status);

}

/*********************************************************************
 * @fn      lightControl_Change
 *
 * @brief   Toggles and change to the next light mode, then call to execute light mode change
 *
 * @param   None
 *
 * @return  None
 *********************************************************************/
uint8_t lights_lightModeChange()
{
    light_mode++;

    if(light_mode > light_mode_Index)
    {     // cycles the light mode between 0 to light mode index
        light_mode = 0;
        lightStatusNew =  LIGHT_STATUS_ON;
    }
    /* send light_mode to led display */
    led_display_setLightMode( light_mode );

    /* updates light mode Characteristic Value -> Mobile App */
    /******              Dashboard services                      **************/
    ptr_charVal = (ptr_lights_profileCharVal->ptr_dash_charVal->ptr_lightMode);
    profile_setCharVal(ptr_charVal, DASHBOARD_LIGHT_MODE_LEN, light_mode);

    (*lightModeArray[light_mode])();

    return light_mode;
}

/*********************************************************************
 * @fn      lights_getLightStatus
 *
 * @brief   call this function to retrieve the current light status
 *
 * @param   None
 *
 * @return  light_status
 *********************************************************************/
uint8_t lights_getLightStatus(void){
    return light_status;
}

/*********************************************************************
 * @fn      lights_lightModeRegister
 *
 * @brief   call this function to return the pointer to light mode to the calling function
 *
 * @param   None
 *
 * @return  pointer to light_mode
 *********************************************************************/
extern void* lights_lightModeRegister(){
    return (&light_mode);
}
