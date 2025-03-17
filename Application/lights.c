/*
 *  lights.c
 *
 *  Lights.c contains codes, commands and controls for head light and tail light
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#include "Hardware/gGo_device_params.h"
#include "Hardware/STM32MCP.h"

#include "Application/lights.h"
#include "Application/ALS_control.h"
#include "Application/led_display.h"
#include "Application/brake_and_throttle.h"

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
uint8_t     taillightStatus = ESCOOTER_TAIL_LIGHT_OFF;
uint8_t     auxiliary_light_status  = LIGHT_STATUS_INITIAL;

static uint8_t     lightStatusNew = LIGHT_STATUS_INITIAL;

static uint16_t    lightControl_pwmPeriod = 1000;
static uint8_t     lightControl_pwmOpenStatus = 0;

profileCharVal_t    *ptr_lights_profileCharVal;
static uint8_t      *ptr_charVal;
STM32MCPD_t     *ptr_lights_STM32MCPDArray;

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
extern void lights_ALSFxn(){
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
uint8_t lights_uart2ErrorStatus = 0;
uint8_t *ptr_lights_flagb;
uint8_t lights_sampleBits = 0;

void lights_init( uint8_t lights_i2cOpenStatus, uint8_t uart2ErrorStatus, uint8_t lightmodeInit ){
    ptr_lights_profileCharVal = profile_charVal_profileCharValRegister();
    lights_uart2ErrorStatus = uart2ErrorStatus;
    ptr_lights_flagb = ALS_control_flagbRegister();
    auxiliary_light_status = bat_auxiliaryLightStatusRegister(&auxiliary_light_status);

    lights_sampleBits = ALS_control_init();

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

    if (light_mode == LIGHT_MODE_ON)
    {
        light_status = LIGHT_STATUS_ON;
        *ptr_lights_flagb = lights_sampleBits;
        taillightStatus = ESCOOTER_TAIL_LIGHT_ON;
    }
    else
    {
        light_status = LIGHT_STATUS_OFF;
        *ptr_lights_flagb = 0;
        taillightStatus = ESCOOTER_TAIL_LIGHT_OFF;
    }

    lightStatusNew = light_status;
    ptr_lights_STM32MCPDArray->tail_light_status = taillightStatus;

    /** set light mode on LED display **/
    led_display_setLightMode( light_mode );

    /** command lights and set light status and LED power on LED display **/
    /* led_display_init() must be before lights_init() */
    lights_statusChg();     //    activate light, sync light status with MCU , led_display_setLEDPower() & led_display_setLightStatus( light_status );

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
void lights_MODE_AUTO(){
    if (light_status != lightStatusNew)
    {
        light_status = lightStatusNew;
        if (light_status == LIGHT_STATUS_ON)
        {
            taillightStatus = ESCOOTER_TAIL_LIGHT_ON;
        }
        else
        {
            taillightStatus = ESCOOTER_TAIL_LIGHT_OFF;
        }
        /**** send new tail light status to motor_controller to command tail light ****/
        ptr_lights_STM32MCPDArray->tail_light_status = taillightStatus;
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
void lights_MODE_OFF(){
    if (light_status != LIGHT_STATUS_OFF)
    {
        light_status = LIGHT_STATUS_OFF;
        *ptr_lights_flagb = 0;
        taillightStatus = ESCOOTER_TAIL_LIGHT_OFF;
        /**** send new tail light status to motor_controller to command tail light ****/
        ptr_lights_STM32MCPDArray->tail_light_status = taillightStatus;
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
void lights_MODE_ON(){
    if (light_status != LIGHT_STATUS_ON)
    {
        light_status = LIGHT_STATUS_ON;
        *ptr_lights_flagb = lights_sampleBits;
        taillightStatus = ESCOOTER_TAIL_LIGHT_ON;
        /**** send new tail light status to motor_controller to command tail light ****/
        ptr_lights_STM32MCPDArray->tail_light_status = taillightStatus;
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
uint16_t lights_PWMDuty;

void lights_statusChg(void){
    /* switch LED display brightness depending on light status */
    uint8_t  ledPower;

    switch(light_status)
    {
    case LIGHT_STATUS_ON:
    {
        /* when light status is on (ambient light must be low), led power set to led_power_light_on (a low power value)  */
        ledPower = LED_POWER_LIGHT_ON;
        lights_PWMDuty = LIGHT_PWM_DUTY;
        break;
    }
    case LIGHT_STATUS_OFF:
    {
        /* when light status is off (ambient light must be high), led power set to led_power_light_off (a high power value)  */
        ledPower = LED_POWER_LIGHT_OFF;
        lights_PWMDuty = 0;
        break;
    }
    default:
        break;
    }

    /*** execute headlight commands ***/
    UDHAL_PWM_setHLDutyAndPeriod(lights_PWMDuty);
    /*** execute taillight commands ***/
    //if (brakeStatus == 0){
        if (!lights_uart2ErrorStatus)// if no uart error
        {
    //        motor_control_taillightControl(light_status);
            motor_control_taillightStatusChg();         // called only when light status has changed, otherwise, it cannot reach here
        }
    //}

    /* updates light status Characteristic Value -> Mobile App */
    /******  Dashboard services  *************************************/
    ptr_charVal = (ptr_lights_profileCharVal->ptr_dash_charVal->ptr_lightStatus);
    profile_setCharVal(ptr_charVal, DASHBOARD_LIGHT_STATUS_LEN, light_status);

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
uint8_t lights_lightModeChange(){
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
    return (light_mode);
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
extern uint8_t lights_getLightStatus(){
    return (light_status);
}

/*********************************************************************
 * @fn      lights_lightStatusRegister
 *
 * @brief   call this function to return the pointer to light status to the calling function
 *
 * @param   None
 *
 * @return  pointer to light_status
 *********************************************************************/
extern void* lights_lightStatusRegister(){
    return (&light_status);
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

/*********************************************************************
 * @fn      lights_STM32MCPDArrayRegister
 *
 * @brief   receive the pointer to STEM32MCDArray
 *
 * @param   None
 *
 * @return  None
 *********************************************************************/
extern void lights_STM32MCPDArrayRegister(STM32MCPD_t *ptrSTM32MCDArray){
    ptr_lights_STM32MCPDArray = ptrSTM32MCDArray;
}

/*********************************************************************
 * @fn      lights_setLightOff
 *
 * @brief   Turn off light
 *
 * @param   None
 *
 * @return  None
 *********************************************************************/
extern void lights_setLightOff( void ){
    light_mode = LIGHT_MODE_OFF;
    lights_MODE_OFF();
}


uint16_t AuxiliaryLight_PWMDuty;
void auxiliaryLightStatusChg()
{
    switch(auxiliary_light_status)
    {
    case LIGHT_STATUS_ON:
    {
        AuxiliaryLight_PWMDuty = AUXLIGHT_PWM_DUTY;
        break;
    }
    case LIGHT_STATUS_OFF:
    {
        AuxiliaryLight_PWMDuty = 0;
        break;
    }
    default:
        break;
    }
#ifdef AUXILIARY_LIGHT
    /*** activate auxiliary light ***/
    UDHAL_PWM_setALDutyAndPeriod(AuxiliaryLight_PWMDuty);
#endif // AUXILIARY_LIGHT

}

//void tail_lightControl(void)  // Chee -> I think this is not needed anymore
//{
//        motor_control_taillightControl(lights_getLightStatus());
//}
