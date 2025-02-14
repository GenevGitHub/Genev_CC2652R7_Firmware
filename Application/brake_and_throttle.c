/*
 * brake_and_throttle.c
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */
/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <math.h>

#include "Hardware/gGo_device_params.h"
#include "Hardware/STM32MCP.h"

#include "Application/brake_and_throttle.h"

#include "Profiles/dashboard_profile.h"
#include "Profiles/controller_profile.h"
#include "Profiles/battery_profile.h"
#include "Profiles/profile_charVal.h"

#include "Application/led_display.h"
#include "Application/snv_internal.h"
#include "Application/periodic_communication.h"

#include "UDHAL/UDHAL_ADC.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 *********************************************************************/
/* ControlLaw Options: NormalLaw (1) or DirectLaw (0).
 *          Normal law algorithm modulates the speed to not exceed the defined limit
 *          Direct law algorithm does not modulate the speed in any way
 */
uint8_t     ControlLaw = BRAKE_AND_THROTTLE_NORMALLAW;
/*  Options:
 *  (1) BRAKE_AND_THROTTLE_NORMALLAW
 *  (2) BRAKE_AND_THROTTLE_DIRECTLAW
*/

// Power On Status Variable
static uint8_t     *ptr_bat_POWER_ON;

uint8_t     speedMode;

uint16_t    adcValues[2];
uint16_t    adc2Result;             // adc2Result is a holder of the throttle ADC reading
uint16_t    throttlePercent;        // Actual throttle applied in percentage
uint16_t    throttlePercentApplied = 0;
uint16_t    IQ_input;               // Iq value input by rider
uint16_t    IQ_applied = 0;             // Iq value command sent to STM32 / motor Controller
uint16_t    brakePercent;           // brake applied in percentage
uint8_t     brakeStatus = 0;
uint16_t    throttleSpeedControl = 0;
uint8_t     dashboardErrorCodePriority = SYSTEM_NORMAL_PRIORITY;

/********************************************************************
 *  when brakeAndThrottle_errorMsg is true (=1),
 *  it generally means either
 *  (1) brake signal is not connected and/or
 *  (2) throttle signal is not connect
 *
 *  IQ_input is set to 0
 */
uint8_t brakeAndThrottle_errorMsg = BRAKE_AND_THROTTLE_NORMAL;
uint8_t throttle_errorStatus = 0;   // 0 indicates normal (no error)
uint8_t brake_errorStatus = 0;      // 0 indicates normal (no error)

/****** Safety Protections  **************************************/
// Safety feature 1:  Speed mode cannot be changed while throttle is pressed
// Safety feature 2:  Abnormal brake or throttle signals will automatically change speed mode to amble mode
//                    0E indicates brake error.  0C indicates throttle error
// Safety feature 3:  Power to motor is cut when brake is engaged
// Safety feature 4:  Speed and Power protections are activated under Normal Control Law
// Safety feature 5:  Power is only applied to motor at speed greater than 3 km/hr (or 80 rpm)
// Safety feature 6:  No Power is applied at negative speeds
/*********************************************************************
 *
 * LOCAL VARIABLES
 */
profileCharVal_t *ptr_bat_profileCharVal;
static uint8_t   *ptr_charVal;
uint8_t *ptr_bat_errorPriority;

static uint8_t  *ptr_bat_auxiliaryLightStatus;
static uint8_t  brakeAndThrottleIndex = 0;
uint16_t        brakeADCSamples[BRAKE_AND_THROTTLE_SAMPLES];
uint16_t        throttleADCSamples[BRAKE_AND_THROTTLE_SAMPLES];
uint16_t        RPM_array[BRAKE_AND_THROTTLE_SAMPLES];

/** Speed limiter control variables **/
uint16_t IQ_increment;    // max IQ increment
float alpha = 0;    // -infinity <= alpha < 1

/**  Speed mode parameters  **/
uint16_t    speedModeIQmax;
static uint8_t      reductionRatio;
static uint16_t     rampRate;
uint16_t    allowableRPM;
uint16_t    IQ_maxPout = 0xFFFF;

/**  Data structs  **/
MCUD_t          *ptr_bat_MCUDArray;
AD_t            *ptr_bat_ADArray;

/*********************************************************************
* FUNCTIONS
*/
/**** obtain/register the pointer to MCUDArray   ****/
extern void brake_and_throttle_MCUArrayRegister(MCUD_t *ptrMCUDArray)
{
    ptr_bat_MCUDArray = ptrMCUDArray;
}

/**** obtain/register the pointer to ADArray   ****/
extern void brake_and_throttle_ADArrayRegister(AD_t *ptrADArray)
{
    ptr_bat_ADArray = ptrADArray;
}

/**** obtain/register the pointer to STM32MCPDArray   ****/
STM32MCPD_t *ptr_bat_STM32MCPDArray;
extern void brake_and_throttle_STM32MCDArrayRegister(STM32MCPD_t *ptrSTM32MCPDArray)
{
    ptr_bat_STM32MCPDArray = ptrSTM32MCPDArray;
}

/**********************************************************************
 *  Local functions
 */
static void brake_and_throttle_directLawAlgorithm();
static void brake_and_throttle_normalLawAlgorithm();
static void throttleSpeedControlAlgorithm();
static void brake_and_throttle_OutputPowerLimitAlgorithm();
/*********************************************************************
 * @fn      brake_and_throttle_init
 *
 * @brief   Start the brake ADC and timer
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
void brake_and_throttle_init()
{
    /**  Set up registers  **/
    ptr_bat_profileCharVal = profile_charVal_profileCharValRegister();
    UDHAL_ADC_ptrADCValues(&adcValues);
    data_analytics_dashErrorCodeStatusRegister(&dashboardErrorCodePriority);
    ptr_bat_errorPriority = led_display_errorPriorityRegister();

    /* Get initial speed mode */
    uint8_t speedModeinit;
    speedModeinit = data_analytics_getSpeedmodeInit();
    speedMode = speedModeinit;

    /* Initiate, obtain and update STM32 speed mode parameters */
    brake_and_throttle_getSpeedModeParams();

    for (uint8_t ii = 0; ii < BRAKE_AND_THROTTLE_SAMPLES; ii++)
    {
        brakeADCSamples[ii] = BRAKE_ADC_CALIBRATE_L;
        throttleADCSamples[ii] = THROTTLE_ADC_CALIBRATE_L;
        RPM_array[ii] = 0;
    }

    /* led_display_init() must be executed before brake_and_throttle_init() */
    led_display_setSpeedMode(speedMode);  // update speed mode on dashboard led display
    led_control_setControlLaw(ControlLaw);

}

/*********************************************************************
 * @fn      brake_and_throttle_ADC_conversion
 *
 * @brief   This function perform ADC conversion
 *          This function is called when timer3 overflows
 *
 * @param
 *********************************************************************/
uint8_t     brake_errorFlag = 0;
uint16_t    RPM_temp;
static uint8_t     brakeAndThrottleIndex_minus_1;
static uint8_t     brakeAndThrottleIndex_minus_2;
uint8_t     throttle_error_count = 0;
uint8_t     brake_error_count = 0;
uint16_t    throttleADCsample = 0;  //throttleADCsample
uint16_t    brakeADCsample = 0;     //brakeADCsample

/**  for studying purposes - evaluating speed and IQ *******************/
static uint16_t    bat_count = 0;
static uint16_t    RPM_prev;
static uint16_t    IQapp_prev = 0;
float       drpmdIQ;
int         drpm;
int         dIQ;

/***    End for studying purposes   *************************************/

/***    This is the main function of brake_and_throttle.c
 *      This function
 *          reads the brake and throttle ADC values,
 *          checks the brake and throttle values and range for abnormality and errors
 *          calculates throttle and brake percentage
 *          Calculate and modulate the IQ values
 ********************************************************************************/
uint8_t brake_and_throttle_ADC_conversion()
{
    /************************************************************************************
     *      get brake ADC measurement
     *      get throttle ADC measurement
     *      Stores ADC measurement in arrays brakeADCSamples & throttleADCSamples
     ************************************************************************************/
    UDHAL_ADC_Convert();
    throttleADCsample = adcValues[0] * ADC_CORRECTION_FACTOR;
    brakeADCsample = adcValues[1] * ADC_CORRECTION_FACTOR;

    /*******************************************************************************************************************************
     *      Error Checking
     *      Check whether throttle ADC reading is logical, if illogical, brakeAndThrottle_errorMsg = error (!=0)
     *      These Conditions occur when throttle or brake signals/power are not connected, or incorrect supply voltage
     *      Once this condition occurs (brakeAndThrottle_errorMsg != 0), check throttle connections, hall sensor fault,
     *      Reset (Power off and power on again) is require to reset brakeAndThrottle_errorMsg.
     *******************************************************************************************************************************/
    if ( (throttleADCsample >= THROTTLE_ADC_THRESHOLD_L) && (throttleADCsample <= THROTTLE_ADC_THRESHOLD_H))
    {
        if (throttle_error_count != 0){
            throttle_error_count = 0;   // Reset throttle_error_count
        }
    }
    else    //    if ( (throttleADCsample < THROTTLE_ADC_THRESHOLD_L) || (throttleADCsample > THROTTLE_ADC_THRESHOLD_H) )
    {
        throttle_error_count++;
        if ((throttle_error_count >= 8) && (!throttle_errorStatus) )
        {
            throttle_errorStatus = 1;
            brakeAndThrottle_errorMsg = THROTTLE_ERROR;

        /* if throttle errorStatus = 1 -> disable throttle input and zero Iq command to Motor Controller Unit */
            led_display_ErrorPriority(THROTTLE_ERROR_PRIORITY);   /* throttle error priority = 11 */

            /******  Dashboard services *************************************/
            /* updates error code Characteristic Value -> Mobile App */
            if (dashboardErrorCodePriority > THROTTLE_ERROR_PRIORITY)
            {
                dashboardErrorCodePriority = THROTTLE_ERROR_PRIORITY;
                ptr_bat_ADArray->dashboardErrorCode = THROTTLE_ERROR_CODE;
                ptr_charVal = (ptr_bat_profileCharVal->ptr_dash_charVal->ptr_dashErrorCode);
                profile_setCharVal(ptr_charVal, DASHBOARD_ERROR_CODE_LEN, ptr_bat_ADArray->dashboardErrorCode);
            }
        }
    }   // -> This set of codes determines if throttle error is present

    /*******************************************************************************************************************************
     *      Throttle Signal Calibration
     *      Truncates the average throttle ADC signals to within THROTTLE_ADC_CALIBRATE_L and THROTTLE_ADC_CALIBRATE_H
     *******************************************************************************************************************************/
    if ((throttleADCsample > THROTTLE_ADC_CALIBRATE_H))
    {
        throttleADCsample = THROTTLE_ADC_CALIBRATE_H;
    }
    if ((throttleADCsample < THROTTLE_ADC_CALIBRATE_L))
    {
        throttleADCsample = THROTTLE_ADC_CALIBRATE_L;
    }
    throttleADCSamples[ brakeAndThrottleIndex ] = throttleADCsample;

    /*******************************************************************************************************************************
     *      Error Checking
     *      Check whether brake ADC reading is logical, if illogical, brakeAndThrottle_errorMsg = error (!=0)
     *      These Conditions occur when brake signals/power are not connected, or incorrect supply voltage
     *      Once this condition occurs (brakeAndThrottle_errorMsg != 0), check brake connections, hall sensor fault,
     *      Reset (Power off and power on again) is require to reset brakeAndThrottle_errorMsg.
     *******************************************************************************************************************************/
    if ( (brakeADCsample >= BRAKE_ADC_THRESHOLD_L) && (brakeADCsample <= BRAKE_ADC_THRESHOLD_H))
    {
        if (brake_error_count != 0){
            brake_error_count = 0;   // Reset brake_error_count
        }
    }
    else    //    if ((brakeADCsample < BRAKE_ADC_THRESHOLD_L) || (brake_errorFlag != 0) || (brakeADCsample > BRAKE_ADC_THRESHOLD_H))
    {
        /* Adjust Sensitivity */
        brake_error_count++;
        if ((brake_error_count >= 8) && (brake_errorStatus == 0))
        {
            brake_errorFlag = 1; /* set and flag that brake_errorFlag = 1 */
        }

      /* For safety reason, speed mode will not immediately change, but flagged to do so only when throttle value is less than THROTTLE_ADC_CALIBRATE_L */
      /* if brake_errorStatus was originally == 0 and throttle is not pressed, then update (this routine prevents repetitive executions) */
        if ((brake_errorStatus == 0) && (throttleADCsample <= THROTTLE_ADC_CALIBRATE_L))
        {
            brakeAndThrottle_errorMsg = BRAKE_ERROR;
            brake_errorStatus = 1;
           /* When brake error occurs, i.e. brake errorStatus = 1 -> limited to Amble mode Only */
            speedMode = BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE;

           /* Note: speed mode is changed! Update Led Display, update App data */
            brake_and_throttle_toggleSpeedMode();   // execute speed mode change
            led_display_setSpeedMode(speedMode);   // set speed mode on led display

            /* Send error code to the Motor Controller */
            led_display_ErrorPriority(BRAKE_ERROR_PRIORITY);   // send brake error priority = 10
            brake_errorFlag = 0; /* reset brake_errorFlag to 0 */

            /******  Dashboard services *************************************/
            /* updates error code Characteristic Value -> Mobile App */
            if (dashboardErrorCodePriority > BRAKE_ERROR_PRIORITY)
            {
                dashboardErrorCodePriority = BRAKE_ERROR_PRIORITY;
                ptr_bat_ADArray->dashboardErrorCode = BRAKE_ERROR_CODE;
                ptr_charVal = (ptr_bat_profileCharVal->ptr_dash_charVal->ptr_dashErrorCode);
                profile_setCharVal(ptr_charVal, DASHBOARD_ERROR_CODE_LEN, ptr_bat_ADArray->dashboardErrorCode);
            }
        }
    }   // this set of codes checks for brake error

    /*******************************************************************************************************************************
     *      Brake Signal Calibration
     *      Truncates the average brake ADC signals to within BRAKE_ADC_CALIBRATE_L and BRAKE_ADC_CALIBRATE_H
     *******************************************************************************************************************************/
    if((brakeADCsample > BRAKE_ADC_CALIBRATE_H))
    {
        brakeADCsample = BRAKE_ADC_CALIBRATE_H;
    }
    if((brakeADCsample < BRAKE_ADC_CALIBRATE_L))
    {
        brakeADCsample = BRAKE_ADC_CALIBRATE_L;
    }
    brakeADCSamples[ brakeAndThrottleIndex ] = brakeADCsample;

    /*******************************************************************************************************************************
     *      the sampling interval is defined by "GPT_TIME"
     *      the number of samples is defined by "BRAKE_AND_THROTTLE_SAMPLES"
     *      Sum the most recent "BRAKE_AND_THROTTLE_SAMPLES" number of data points, and
     *      calculate weighted moving average brake and throttle ADC values
     *      Weight factor of 2 is applied to the newest throttle sample, all other samples have weight factor of 1
     *******************************************************************************************************************************/
    uint16_t    brakeADCsum = 0;
    uint16_t    throttleADCsum = 0;
    uint8_t     jj = 0;

    for (jj = 0; jj < BRAKE_AND_THROTTLE_SAMPLES; jj++)
    {
        brakeADCsum += brakeADCSamples[jj];
        throttleADCsum += throttleADCSamples[jj];
    }

    /***** Calculate moving average values ******/
    uint16_t    brakeADCAvg = brakeADCsum / BRAKE_AND_THROTTLE_SAMPLES;             // declared as global variable for debugging only
    uint16_t    throttleADCAvg = throttleADCsum / BRAKE_AND_THROTTLE_SAMPLES;

    /********************************************************************************************************************************
     *  brakePercent is in percentage - has value between 0 - 100 %
     *  throttlePercent is in percentage - has value between 0 - 100 %
     *********************************************************************************************************************************/
    brakePercent = (brakeADCAvg - BRAKE_ADC_CALIBRATE_L) * 100 / (BRAKE_ADC_CALIBRATE_H - BRAKE_ADC_CALIBRATE_L);
    throttlePercent = (throttleADCAvg - THROTTLE_ADC_CALIBRATE_L) * 100 / (THROTTLE_ADC_CALIBRATE_H - THROTTLE_ADC_CALIBRATE_L);

    /**** throttleSpeedControl correlates throttlePercent with targeted Speed   **********************/
    throttleSpeedControl = throttlePercent * (allowableRPM - (REG_MINP_RPM / 2)) / 100 + (REG_MINP_RPM / 2);

    /*****  update brakeAndThrottle_errorMsg on STM32MCPDArray.error_msg  *****/
    ptr_bat_STM32MCPDArray->error_msg = brakeAndThrottle_errorMsg;

    /********************** Brake Power Cut Off Protect State Machine  *******************************************************************************
     *              Brake is considered engaged when brakePercent is greater than BRAKEPERCENTTHRESHOLD,
     *              dashboard will instruct motor controller to cut power to motor.
     *              Once power to motor is cut, both the brake & throttle must be fully released before power delivery can be resumed
    **********************************************************************************************************************************************/
    if (brake_errorStatus == 0)/* if there is no brake error .... */
    {
     /* condition where brake is engaged (1) and throttle is greater than 30% */
        if ((brakeStatus == 1) && (brakePercent <= BRAKEPERCENTTHRESHOLD) && (throttlePercent <= THROTTLEPERCENTTHRESHOLD))
        { // This condition resets brakeStatus to zero
            brakeStatus = 0;
        }
        else if ((brakeStatus == 0) && (brakePercent > BRAKEPERCENTTHRESHOLD))
        {// condition when brake is not initially pressed and rider pulls on the brake
            brakeStatus = 1;    // if brakeStatus == 1, cut power to motor
        }
    }
    else
    {  // when brake_errorStatus == 1, we cannot know if the brake lever is pulled or not. Therefore, set brake status = 0 at all times.
        brakeStatus = 0;    // if we set brakeStatus = 1 when brake error exists, IQ value sent to motor is set to zero.
                            // if we set brakeStatus = 0 when brake error exists, command to motor is as normal.
    }

    /**************************************************************************************
     *       Send / update brake% and brake status in STM32MCPDArray
     **************************************************************************************/
    ptr_bat_STM32MCPDArray->brake_percent = brakePercent;
    ptr_bat_STM32MCPDArray->brake_status = brakeStatus;

    /******** Get RPM from mcu  ******************/
    RPM_temp = ptr_bat_MCUDArray->speed_rpm;            //

    /******** Throttle Error Safety Protocol -> when throttle error detected or speed is negative, IQ_input is set to zero
     *  Calculating the IQ Value
     *  Notes: Power is delivered to the motor if:
     *   (1) brake is not engaged
     *   (2) RPM is not negative or is greater than the REG_MINP_RPM
     *   (3) no detected critical hardware / firmware errors
     *  These features are for safety reasons
     **********************************************************************************/
    /**       if rpm is positive       and   if error is not fatal or critical errors  **/
    if ((ptr_bat_MCUDArray->rpm_status) && (*ptr_bat_errorPriority >= BRAKE_ERROR_PRIORITY))
    {
        switch (ControlLaw)
        {
        case (BRAKE_AND_THROTTLE_DIRECTLAW):
        {
            brake_and_throttle_directLawAlgorithm();
            break;
        }
        case (BRAKE_AND_THROTTLE_NORMALLAW):
        {
            brake_and_throttle_normalLawAlgorithm();
            break;
        }
        default:
            break;
        }
    }
    else   /** if rpm is negative or if error is fatal or critical errors  **/
    {
        IQ_input = 0;
        IQ_applied = IQ_input;
    }

    /********************************************************************************************************************************
     * Update IQ_applied to STM32MCPDArray.IQ_value fot commanding Motor via motor controller
     ********************************************************************************************************************************/
    ptr_bat_STM32MCPDArray->IQ_value = IQ_applied;

    /***** Increments brakeAndThrottleIndex by 1  ***/
    brakeAndThrottleIndex++;
    if (brakeAndThrottleIndex >= BRAKE_AND_THROTTLE_SAMPLES)
    {
        brakeAndThrottleIndex = 0;
    }

    return (brakeStatus);
}

/*********************************************************************
 * @fun    brake_and_throttle_directLawAlgorithm
 *
 * @brief   Normal Law Algorithm
 *
 * @param   Nil
 *
 * @return  Nil
 *********************************************************************/
static void brake_and_throttle_directLawAlgorithm()
{
    /** To get here,
     * (1) RPM status is 1, i.e., RPM is positive
     * (2) No fatal errors
     ***/
    if ((brakeStatus) || (RPM_temp < REG_MINP_RPM)) /* if brake is activated or rpm is less than the min power rpm */
    {
    #ifndef MOTOR_0RPM_START_MODE
        throttlePercentApplied = 0;
        IQ_input = 0;
    #endif  // MOTOR_0RPM_START_MODE

    #ifdef MOTOR_0RPM_START_MODE
        if (brakeStatus)
        {
            IQ_input = 0;
            throttlePercentApplied = 0;
        }
        else
        {
            if (throttlePercent == 0)
            {
                throttlePercentApplied = 0;
            }
            else if (throttlePercent > throttlePercentApplied)
            {
                if (throttlePercentApplied < 6)
                {
                    throttlePercentApplied = 6; // The minimum starting throttle Percent is 6% for computing IQ_input.
                                                // If starting IQ is too small, motor will not start.
                }
                else
                {
                    if ((throttlePercent - throttlePercentApplied) > THROTTLE_INCREMENT_LIMIT)
                    {
                        throttlePercentApplied += THROTTLE_INCREMENT_LIMIT;    // throttle percent increase limited to "THROTTLE_INCREMENT_LIMIT"% at any time
                    }
                    else
                    {
                        throttlePercentApplied = throttlePercent;
                    }
                }
            }
            else
            {
                throttlePercentApplied = throttlePercent;
            }
            IQ_input = speedModeIQmax * throttlePercentApplied / 100;
        }
    #endif  // MOTOR_0RPM_START_MODE

    brake_and_throttle_OutputPowerLimitAlgorithm();
    }
    else
    {
        if (throttlePercent == 0)
        {
            throttlePercentApplied = 0;
        }
        else if (throttlePercent > throttlePercentApplied)
        {

            if (throttlePercentApplied < 6)
            {
                throttlePercentApplied = 6; // The minimum starting throttle Percent is 6% for computing IQ_input.
                                            // If starting IQ is too small, motor will not start.
            }
            else
            {
                if ((throttlePercent - throttlePercentApplied) > THROTTLE_INCREMENT_LIMIT)
                {
                    throttlePercentApplied += THROTTLE_INCREMENT_LIMIT;    // throttle percent increase limited to "THROTTLE_INCREMENT_LIMIT"% at any time
                }
                else
                {
                    throttlePercentApplied = throttlePercent;
                }
            }
        }
        else
        {
            throttlePercentApplied = throttlePercent;
        }

        IQ_input = speedModeIQmax * throttlePercentApplied / 100;
        brake_and_throttle_OutputPowerLimitAlgorithm();
    }

    IQ_applied = IQ_input;
}

/*********************************************************************
 * @fun    brake_and_throttle_normalLawAlgorithm
 *
 * @brief   Normal Law Algorithm
 *
 * @param   Nil
 *
 * @return  Nil
 *********************************************************************/
static void brake_and_throttle_normalLawAlgorithm()
{
    /** To get here,
     * (1) RPM status is 1, i.e., RPM is positive
     * (2) No fatal errors
     ***/

    if ((brakeStatus) || (RPM_temp < REG_MINP_RPM)) /* if brake is activated or rpm is less than the min power rpm */
    {
    #ifndef MOTOR_0RPM_START_MODE
        IQ_input = 0;
    #endif  // MOTOR_0RPM_START_MODE

    #ifdef MOTOR_0RPM_START_MODE
        if (brakeStatus)
        {
            IQ_input = 0;
        }
        else
        {
            throttleSpeedControlAlgorithm();
            brake_and_throttle_OutputPowerLimitAlgorithm();

        }
    #endif  // MOTOR_0RPM_START_MODE
    }
    else
    {
        throttleSpeedControlAlgorithm();
        brake_and_throttle_OutputPowerLimitAlgorithm();

    }

    IQ_applied = IQ_input;
}

/*********************************************************************
 * @fun    throttleSpeedControlAlgorithm
 *
 * @brief   Speed Control Algorithm
 *
 * @param   Nil
 *
 * @return  Nil
 *********************************************************************/
uint8_t OPL_exceedance = 0; // debugging

uint8_t speedControlLimit1 = 0;
uint8_t speedControlLimit2 = 0;
static void throttleSpeedControlAlgorithm()
{
    IQ_increment = speedModeIQmax / NN_IQ_INCREMENTS;
    speedControlLimit1 = allowableRPM / SPEEDCONTROLLIMITFACTOR1;
    speedControlLimit2 = allowableRPM / SPEEDCONTROLLIMITFACTOR2;

    /*******************************   1. Speed limit algorithm  **************************************/
    if (throttleSpeedControl < REG_MINP_RPM)
    {   // when the equivalent throttle speed is less than the min motor powered rpm, IQ = 0
        alpha = (float) 0;
        IQ_input = 0;
    }
    else
    {
//        if (IQ_input == 0)
//        {
//            IQ_input = 0.1 * speedModeIQmax;
//        }
//        else if (throttlePercent > throttlePercentApplied)
//        {
//
//        }
        if (throttleSpeedControl >= RPM_temp)   // RPM_temp is >= min powered rpm
        {
            if ((throttleSpeedControl - RPM_temp) < speedControlLimit1)  // difference in rpm less than "x" rpm (8 rpm = 0.3 km/h)
            {
                alpha = (float) 0;
            }
            else if (((throttleSpeedControl - RPM_temp) >= speedControlLimit1) && ((throttleSpeedControl - RPM_temp) < speedControlLimit2))
//            if ((throttleSpeedControl - RPM_temp) < speedControlLimit2)
            {
                alpha = (float)(throttleSpeedControl - RPM_temp) / (throttleSpeedControl);
            }
            else
            {
                alpha = (float)(speedControlLimit2) / (throttleSpeedControl); // caps the max change in speedControl per loop
            }
            IQ_input = (uint16_t) (IQ_input + (float) alpha * IQ_increment);

            if (IQ_input > speedModeIQmax)
            {
                IQ_input = speedModeIQmax;
            }
        }
        else    //else if (throttleSpeedControl < RPM_temp)
        {
            if ((RPM_temp - throttleSpeedControl) < speedControlLimit1)  // difference in rpm less than "x" rpm (8 rpm = 0.3 km/h)
            {
                alpha = (float) 0;
            }
            else
            {
                alpha = (float)(throttleSpeedControl - RPM_temp) / (throttleSpeedControl);
                if (alpha < -1)
                {
                    alpha = -1;
                }
            }
            if ((IQ_input + (float) alpha * IQ_increment) <= 0)
            {
                IQ_input = 0;
            }
            else
            {
                IQ_input = IQ_input + (float) alpha * IQ_increment;
            }
        }
    }
}

///*********************************************************************
// * @fun    brake_and_throttle_OutputPowerLimitAlgorithm
// *
// * @brief   Output Power Limit Protection
// *
// * @param   Nil
// *
// * @return  Nil
// *********************************************************************/
static void brake_and_throttle_OutputPowerLimitAlgorithm()
{
    /**********************  2. Output Power Limit Protection  **********************************************/
    if (RPM_temp >= 1)
    {
        if ((REG_MAXPOUT / (RPM_temp * 2 * PI_CONSTANT / 60) / KT_CONSTANT / KIQ_CONSTANT) > speedModeIQmax)
        {
            IQ_maxPout = speedModeIQmax;
        }
        else
        {
            IQ_maxPout = (float) REG_MAXPOUT / (RPM_temp * 2 * PI_CONSTANT / 60) / KT_CONSTANT / KIQ_CONSTANT;
        }
    }
    else
    {
        IQ_maxPout = speedModeIQmax;
    }

    if (IQ_input > IQ_maxPout)
    {
        IQ_input = IQ_maxPout;    // making sure IQ applied will not exceed allowable output power limit
        OPL_exceedance ++;
    }
}

/*********************************************************************
 * @fn      brakeAndThrottle_getSpeedModeParams
 *
 * @brief   Get speed Mode parameters
 *
 * @param   speedMode
 *
 * @return  none
 *********************************************************************/
extern void brake_and_throttle_getSpeedModeParams()
{
    switch(speedMode)
    {
    case BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE:                   // Amble mode
        {
            if (ControlLaw == BRAKE_AND_THROTTLE_DIRECTLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_AMBLE;
                allowableRPM = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS * 1.5;
            }
            else if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS;
                allowableRPM = BRAKE_AND_THROTTLE_MAXSPEED_AMBLE;
            }
            rampRate = BRAKE_AND_THROTTLE_RAMPRATE_AMBLE;
            break;
        }
    case BRAKE_AND_THROTTLE_SPEED_MODE_LEISURE:                 // Leisure mode
        {
            if (ControlLaw == BRAKE_AND_THROTTLE_DIRECTLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_LEISURE;
                allowableRPM = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS * 1.5;
            }
            else if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS;
                allowableRPM = BRAKE_AND_THROTTLE_MAXSPEED_LEISURE;
            }
            rampRate = BRAKE_AND_THROTTLE_RAMPRATE_LEISURE;
            break;
        }
    case BRAKE_AND_THROTTLE_SPEED_MODE_SPORTS:                  // Sports mode
        {
            if (ControlLaw == BRAKE_AND_THROTTLE_DIRECTLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS;
                allowableRPM = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS * 1.5;
            }
            else if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS;
                allowableRPM = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS;
            }
            rampRate = BRAKE_AND_THROTTLE_RAMPRATE_SPORTS;
            break;
        }
    default:
        break;
    }
    speedModeIQmax = reductionRatio * STM32MCP_TORQUEIQ_MAX / 100;

    /* Send updated speed mode parameters to motor control unit */
    /*  these following codes are necessary for control law change  */
    ptr_bat_STM32MCPDArray->speed_mode = speedMode;
    ptr_bat_STM32MCPDArray->speed_mode_IQmax = speedModeIQmax;
    ptr_bat_STM32MCPDArray->ramp_rate = rampRate;
    ptr_bat_STM32MCPDArray->allowable_rpm = allowableRPM;
    /* call to update and execute speed mode change on MCU */
    motor_control_changeSpeedMode();   // Note STM32MCP functions have been commented out for debugging purposes

    /* updates led display */
    led_display_setSpeedMode(speedMode);    // update led display
    /******  Dashboard services *************************************/
    /* updates speed mode Characteristic Value -> Mobile App */
    bat_dashboard_speedmode_service();

}

/*********************************************************************
 * @fn      bat_dashboard_speedmode_service
 *
 * @brief   Dashboard services updates speed mode Characteristic Value to Mobile App
 *
 * @param   none
 *
 * @return  none
 */
extern void bat_dashboard_speedmode_service()
{
    /******  Dashboard services *************************************/
    /* updates speed mode Characteristic Value -> Mobile App */
    ptr_charVal = (ptr_bat_profileCharVal->ptr_dash_charVal->ptr_speedMode);
    profile_setCharVal(ptr_charVal, DASHBOARD_SPEED_MODE_LEN, speedMode);
}

/*********************************************************************
 * @fn      brake_and_throttle_toggleSpeedMode
 *
 * @brief   To change / toggle the speed Mode of the e-scooter
 *          Amble mode:     up to 10km/h
 *          Leisure mode:   up to 18km/h
 *          Sports mode:    up to max regulated speed (25km/h)
 *
 * @param   none
 *
 * @return  none
 */
uint8_t brake_and_throttle_toggleSpeedMode()
{
    if (brake_errorStatus == 0) // no brake error
    {
        if (throttleADCsample <= 1.1 * THROTTLE_ADC_CALIBRATE_L)    // Fully release throttle to change speed mode,  no change when throttle is applied - will by-pass if throttle is not zero
        {
            if(speedMode == BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE)  // if Amble mode, change to Leisure mode
            {
                speedMode = BRAKE_AND_THROTTLE_SPEED_MODE_LEISURE;
                if (ControlLaw == BRAKE_AND_THROTTLE_DIRECTLAW)
                {
                    reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_LEISURE;
                    allowableRPM = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS * 1.5;
                }
                else if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
                {
                    reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS;
                    allowableRPM = BRAKE_AND_THROTTLE_MAXSPEED_LEISURE;
                }
                rampRate = BRAKE_AND_THROTTLE_RAMPRATE_LEISURE;

                // turn off auxiliary light
                *ptr_bat_auxiliaryLightStatus = 0;
            }
            else if(speedMode == BRAKE_AND_THROTTLE_SPEED_MODE_LEISURE) // if Leisure mode, change to Sports mode
            {
                speedMode = BRAKE_AND_THROTTLE_SPEED_MODE_SPORTS;
                if (ControlLaw == BRAKE_AND_THROTTLE_DIRECTLAW)
                {
                    reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS;
                    allowableRPM = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS * 1.5;
                }
                else if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
                {
                    reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS;
                    allowableRPM = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS;
                }
                rampRate = BRAKE_AND_THROTTLE_RAMPRATE_SPORTS;
            }
            else if(speedMode == BRAKE_AND_THROTTLE_SPEED_MODE_SPORTS) // if Sports mode, change back to Amble mode
            {
                speedMode = BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE;
                if (ControlLaw == BRAKE_AND_THROTTLE_DIRECTLAW)
                {
                    reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_AMBLE;
                    allowableRPM = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS * 1.5;
                }
                else if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
                {
                    reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS;
                    allowableRPM = BRAKE_AND_THROTTLE_MAXSPEED_AMBLE;
                }
                rampRate = BRAKE_AND_THROTTLE_RAMPRATE_AMBLE;

                // turn on auxiliary light
                *ptr_bat_auxiliaryLightStatus = 1;
            }
        }
    }
    else    // brake error
    {
        if(speedMode != BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE)        // This condition prevents unnecessary repetitive changes that does nothing
        {
            speedMode = BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE;
            if (ControlLaw == BRAKE_AND_THROTTLE_DIRECTLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_AMBLE;
                allowableRPM = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS * 1.5;
            }
            else if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS;
                allowableRPM = BRAKE_AND_THROTTLE_MAXSPEED_AMBLE;
            }
            rampRate = BRAKE_AND_THROTTLE_RAMPRATE_AMBLE;

            // turn on auxiliary light
            *ptr_bat_auxiliaryLightStatus = 1;
        }
    }

    speedModeIQmax = reductionRatio * STM32MCP_TORQUEIQ_MAX / 100;

    /* Send updated speed mode parameters to motor control unit */
    ptr_bat_STM32MCPDArray->speed_mode = speedMode;
    ptr_bat_STM32MCPDArray->speed_mode_IQmax = speedModeIQmax;
    ptr_bat_STM32MCPDArray->ramp_rate = rampRate;
    ptr_bat_STM32MCPDArray->allowable_rpm = allowableRPM;

    /* call to update and execute speed mode change on MCU */
//   motor_control_speedModeParamsChg();   // Note STM32MCP functions have been commented out for debugging purposes
    motor_control_changeSpeedMode();

    /* updates led display */
    led_display_setSpeedMode(speedMode);    // update led display

    /******  Dashboard services *************************************/
    /* updates speed mode Characteristic Value -> Mobile App */
    bat_dashboard_speedmode_service();

    return (speedMode);
}

/*********************************************************************
 * @fn      brake_and_throttle_setSpeedMode
 *
 * @brief   To set the speed mode of the escooter
 *
 * @param   speedMode - the speed mode of the escooter
 *
 * @return  none
 */
void brake_and_throttle_setSpeedMode(uint8_t speed_mode)
{
    speedMode = speed_mode;
}

/*********************************************************************
 * @fn      brake_and_throttle_getSpeedMode
 *
 * @brief   To get the speed mode of the escooter
 *
 * @param   none
 *
 * @return  the speedmode of the escooter
 */
uint8_t brake_and_throttle_getSpeedMode()
{
    return (speedMode);
}

/*********************************************************************
 * @fn      brake_and_throttle_getControlLaw
 *
 * @brief   call this function to retrieve the current control law
 *
 * @param   None
 *
 * @return  ControlLaw
 *********************************************************************/
extern uint8_t brake_and_throttle_getControlLaw()
{
    return (ControlLaw);
}

/*********************************************************************
 * @fn      brake_and_throttle_setControlLaw
 *
 * @brief   call this function to set Control Law
 *
 * @param   UnitSelectDash
 *
 * @return  none
 *********************************************************************/
extern void brake_and_throttle_setControlLaw(uint8_t control_law)
{
    ControlLaw = control_law;
}

/*********************************************************************
 * @fn      brake_and_throttle_getThrottlePercent
 *
 * @brief   To get the throttle percentage of the escooter
 *
 * @param   none
 *
 * @return  the throttle percentage of the escooter
 */
uint16_t brake_and_throttle_getThrottlePercent()
{
    return (throttlePercent);
}

/*********************************************************************
 * @fn      brake_and_throttle_getBrakePercent
 *
 * @brief   To get the brake percentage of the escooter
 *
 * @param   none
 *
 * @return  the brake percentage of the escooter
 */
uint16_t brake_and_throttle_getBrakePercent()
{
    return (brakePercent);
}

/*********************************************************************
 * @fn      bat_dashboardErrorCodePriorityRegister
 *
 * @brief   Return the point to dashboardErrorCodePriority to the calling function
 *
 * @param   none
 *
 * @return  &dashboardErrorCodePriority
 */
extern uint8_t* bat_dashboardErrorCodePriorityRegister()
{
    return (&dashboardErrorCodePriority);
}


extern uint8_t bat_auxiliaryLightStatusRegister(uint8_t *ptrAuxiliaryLightStatus)
{
    ptr_bat_auxiliaryLightStatus = ptrAuxiliaryLightStatus;
    if (speedMode == BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE){
        *ptr_bat_auxiliaryLightStatus = 1;
    }
    else {
        *ptr_bat_auxiliaryLightStatus = 0;
    }
    return (*ptr_bat_auxiliaryLightStatus);
}

/*********************************************************************
 * @fn      bat_powerOnRegister
 *
 * @brief   call to assign and register the pointer to powerOn
 *
 * @param   a pointer to powerOn, i.e. ptr_powerOn
 *
 * @return  None
 */
extern void bat_powerOnRegister(uint8_t *ptrpowerOn)
{
    ptr_bat_POWER_ON = ptrpowerOn;
}

/*********************************************************************
 * @fn      bat_sendIQZero
 *
 * @brief   call at Power Off to send IQ = 0 to motor controller
 *
 * @param   None
 *
 * @return  None
 */
extern void bat_zeroIQ()
{
    IQ_input = 0;
    IQ_applied = IQ_input;
    /********************************************************************************************************************************
    * Update IQ_applied to STM32MCPDArray.IQ_value for commanding Motor via motor controller
    ********************************************************************************************************************************/
    ptr_bat_STM32MCPDArray->IQ_value = IQ_applied;
}
