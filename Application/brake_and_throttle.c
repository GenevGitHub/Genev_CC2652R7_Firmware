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

#include "Application/brake_and_throttle.h"

#include "Profiles/dashboard_profile.h"
#include "Profiles/controller_profile.h"
#include "Profiles/battery_profile.h"
#include "Profiles/profile_charVal.h"

#include "Application/led_display.h"
#include "Application/data_analytics.h"
#include "Application/snv_internal.h"
#include "Application/periodic_communication.h"

#include "UDHAL/UDHAL_ADC.h"

/*********************************************************************
 * CONSTANTS
 */
#undef  ESCOOTER_RUN
#define ESCOOTER_DEBUG 1
/*********************************************************************
 * GLOBAL VARIABLES
 *********************************************************************/
/* ControlLaw Options: NormalLaw (1) or DirectLaw (0).
 *          Normal law algorithm modulates the speed to not exceed the defined limit
 *          Direct law algorithm does not modulate the speed in any way
 */
uint8_t     ControlLaw = BRAKE_AND_THROTTLE_NORMALLAW; // BRAKE_AND_THROTTLE_NORMALLAW or BRAKE_AND_THROTTLE_DIRECTLAW; //

uint8_t     speedMode = 2;
uint16_t    adcValues[2];

uint16_t    adc2Result;             // adc2Result is a holder of the throttle ADC reading
uint16_t    throttlePercent;        // Actual throttle applied in percentage
uint16_t    throttlePercent0 = 0;
uint16_t    IQ_input;                // Iq value command sent to STM32 / motor Controller
uint16_t    IQ_applied;
uint16_t    rpm_limit = REG_MAXP_RPM;
uint16_t    brakePercent;           // Actual brake applied in percentage
uint16_t    brakeStatus = 0;
//uint16_t    brakeADCAvg;            // declared as global variable for debugging only
//uint16_t    throttleADCAvg;         // declared as global variable for debugging only

uint8_t     dashboardErrorCodeStatus = 0xFF;

/********************************************************************
 *  when brakeAndThrottle_errorMsg is true (=1),
 *  it generally means either (1) brake signal is not connected and/or (2) throttle signal is not connect
 *  IQ_input is set to 0 = zero throttle by default
 */
uint8_t brakeAndThrottle_errorMsg = BRAKE_AND_THROTTLE_NORMAL;
uint8_t throttle_errorStatus = 0;   // 0 indicates normal (no error)
uint8_t brake_errorStatus = 0;      // 0 indicates normal (no error)

// Safety feature 1:  User cannot change speed mode while throttle is pressed

/*********************************************************************
 *
 * LOCAL VARIABLES
 */
profileCharVal_t *ptr_bat_profileCharVal;
static uint8_t   *ptr_charVal;

static uint8_t  state = 0;
static uint8_t         brakeAndThrottleIndex = 0;
static uint16_t brakeADCSamples[BRAKE_AND_THROTTLE_SAMPLES];
uint16_t        throttleADCSamples[BRAKE_AND_THROTTLE_SAMPLES];
//uint16_t        throttleADC_movingAvg[BRAKE_AND_THROTTLE_SAMPLES];
uint16_t        RPM_array[BRAKE_AND_THROTTLE_SAMPLES];

uint8_t         exponent  = 3;
float           speedModulationFactor = 1;
uint32_t        sl_flag = 0;

#define array_size      50

uint8_t  bat_count_array[array_size];
uint16_t IQ_input_array[array_size];
uint16_t IQ_applied_array[array_size];
uint16_t RPM_temp_array[array_size];
float    SMFactor[array_size];

static uint16_t speedModeIQmax;
static uint8_t  reductionRatio;
static uint16_t rampRate;
static uint16_t allowableSpeed;

MCUD_t *ptr_bat_MCUDArray;


/*********************************************************************
* FUNCTIONS
*/

/**** obtain/register the pointer to MCUDArray   ****/
extern void brake_and_throttle_MCUArrayRegister(MCUD_t *ptrMCUDArray)
{
    ptr_bat_MCUDArray = ptrMCUDArray;
}

/**********************************************************************
 *  Local functions
 */
static void brake_and_throttle_getSpeedModeParams();

/*********************************************************************
 * @fun    brakeAndThrottle_motorControl_rpm
 *
 * @brief   Receiving the pointer for BrakeAndThrottleRPM from motorControl.c
 *
 * @param   ptrBrakeAndThrottleRPM
 *
 * @return  Nil
 *********************************************************************/
//uint16_t *ptr_brakeAndThrottle_rpm;
//extern void brake_and_throttle_motorControl_rpm(uint16_t *ptrBrakeAndThrottleRPM)
//{
//    ptr_brakeAndThrottle_rpm = ptrBrakeAndThrottleRPM;
//}

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

    ptr_bat_profileCharVal = profile_charVal_profileCharValRegister();

    UDHAL_ADC_ptrADCValues(&adcValues);
    data_analytics_dashErrorCodeStatusRegister(&dashboardErrorCodeStatus);

    /* Get initial speed mode */
    speedMode = data_analytics_getSpeedmodeInit();

    brake_and_throttle_getSpeedModeParams();

    for (uint8_t ii = 0; ii < BRAKE_AND_THROTTLE_SAMPLES; ii++)
    {
        brakeADCSamples[ii] = BRAKE_ADC_CALIBRATE_L;
        throttleADCSamples[ii] = THROTTLE_ADC_CALIBRATE_L;
//        throttleADC_movingAvg[ii] = THROTTLE_ADC_CALIBRATE_L;
        RPM_array[ii] = 0;
    }

    led_display_setSpeedMode(speedMode);  // update speed mode on dashboard led display

}


/*********************************************************************
 * @fn      brake_and_throttle_ADC_conversion
 *
 * @brief   This function perform ADC conversion
 *          This function is called when timer6 overflows
 *
 * @param
 *********************************************************************/
uint8_t brake_errorFlag = 0;
uint16_t RPM_temp;
uint8_t brakeAndThrottleIndex_minus_1;
uint8_t brakeAndThrottleIndex_minus_2;
uint8_t  throttle_error_count = 0;
uint8_t  brake_error_count = 0;
uint16_t throttleADCsample = 0;  //throttleADCsample
uint16_t brakeADCsample = 0;     //brakeADCsample

uint8_t  array_index = 0;
uint16_t bat_count = 0;

void brake_and_throttle_ADC_conversion()
{
    /************************************************************************************
     *      get brake ADC measurement
     *      get throttle ADC measurement
     *      Stores ADC measurement in arrays brakeADCSamples & throttleADCSamples
     ************************************************************************************/
    UDHAL_ADC_Convert();
    throttleADCsample = adcValues[0];
    brakeADCsample = adcValues[1];

    /*******************************************************************************************************************************
     *      Error Checking
     *      Check whether throttle ADC reading is logical, if illogical, brakeAndThrottle_errorMsg = error (!=0)
     *      These Conditions occur when throttle or brake signals/power are not connected, or incorrect supply voltage
     *      Once this condition occurs (brakeAndThrottle_errorMsg != 0), check throttle connections, hall sensor fault,
     *      Reset (Power off and power on again) is require to reset brakeAndThrottle_errorMsg.
     *******************************************************************************************************************************/
    if ( (throttleADCsample >= THROTTLE_ADC_THRESHOLD_L) && (throttleADCsample <= THROTTLE_ADC_THRESHOLD_H) && (throttle_error_count != 0) )
    {
        throttle_error_count = 0;   // Reset throttle_error_count
    }

    if ( (throttleADCsample < THROTTLE_ADC_THRESHOLD_L) || (throttleADCsample > THROTTLE_ADC_THRESHOLD_H) )
    {
        throttle_error_count++;
        if ((throttle_error_count >= 5) && (!throttle_errorStatus) )
        {
            throttle_errorStatus = 1;
        /* if throttle errorStatus = 1 -> disable throttle input and zero Iq command to Motor Controller Unit */
            led_display_ErrorPriority(THROTTLE_ERROR_PRIORITY);   /* throttle error priority = 11 */

            /******  Dashboard services *************************************/
            /* updates error code Characteristic Value -> Mobile App */
            if (dashboardErrorCodeStatus > THROTTLE_ERROR_PRIORITY)
            {
                dashboardErrorCodeStatus = THROTTLE_ERROR_PRIORITY;
                ptr_charVal = (ptr_bat_profileCharVal->ptr_dash_charVal->ptr_dashErrorCode);
                profile_setCharVal(ptr_charVal, DASHBOARD_ERROR_CODE_LEN, THROTTLE_ERROR_CODE);
            }

        }
    }

    /*******************************************************************************************************************************
     *      Throttle Signal Calibration
     *      Truncates the average throttle ADC signals to within THROTTLE_ADC_CALIBRATE_L and THROTTLE_ADC_CALIBRATE_H
     *******************************************************************************************************************************/
    if( (throttleADCsample > THROTTLE_ADC_CALIBRATE_H))
    { // && (throttleADCsample <= THROTTLE_ADC_THRESHOLD_H)) {
        throttleADCsample = THROTTLE_ADC_CALIBRATE_H;
    }

    if((throttleADCsample < THROTTLE_ADC_CALIBRATE_L))
    {// && (throttleADCsample >= THROTTLE_ADC_THRESHOLD_L)) {
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
    if ((brakeADCsample < BRAKE_ADC_THRESHOLD_L) || (brake_errorFlag != 0) || (brakeADCsample > BRAKE_ADC_THRESHOLD_H))
    {
        /* Adjust Sensitivity */

        brake_error_count++;

        if ((brake_error_count >= 5) && (brake_errorStatus == 0))
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
            if (dashboardErrorCodeStatus > BRAKE_ERROR_PRIORITY)
            {
                dashboardErrorCodeStatus = BRAKE_ERROR_PRIORITY;
                ptr_charVal = (ptr_bat_profileCharVal->ptr_dash_charVal->ptr_dashErrorCode);
                profile_setCharVal(ptr_charVal, DASHBOARD_ERROR_CODE_LEN, BRAKE_ERROR_CODE);
            }
        }
    }

    /*******************************************************************************************************************************
     *      Brake Signal Calibration
     *      Truncates the average brake ADC signals to within BRAKE_ADC_CALIBRATE_L and BRAKE_ADC_CALIBRATE_H
     *******************************************************************************************************************************/
    if((brakeADCsample > BRAKE_ADC_CALIBRATE_H))
    {// && (brakeADCsample <= BRAKE_ADC_THRESHOLD_H)) {
        brakeADCsample = BRAKE_ADC_CALIBRATE_H;
    }

    if((brakeADCsample < BRAKE_ADC_CALIBRATE_L))
    { // && (brakeADCsample >= BRAKE_ADC_THRESHOLD_L)) {
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
    uint8_t     jj;

    for (jj = 0; jj < BRAKE_AND_THROTTLE_SAMPLES; jj++)
    {
        brakeADCsum += brakeADCSamples[jj];
        throttleADCsum += throttleADCSamples[jj];
    }

    /***** Calculate moving average values ******/
    uint16_t    brakeADCAvg = brakeADCsum / BRAKE_AND_THROTTLE_SAMPLES;             // declared as global variable for debugging only
    uint16_t    throttleADCAvg = throttleADCsum / BRAKE_AND_THROTTLE_SAMPLES;
//    throttleADC_movingAvg[ brakeAndThrottleIndex ] = throttleADCAvg;

    /********************************************************************************************************************************
     *  brakePercent is in percentage - has value between 0 - 100 %
     ********************************************************************************************************************************/
    brakePercent = (brakeADCAvg - BRAKE_ADC_CALIBRATE_L) * 100 / (BRAKE_ADC_CALIBRATE_H - BRAKE_ADC_CALIBRATE_L);
    /********************************************************************************************************************************
     *  throttlePercent is in percentage - has value between 0 - 100 %
     ********************************************************************************************************************************/
    throttlePercent = (throttleADCAvg - THROTTLE_ADC_CALIBRATE_L) * 100 / (THROTTLE_ADC_CALIBRATE_H - THROTTLE_ADC_CALIBRATE_L);

    /********************** Brake Power Off Protect State Machine  *******************************************************************************
     *              if brake is engaged, defined as brakePercent being greater than say 50%,
     *              dashboard will instruct motor controller to cut power to motor for safety precautions.
     *              Once power to motor is cut, both the brake & throttle must be fully released before power delivery can be resumed
    **********************************************************************************************************************************************/
    if (brake_errorStatus == 0)
    {        /* if there is no brake error .... */
    /* condition where brake is engaged (1) and throttle is greater than 30% */
        if ((brakeStatus == 1) && (brakePercent <= BRAKEPERCENTTHRESHOLD) && (throttlePercent <= THROTTLEPERCENTTHRESHOLD))
        { // This condition resets brakeStatus to zero
                brakeStatus = 0;
        }
        else if ((brakeStatus == 0) && (brakePercent > BRAKEPERCENTTHRESHOLD))
        {                                                // condition when brake is not initially pressed and rider pulls on the brake
            brakeStatus = 1;    // if brakeStatus == 1, cut power to motor
        }
    }
    else {  // when brake_errorStatus == 1
        brakeStatus = 0;
    }

    /**************************************************************************************
     *       if Brake is engaged -> cut power to motor and activate brake light
     **************************************************************************************/
    if (brakeStatus == 1)
    {

#ifdef CC2640R2_GENEV_5X5_ID
//        STM32MCP_setEscooterControlDebugFrame(STM32MCP_ESCOOTER_BRAKE_PRESS);
        /* send instruction to STM32 to activate brake light */
        /* for regen-brake, send brakePercent to STM32 */
#endif  // CC2640R2_GENEV_5X5_ID

    }
    else { /* when Brake is not engaged */
//        STM32MCP_setEscooterControlDebugFrame(STM32MCP_ESCOOTER_BRAKE_RELEASE);
        /* brake light off */
        /* regen-brake off */
    }
//
    /******** Get RPM from mcu  ******************/
    RPM_temp = ptr_bat_MCUDArray->speed_rpm;           //
//
    /******** Throttle Error Safety Protocol -> when throttle error detected, IQ_input is set to zero
     *  Calculating the IQ Value
     *  Notes: Power is delivered to the motor if:
     *   (1) brake is not engaged
     *   (2) RPM is above the REG_MINP_RPM
     *  These features are for safety reasons
     **********************************************************************************/
    if (throttle_errorStatus == 0)
    {
        if ((brakeStatus == 1)||(RPM_temp < REG_MINP_RPM))
        {
        /*The E-Scooter Stops*/
        /*DRIVE_START = 0 --> Then we don't need to send dynamic Iq messages to the motor controller in order to relieve the UART loads*/
#ifdef ESCOOTER_RUN
            IQ_input = 0;
#endif

#ifdef ESCOOTER_DEBUG
            /*The E-Scooter Starts*/
            /*DRIVE_START = 1 --> The We could send dynamic Iq messages to the motor controller */
            IQ_input = speedModeIQmax * throttlePercent / 100;
#endif
        }
        else
        {
        /*The E-Scooter Starts*/
        /*DRIVE_START = 1 --> The We could send dynamic Iq messages to the motor controller */
            IQ_input = speedModeIQmax * throttlePercent / 100;
        }
    }
    else
    {
        IQ_input = 0;
    }

    /***** Normal Law: modulated IQ_input  *********************/
    if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
    {
        brake_and_throttle_normalLawControl();
    }
    else
    {
        IQ_applied = IQ_input;
    }

    /***** End Normal Law *************************************/

    /***  For debugging  ***/
    IQ_input_array[ array_index ] = IQ_input;
    IQ_applied_array[ array_index ] = IQ_applied;
    RPM_temp_array[ array_index ] = RPM_temp;
    bat_count_array[ array_index ] = bat_count++;
    SMFactor[ array_index ] = speedModulationFactor;

    /********************************************************************************************************************************
     * Send the throttle signal to STM32 Motor Controller
     ********************************************************************************************************************************/
    // in "brakeAndThrottle_CB(allowableSpeed, IQ_input, brakeAndThrottle_errorMsg)", brakeAndThrottle_errorMsg is sent to the motor control unit for error handling if necessary.
    // Add one more conditions in order to fed the power into the motor controller
    // if DRIVE_START == 1 -> then run the command for dynamic Iq, otherwise: ignore it!
#ifdef CC2640R2_GENEV_5X5_ID
//    brakeAndThrottle_CBs -> brakeAndThrottle_CB(allowableSpeed, IQ_applied, brakeAndThrottle_errorMsg);
#endif

    /******  Sends brake signal to the controller for tail light toggling  *****/
    //Add codes here

    /******  Send brake percentage for Regen Brake....  *****/
    //Add codes here


    // ***** Increments brakeAndThrottleIndex by 1
    brakeAndThrottleIndex++;
    if (brakeAndThrottleIndex >= BRAKE_AND_THROTTLE_SAMPLES)
    {
        brakeAndThrottleIndex = 0;
    }

    array_index++;
    if (array_index >= array_size)
    {
        array_index = 0;
    }

}

/*********************************************************************
 * @fun    brake_and_throttle_normalLawControl
 *
 * @brief   Normal Law Algorithm
 *
 * @param   Nil
 *
 * @return  Nil
 *********************************************************************/
uint32_t normalLaw_count = 0;

static void brake_and_throttle_normalLawControl()
{
    uint16_t RPM_0;
    if (RPM_temp < 1)
    {
        RPM_0 = 1;
    }
    else
    {
        RPM_0 = RPM_temp;
    }
    speedModulationFactor = pow((rpm_limit/(float)RPM_0), exponent);

    if (IQ_input == 0)
    {
        IQ_applied = IQ_input;
    }
    else // (IQ_input > 0)    // i.e. (IQ_input != 0)
    {
        if (( RPM_temp < rpm_limit ) && (!sl_flag))
        {
            IQ_applied = IQ_input;

        }
        else if (( RPM_temp >= rpm_limit ) && (!sl_flag))
        {
            IQ_applied = (float)speedModulationFactor * IQ_input;
            sl_flag = 1;

        }
        else if (( RPM_temp >= rpm_limit ) && (sl_flag))
        {
            IQ_applied = (float)speedModulationFactor * IQ_applied;

        }
        else if ((RPM_temp < rpm_limit) && (sl_flag))    // when (RPM_temp >= rpm_limit), speedModulationFactor is less than or equal to 1.
        {
            IQ_applied = IQ_input * 0.5;
            sl_flag = 0;

        }

        if (IQ_applied >= IQ_input)
        {
            IQ_applied = IQ_input;
        }

    }

    normalLaw_count++;
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
void brake_and_throttle_getSpeedModeParams()
{
    switch(speedMode)
    {
    case BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE:                       // Amble mode
        {
            reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_AMBLE;
            speedModeIQmax = reductionRatio * BRAKE_AND_THROTTLE_TORQUEIQ_MAX / 100;
            rampRate = BRAKE_AND_THROTTLE_RAMPRATE_AMBLE;
            allowableSpeed = BRAKE_AND_THROTTLE_MAXSPEED_AMBLE;
            break;
        }
    case BRAKE_AND_THROTTLE_SPEED_MODE_LEISURE:                 // Leisure mode
        {
            reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_LEISURE;
            speedModeIQmax = reductionRatio * BRAKE_AND_THROTTLE_TORQUEIQ_MAX / 100;
            rampRate = BRAKE_AND_THROTTLE_RAMPRATE_LEISURE;
            allowableSpeed = BRAKE_AND_THROTTLE_MAXSPEED_LEISURE;
            break;
        }
    case BRAKE_AND_THROTTLE_SPEED_MODE_SPORTS:                  // Sports mode
        {
            reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS;
            speedModeIQmax = reductionRatio * BRAKE_AND_THROTTLE_TORQUEIQ_MAX / 100;
            rampRate = BRAKE_AND_THROTTLE_RAMPRATE_SPORTS;
            allowableSpeed = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS;
            break;
        }
    default:
        break;
    }
}

/*********************************************************************
 * @fn      brake_and_throttle_toggleSpeedMode
 *
 * @brief   To change / toggle the speed Mode of the e-scooter
 *
 * @param   none
 *
 * @return  none
 */
uint8_t brake_and_throttle_toggleSpeedMode()
{
    if (brake_errorStatus == 0)
    {
        if (throttleADCsample <= THROTTLE_ADC_CALIBRATE_L)                                    // Only allow speed mode change when no throttle is applied - will by-pass if throttle is applied
        {
            if(speedMode == BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE)                       // Amble mode to Leisure mode
            {
                speedMode = BRAKE_AND_THROTTLE_SPEED_MODE_LEISURE;
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_LEISURE;
                speedModeIQmax = reductionRatio * BRAKE_AND_THROTTLE_TORQUEIQ_MAX / 100;
                rampRate = BRAKE_AND_THROTTLE_RAMPRATE_LEISURE;
                allowableSpeed = BRAKE_AND_THROTTLE_MAXSPEED_LEISURE;
            }
            else if(speedMode == BRAKE_AND_THROTTLE_SPEED_MODE_LEISURE)                 // Leisure mode to Sports mode
            {
                speedMode = BRAKE_AND_THROTTLE_SPEED_MODE_SPORTS;
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS;
                speedModeIQmax = reductionRatio * BRAKE_AND_THROTTLE_TORQUEIQ_MAX / 100;
                rampRate = BRAKE_AND_THROTTLE_RAMPRATE_SPORTS;
                allowableSpeed = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS;
            }
            else if(speedMode == BRAKE_AND_THROTTLE_SPEED_MODE_SPORTS)                  // Sports mode back to Amble mode
            {
                speedMode = BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE;
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_AMBLE;
                speedModeIQmax = reductionRatio * BRAKE_AND_THROTTLE_TORQUEIQ_MAX / 100;
                rampRate = BRAKE_AND_THROTTLE_RAMPRATE_AMBLE;
                allowableSpeed = BRAKE_AND_THROTTLE_MAXSPEED_AMBLE;
            }

            /* Send updated speed mode parameters to motor control unit */
//            motorcontrol_speedModeChgCB(speedModeIQmax, allowableSpeed, rampRate);
        }
    }
    else
    {
        if(speedMode != BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE)        // This condition prevents unnecessary repetitive changes that does nothing
        {
            speedMode = BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE;
            reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_AMBLE;
            speedModeIQmax = reductionRatio * BRAKE_AND_THROTTLE_TORQUEIQ_MAX / 100;
            rampRate = BRAKE_AND_THROTTLE_RAMPRATE_AMBLE;
            allowableSpeed = BRAKE_AND_THROTTLE_MAXSPEED_AMBLE;

            /* Send updated speed mode parameters to motor control unit */
//            motorcontrol_speedModeChgCB(speedModeIQmax, allowableSpeed, rampRate);

        }
    }
    /* updates led display */
    led_display_setSpeedMode(speedMode);    // update led display

    /******  Dashboard services *************************************/
    /* updates speed mode Characteristic Value -> Mobile App */
    ptr_charVal = (ptr_bat_profileCharVal->ptr_dash_charVal->ptr_speedMode);
    profile_setCharVal(ptr_charVal, DASHBOARD_SPEED_MODE_LEN, speedMode);

    return speedMode;
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
void brake_and_throttle_setSpeedMode(uint8_t speed_Mode)
{
    speedMode = speed_Mode;
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
    return speedMode;
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
extern void brake_and_throttle_setControlLaw(uint8_t newControlLaw)
{
    ControlLaw = newControlLaw;
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
 * @fn      bat_dashboardErrorCodeStatusRegister
 *
 * @brief   Return the point to dashboardErrorCodeStatus to the calling function
 *
 * @param   none
 *
 * @return  &dashboardErrorCodeStatus
 */
extern uint8_t* bat_dashboardErrorCodeStatusRegister()
{
    return (&dashboardErrorCodeStatus);
}
