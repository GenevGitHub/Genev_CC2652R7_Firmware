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
#include "Application/general_purpose_timer.h"
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
static bool     *ptr_bat_POWER_ON;

uint8_t     speedMode;
uint8_t     shutDownReady = 0;

uint16_t    adcValues[2];
uint16_t    adc2Result;             // adc2Result is a holder of the throttle ADC reading
uint16_t    throttlePercent = 0;        // Actual throttle applied in percentage
uint16_t    throttlePercent0 = 0;
uint16_t    throttlePercentApplied = 0; // throttle percentage applied to the motor
uint16_t    throttlePercentApplied_x100 = 0;
uint16_t    IQ_input = 0;               // Iq value input by rider
uint32_t    IQ_input_x10 = 0;
uint16_t    IQ_applied = 0;             // Iq value command sent to STM32 / motor Controller
uint16_t    brakePercent = 0;           // brake applied in percentage
uint8_t     brakeStatus = 0;
uint16_t    throttleRPMdemand = 0;
uint16_t    throttleRPMdemand0 = 0;
uint16_t    startIQPercent = 0;
float       deltaIQ_x10 = 0;

static uint16_t throttlePercentIncrement_x100 = 0; // declare here for debugging only
uint8_t motor_state = THROTTLE_CONTROL_STATE;    // motor_state is a flags that indicates whether RPM was below min RPM or brake was engaged
uint8_t motor_state_counter = 0;
float K_acc = 0;
uint8_t startThrottlePercent = 0;

uint8_t     dashboardErrorCodePriority = SYSTEM_NORMAL_PRIORITY;

/********************************************************************
 *  when brakeAndThrottle_errorMsg is true (=1),
 *  it generally means either
 *  (1) brake signal is not connected and/or
 *  (2) throttle signal is not connect
 *
 *  IQ_input is ramped down to 0
 */
static uint8_t  brakeAndThrottle_errorMsg = BRAKE_AND_THROTTLE_NORMAL;
static uint8_t  throttle_errorStatus = 0;   // 0 indicates normal (no error)
static uint8_t  brake_errorStatus = 0;      // 0 indicates normal (no error)

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
static profileCharVal_t *ptr_bat_profileCharVal;
static uint8_t  *ptr_charVal;
static uint8_t  *ptr_bat_errorPriority;
static uint8_t  *ptr_bat_auxiliaryLightStatus;

static uint8_t  brakeAndThrottleIndex = 0;
static uint8_t  ThrottleIndex = 0;
static uint8_t  RPM_index = 0;

static uint16_t brakeADCSamples[BRAKE_AND_THROTTLE_SAMPLES];
static uint16_t throttleADCSamples[BRAKE_AND_THROTTLE_SAMPLES];
static uint16_t RPMarray[RPM_SAMPLES] = {0};    // for computing acceleration
static uint16_t IQarray[RPM_SAMPLES] = {0};

/** Speed limiter control variables **/
static uint16_t IQ_increment_x10;    // max IQ increment
float           K_Limit = 0;    // 0 <= K_Limit < 1
float           K_prop = 0;     //  -ve to +ve

/**  Speed mode parameters  **/
uint16_t        speedModeIQmax;
static uint8_t  reductionRatio;
static uint16_t rampRate;
       uint16_t speedModeRPMmax;
       uint16_t allowableRPM;
       uint16_t IQ_maxPout = 0xFFFF;
       uint16_t throttlePercentArray[THROTTLE_INDEX_SAMPLES] = {0};
       uint16_t throttleRPMdemandArray[THROTTLE_INDEX_SAMPLES] = {0};
static uint8_t  N_rpm = 1;
static uint16_t RPM_prev = 0;
static uint16_t IQ_prev = 0;
       float    acceleration = 0;
       float    accelerationThreshold = 0;

/**  Data structs  **/
static MCUD_t   *ptr_bat_MCUDArray;
static AD_t     *ptr_bat_ADArray;

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
    allowableRPM = speedModeRPMmax;
    for (uint8_t ii = 0; ii < BRAKE_AND_THROTTLE_SAMPLES; ii++) {
        brakeADCSamples[ii] = BRAKE_ADC_CALIBRATE_L;
        throttleADCSamples[ii] = THROTTLE_ADC_CALIBRATE_L;
    }

    for (uint8_t ll = 0; ll < RPM_SAMPLES; ll++) {
        RPMarray[ll] = 0;
        IQarray[ll] = 0;
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
static uint8_t     brake_errorFlag = 0;
       uint16_t    RPM_temp;
static uint8_t     brakeAndThrottleIndex_minus_1;
static uint8_t     brakeAndThrottleIndex_minus_2;
static uint8_t     throttle_error_count = 0;
static uint8_t     brake_error_count = 0;
static uint16_t    throttleADCsample = 0;  //throttleADCsample
static uint16_t    brakeADCsample = 0;     //brakeADCsample
 float       K_volt = 1;
static int         changeInIQ = 0;

float debug_expression_1 = 0;   // for debugging
float debug_expression_3 = 0;   // for debugging
uint8_t debug_expression_2 = 0;   // for debugging

/***    This is the main function of brake_and_throttle.c
 *      This function
 *          reads the brake and throttle ADC values,
 *          checks the brake and throttle values and range for abnormality and errors
 *          calculates throttle and brake percentage
 *          Calculate and modulate the IQ values
 ********************************************************************************/
uint8_t brake_and_throttle_ADC_conversion(uint32_t gptCounter)
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
     *      Error Checking and throttle error protocol
     *      Check whether throttle ADC reading is logical, if illogical, brakeAndThrottle_errorMsg = error (!=0)
     *      These Conditions occur when throttle or brake signals/power are not connected, or incorrect supply voltage
     *      Once this condition occurs (brakeAndThrottle_errorMsg != 0), check throttle connections, hall sensor fault,
     *      Reset (Power off and power on again) is require to reset brakeAndThrottle_errorMsg.
     *******************************************************************************************************************************/
    if ( (throttleADCsample >= THROTTLE_ADC_THRESHOLD_L) && (throttleADCsample <= THROTTLE_ADC_THRESHOLD_H))
    {
        if (throttle_error_count != 0) {
            throttle_error_count = 0;   // Reset throttle_error_count
        }
    }
    else    //    if ( (throttleADCsample < THROTTLE_ADC_THRESHOLD_L) || (throttleADCsample > THROTTLE_ADC_THRESHOLD_H) )
    {
        throttle_error_count++;
        if ((throttle_error_count >= 10) && (!throttle_errorStatus) )
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
     *      Error Checking and brake error protocol
     *      Check whether brake ADC reading is logical, if illogical, brakeAndThrottle_errorMsg = error (!=0)
     *      These Conditions occur when brake signals/power are not connected, or incorrect supply voltage
     *      Once this condition occurs (brakeAndThrottle_errorMsg != 0), check brake connections, hall sensor fault,
     *      Reset (Power off and power on again) is require to reset brakeAndThrottle_errorMsg.
     *******************************************************************************************************************************/
    if ( (brakeADCsample >= BRAKE_ADC_THRESHOLD_L) && (brakeADCsample <= BRAKE_ADC_THRESHOLD_H)) // brake ADC reading is normal
    {
        if (brake_error_count) {
            brake_error_count = 0;   // Reset brake_error_count
        }
    }
    else    //    if brake ADC reading out of range. i.e. abnormal
    {
        /* Adjust Sensitivity */
        brake_error_count++;
        if (brake_error_count >= 10) {
            if (!brake_errorStatus) {
                brake_errorFlag = 1; /* set and flag that brake_errorFlag = 1 */
            }
        }
      /* For safety reason, speed mode will not change immediately while throttle is still pressed. A flag is set to execute error protocol once throttle value is less than THROTTLE_ADC_CALIBRATE_L */
      /* if brake_errorFlag == 1 and throttle is released to or below THROTTLE_ADC_CALIBRATE_L, execute protocol.
       * This routine also prevents repetitive executions if brake error exists */
        if (brake_errorFlag)
        {
            if (throttleADCsample <= THROTTLE_ADC_CALIBRATE_L)  // brake error protocol is only activated when safe to do so.
            {
                brakeAndThrottle_errorMsg = BRAKE_ERROR;
                brake_errorStatus = 1;
               /* When brake error occurs, i.e. brake errorStatus = 1 -> restrict riding in Amble mode Only */
                speedMode = BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE;

               /* Note: speed mode is changed! Update Led Display, update App data */
                brake_and_throttle_toggleSpeedMode();   // execute speed mode change to Amble mode
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
     *      calculate  moving average brake and throttle ADC values
     *
     *******************************************************************************************************************************/
    uint16_t    brakeADCsum = 0;
    uint16_t    throttleADCsum = 0;

    for (uint8_t jj = 0; jj < BRAKE_AND_THROTTLE_SAMPLES; jj++)
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
    brakePercent = (brakeADCAvg - BRAKE_ADC_CALIBRATE_L) * 99 / (BRAKE_ADC_CALIBRATE_H - BRAKE_ADC_CALIBRATE_L);  // max 99%
    if (brakePercent > 99) {
        brakePercent = 99;
    }        // percentage must never be greater than 100%
    throttlePercent = (throttleADCAvg - THROTTLE_ADC_CALIBRATE_L) * 99 / (THROTTLE_ADC_CALIBRATE_H - THROTTLE_ADC_CALIBRATE_L); // max 99%
    if (throttlePercent > 99) {
        throttlePercent = 99;
    }  // percentage must never be greater than 100%

    /**** throttleRPMdemand correlates throttlePercent with targeted Speed   **********************/
    throttleRPMdemand = throttlePercent * (allowableRPM - (REG_MINP_RPM / 2)) / 100 + (REG_MINP_RPM / 2);// this equation sets the throttleRPMdemand value at 40 rpm instead of 80 rpm when throttle% is 0

    /*****  update brakeAndThrottle_errorMsg on STM32MCPDArray.error_msg  *****/
    ptr_bat_STM32MCPDArray->error_msg = brakeAndThrottle_errorMsg;

    /********************** Brake Power Cut Off Protect State Machine  *******************************************************************************
     *              Brake is considered engaged when brakePercent is greater than BRAKEPERCENTTHRESHOLD,
     *              dashboard will instruct motor controller to cut power to motor.
     *              Once power to motor is cut, both the brake & throttle must be fully released before power delivery can be resumed
    **********************************************************************************************************************************************/
    if (!brake_errorStatus) {   /* if there is no brake error .... */
     /* condition where brake is engaged (1) and throttle is greater than 30% */
        if ((brakeStatus) && (brakePercent <= BRAKEPERCENTTHRESHOLD)) //&& (throttlePercent <= THROTTLEPERCENTTHRESHOLD))
        { // This condition resets brakeStatus to zero
            brakeStatus = 0;
        }
        else if ((!brakeStatus) && (brakePercent > BRAKEPERCENTTHRESHOLD))
        {// condition when brake is not initially pressed and rider pulls on the brake
            brakeStatus = 1;    // if brakeStatus == 1, cut power to motor
        }
    }
    else {  // when brake_errorStatus == 1, we cannot determine whether the brake lever is pulled or not -> handling -> set brake status = 0 at all times.
        brakeStatus = 0;    // if we set brakeStatus = 1 when brake error exists, IQ value sent to motor is set to zero.
                            // if we set brakeStatus = 0 when brake error exists, command to motor remains normal.
    }

    /**************************************************************************************
     *       Send / update brake% and brake status in STM32MCPDArray
     **************************************************************************************/
    ptr_bat_STM32MCPDArray->brake_percent = brakePercent;
    ptr_bat_STM32MCPDArray->brake_status = brakeStatus;

    /******** Get RPM from mcu  ******************/
    RPM_temp = ptr_bat_MCUDArray->speed_rpm;            //

    /*** K_volt augment the max IQ and IQ increments based on the instant battery voltage
     *   K_volt reduces IQ increment and max IQ when battery voltage is greater than the nominal value
     *   K_volt reduces the IQ increment and max IQ when battery voltage is below the nominal value to
     *   prevent the battery from draining out too quickly ***/
    if (ptr_bat_MCUDArray->bat_voltage_mV > BATTERY_NOMINAL_VOLTAGE) {  // battery voltage is grater than nominal voltage
        K_volt = (float) BATTERY_NOMINAL_VOLTAGE / ptr_bat_MCUDArray->bat_voltage_mV;
        allowableRPM = speedModeRPMmax;
    }
    else {
        if (ptr_bat_MCUDArray->bat_voltage_mV < BATTERY_MIN_VOLTAGE) {
            K_volt = 0;
        }
        else {
            K_volt = (float) (ptr_bat_MCUDArray->bat_voltage_mV - BATTERY_MIN_VOLTAGE) / (BATTERY_NOMINAL_VOLTAGE - BATTERY_MIN_VOLTAGE);
            K_volt = pow(K_volt, 1.2);
        }
        allowableRPM = speedModeRPMmax;//K_volt * (speedModeRPMmax - REG_MINP_RPM) + REG_MINP_RPM;
    }

    /****  instantaneous acceleration calculation  ****/
    RPM_prev = RPMarray[RPM_index];
    RPMarray[RPM_index] = RPM_temp;
    acceleration = (float)(RPM_temp - RPM_prev) * 1000 / (N_rpm * GPT_TIME);    // in rpm per second
    if (N_rpm < RPM_SAMPLES) {   // TRUE only for the 1st RPM_SAMPLE number of measurements
        N_rpm++;
    }

    /******** Throttle Error Safety Protocol -> when throttle error detected or speed is negative, IQ_input is set to zero
     *  Calculating the IQ Value
     *  Notes: Power is delivered to the motor if:
     *   (1) brake is not engaged
     *   (2) RPM is not negative or is greater than the REG_MINP_RPM
     *   (3) no detected critical hardware / firmware errors
     *  These features are for safety reasons
     **********************************************************************************/
    /********  if rpm is positive       and    if error is not fatal or critical errors     and     POWER ON      and   gpt count > 20 **/
    if ((ptr_bat_MCUDArray->rpm_status) && (*ptr_bat_errorPriority >= BRAKE_ERROR_PRIORITY) && (*ptr_bat_POWER_ON) && (gptCounter > 20))
    {
        switch (ControlLaw)
        {
        case (BRAKE_AND_THROTTLE_DIRECTLAW):
        {
            brake_and_throttle_directLawAlgorithm();
            brake_and_throttle_OutputPowerLimitAlgorithm();
            break;
        }
        case (BRAKE_AND_THROTTLE_NORMALLAW):
        {
            brake_and_throttle_normalLawAlgorithm();
            brake_and_throttle_OutputPowerLimitAlgorithm();
            break;
        }
        default:
            break;
        }

    }
    else {  /** if rpm is negative or if error is fatal or critical errors or powered-off  **/
        // Provides a soft landing for IQ_input to zero
        if (IQ_input_x10 < (float) speedModeIQmax * 10 / 50) {
            IQ_input_x10 = 0;
        }
        else {
            IQ_input_x10 -= (float) speedModeIQmax * 10 / 50; // provide a soft landing towards 0 when IQ_input is not already zero
        }
        IQ_input = ((float) IQ_input_x10 / 10);   // synchronize IQ_input

        if (!(*ptr_bat_POWER_ON)) {
            if (!IQ_input_x10) {
                shutDownReady = 1;  // if power = off & IQ_input reduced to Zero -> shutdown is ready
            }
        }
    }

    IQ_prev = IQarray[RPM_index];
    IQarray[RPM_index] = IQ_input;
    changeInIQ = IQ_input - IQ_prev;

    IQ_applied = IQ_input;

    /********************************************************************************************************************************
     * Update IQ_applied to STM32MCPDArray.IQ_value fot commanding Motor via motor controller
     ********************************************************************************************************************************/
    ptr_bat_STM32MCPDArray->IQ_value = IQ_applied;

    /***** Increments brakeAndThrottleIndex  ***/
    brakeAndThrottleIndex++;
    if (brakeAndThrottleIndex >= BRAKE_AND_THROTTLE_SAMPLES) {
        brakeAndThrottleIndex = 0;
    }

    throttlePercent0 = throttlePercentArray[ThrottleIndex];
    throttlePercentArray[ThrottleIndex] = throttlePercent;
    throttleRPMdemand0 = throttleRPMdemandArray[ThrottleIndex];
    throttleRPMdemandArray[ThrottleIndex] = throttleRPMdemand;

    /***** Increments ThrottleIndex  ***/
    ThrottleIndex++;
    if (ThrottleIndex >= THROTTLE_INDEX_SAMPLES) {
        ThrottleIndex = 0;
    }

    RPM_index++;
    if (RPM_index >= RPM_SAMPLES) { // reset RPM_index to 0 once RPM_index reaches the end of the array
        RPM_index = 0;
    }
    return (brakeStatus);
}

/*********************************************************************
 * @fun    brake_and_throttle_directLawAlgorithm
 *
 * @brief   Normal Law Algorithm - uses (1-cos(x)^(0.8)) profile
 *
 * @param   Nil
 *
 * @return  Nil
 *********************************************************************/
uint8_t ThrottleReleasedFlag = 0;

static void brake_and_throttle_directLawAlgorithm()
{
    /** To get here,
     * (1) RPM status is 1, i.e., RPM is positive
     * (2) No fatal errors
     ***/
    if ((brakeStatus) || (RPM_temp < REG_MINP_RPM)) {/* if brake is applied or rpm is less than the min power rpm */
        // Set IQ = 0, but never set throttlePercentApplied (IQ) to zero instantly.
        // Always provide a soft landing to throttlePercentApplied (IQ) from non-zero value to zero.
        // The following routine provides a soft landing by decreasing throttlePercentApplied at increments.
        if (throttlePercentApplied_x100) {  // if throttle Applied is greater than zero
            if(throttlePercentApplied_x100 < THROTTLE_INCREMENT_LIMIT * 100) {
                throttlePercentApplied_x100 = 0;
            }
            else {  // providing a soft landing to throttlePercentApplied from non-zero value to zero
                throttlePercentApplied_x100 -= THROTTLE_INCREMENT_LIMIT * 100;
            }
        }
        throttlePercentApplied = ((float)throttlePercentApplied_x100 / 100);

        if (brakeStatus) {// when brake is applied, motor controller enters the "brake start" state.
            motor_state = BRAKESTART_STATE;
        }
        if (RPM_temp < REG_MINP_RPM) {// when rpm < min rpm, motor controller enters the "kick start" state, i.e. motor_state = 0
            motor_state = KICKSTART_STATE;
        }
        motor_state_counter = 0;
    }
    else {
        if ((throttlePercent > 3) && (throttlePercent > throttlePercentApplied)) {
            /*** Kick start state and Brake start state ***/
            if ((motor_state == KICKSTART_STATE) || (motor_state == BRAKESTART_STATE)) {  //this statement is TRUE if motor is in a kick start or brake start states
                if (motor_state == KICKSTART_STATE) {
                    startThrottlePercent = MIN_KICK_START_THROTTLE_PERCENT;
                }
                if (motor_state == BRAKESTART_STATE) {
                    startThrottlePercent = MIN_BRAKE_START_THROTTLE_PERCENT;
                }
                if (!motor_state_counter) {
                    if (throttlePercentApplied_x100 < startThrottlePercent * 100) {
                        // If starting IQ is too small, motor will not start.
                        throttlePercentApplied_x100 = startThrottlePercent * 100;     // minimum starting throttle Percent is x%
                    }
                    throttlePercentApplied =  ((float)throttlePercentApplied_x100 / 100);
                    motor_state_counter = 1;
                }
                else {
                    if (acceleration > (float) 0.75 * accelerationThreshold) { // if acceleration is sufficient, no need to increment IQ
                        motor_state_counter++;
                        if (motor_state_counter >= 5) {  // when acceleration is sufficient for 5 consecutive times -> exit kick start and brake start states
                            motor_state = THROTTLE_CONTROL_STATE;            // Exit kick start state or brake start state
                        }
                    }
                    else {  // if acceleration is not sufficient, increment IQ
                        if (throttlePercent > throttlePercentApplied) {
                            if (throttlePercentApplied_x100 + 170 > 9900) {// 150 = 1.5% x 100
                                throttlePercentApplied_x100 = 9900; // must never exceed 100%
                                motor_state = THROTTLE_CONTROL_STATE;        // If we reached this point, exit kick start and brake start states
                            }
                            else {
                                throttlePercentApplied_x100 += 170;     // 150 = 1.5% x 100 -> increment by 15% every loop
                            }
                        }
                        throttlePercentApplied = ((float)throttlePercentApplied_x100 / 100);
                        // set motor_state_counter = 1, this ensures kick start or brake start states will only exit when sufficient acceleration occurred 5 consecutive times
                        motor_state_counter = 1;
                    }
                }
            }
            /*** Throttle control state - Not in Brake start or Kick start state ***/
            else {  // this statement is TURE when NOT in kick start mode or brake start mode
                if (acceleration < 0.25 * accelerationThreshold) {
                    K_acc = 4;// this factor is equal to 1 divided by the factor in (acceleration >= 0.5 * accelerationThreshold)
                }
                else {//if (acceleration >= 0.5 * accelerationThreshold) {
                    K_acc = accelerationThreshold / acceleration; // this factor moderates throttle% increment depending on acceleration
                }
                /**     Inverse profile     **/
                //K_prop = REG_MINP_RPM / (float)(RPM_temp); // this factor moderates throttle% increment depending on RPM
                /****   Cosine profile K_prop  ****/
                if (RPM_temp <= REG_MINP_RPM) {
                    K_prop = 1;
                }
                else if ((RPM_temp > REG_MINP_RPM) && (RPM_temp <= allowableRPM)) {
                    K_prop = cos(PI_CONSTANT * (float) (RPM_temp - REG_MINP_RPM) / (allowableRPM - REG_MINP_RPM) / 2);
//                    K_prop = pow(K_prop, 1);
                }
                else {
                    K_prop = 0;
                }

                uint16_t throttlePercentIncrement_x100 = ((float) K_prop * 100 * K_acc * THROTTLE_INCREMENT_LIMIT);

                if ((throttlePercent - throttlePercentApplied) * 100 > throttlePercentIncrement_x100)
                {   // limiting the maximum IQ increment per loop
                    if (throttlePercentApplied_x100 + throttlePercentIncrement_x100 > 9900) { // 99% x 100
                        throttlePercentApplied_x100 = 9900; // 99% x 100
                    }
                    else {
                        throttlePercentApplied_x100 += throttlePercentIncrement_x100;
                        if (ThrottleReleasedFlag == 1) {
                            if (throttlePercentApplied_x100 < MIN_BRAKE_START_THROTTLE_PERCENT * 100) {
                                throttlePercentApplied_x100 = MIN_BRAKE_START_THROTTLE_PERCENT * 100;
                            }
                            ThrottleReleasedFlag = 0;
                        }
                    }
                }
                else {
                    throttlePercentApplied_x100 = throttlePercent * 100;
                }
                throttlePercentApplied = ((float) throttlePercentApplied_x100 / 100);
            }
        }
        else {// when (throttlePercent <= 3) && (throttlePercent < throttlePercentApplied) -> decrease IQ
            ThrottleReleasedFlag = 1;

            if (RPM_temp > 500) {        // deceleration varies depending on RPM
                K_prop = 1.3;
            }
            else if ((RPM_temp <= 500) && (RPM_temp > 265)) {
                K_prop = 1;
            }
            else {
                K_prop = 0.7;
            }
            uint16_t throttlePercentIncrement_x100 = ((float) K_prop * 100* THROTTLE_INCREMENT_LIMIT);
            if ((throttlePercentApplied - throttlePercent) * 100 > throttlePercentIncrement_x100) {
                if ( throttlePercentIncrement_x100 > throttlePercentApplied_x100) {
                    throttlePercentApplied_x100 = 0;
                }
                else {
                    throttlePercentApplied_x100 -= throttlePercentIncrement_x100;
                }
            }
            else {
                throttlePercentApplied_x100 = throttlePercent * 100;
            }
            throttlePercentApplied = ((float)throttlePercentApplied_x100 / 100);
        }
    }

/****    throttle percentage error protocol    ****/
    throttleErrorHandler();
    if (throttle_errorStatus) {
        return;
    }
    // IQ_input is a nonlinear function of throttlePercentApplied - enabling higher control precision at low speeds
    IQ_input_x10 = (float) speedModeIQmax * 10 * K_volt * (1 - pow(cos(PI_CONSTANT * (float) throttlePercentApplied / 200), DIRECT_MODE_IQ_EXPONENT));
    IQ_input = ((float) IQ_input_x10 / 10);
}

/*********************************************************************
 * @fun    brake_and_throttle_normalLawAlgorithm
 *
 * @brief   Normal Law Algorithm - use an adaptive acceleration control algorithm
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
    if ((brakeStatus) || (RPM_temp < REG_MINP_RPM)) {/* if brake is applied or rpm is less than the min power rpm */
        // Never set throttleRPMApplied (IQ) to zero instantly.
        // Always provide a soft landing to throttleRPMApplied (IQ) from non-zero value to zero.
        // The following routine provides a soft landing by decreasing throttleRPMApplied at increments.
        if(IQ_input_x10) {  // if IQ_input_x10 is greater than 0
            if(IQ_input_x10 < (float) speedModeIQmax * MAX_IQ_DECREMENT_PERCENT * 10 / 100) {
                IQ_input_x10 = 0;
            }
            else {  // providing a soft landing to throttlePercentApplied from non-zero value to zero
                IQ_input_x10 -= (float) speedModeIQmax * MAX_IQ_DECREMENT_PERCENT * 10 / 100;  // decrement 2% per computational cycle
            }
        }
        if (brakeStatus) {// when brake is applied, motor controller enters the "brake start" state.
            motor_state = BRAKESTART_STATE;
        }
        if (RPM_temp < REG_MINP_RPM) {// when rpm < min rpm, motor controller enters the "kick start" state, i.e. motor_state = 0
            K_prop = 0;
            K_Limit = 0;
            K_acc = 0;
            motor_state = KICKSTART_STATE;
        }
        motor_state_counter = 0;
    }
    else {
        if ((throttleRPMdemand > REG_MINP_RPM) && (throttleRPMdemand > RPM_temp)) {
            /*** Kick start state and Brake start state ***/
            if ((motor_state == KICKSTART_STATE) || (motor_state == BRAKESTART_STATE)) {  //this statement is TRUE if motor is in a kick start or brake start states
                if (motor_state == KICKSTART_STATE) {
                    startIQPercent = MIN_KICK_START_IQ_PERCENT;
                }
                if (motor_state == BRAKESTART_STATE) {
                    startIQPercent = MIN_BRAKE_START_IQ_PERCENT;    // it's like providing a small burst each time throttle control state resumes
                }
                if (!motor_state_counter) { // if motor_state_counter == 0
                    if (IQ_input_x10 < (float) speedModeIQmax * K_volt * startIQPercent * 10 / 100) {
                        // If starting IQ is too small, motor will not start.
                        IQ_input_x10 = (float) speedModeIQmax * K_volt * startIQPercent * 10 / 100;     // minimum starting throttle Percent is x%
                    }
                    motor_state_counter = 1;
                }
                else {  // if motor_state_counter != 0
                    if (acceleration > (float) 0.8 * accelerationThreshold) { // if acceleration is sufficient, no need to increment IQ
                        motor_state_counter++;
                        if (motor_state_counter >= 5) {  // when acceleration is sufficient for 5 consecutive times -> exit kick start and brake start states
                            motor_state = THROTTLE_CONTROL_STATE;            // Exit kick start state or brake start state
                        }
                    }
                    else {  // if acceleration is not sufficient, increment IQ
                        if (IQ_input_x10 + (float) speedModeIQmax * K_volt * 5 * 10 / 100 > (float) speedModeIQmax * K_volt * 10) { // IQ increment of 5%
                            IQ_input_x10 = (float) speedModeIQmax * K_volt * 10; // must never exceed (speedModeIQmax * K_volt)
                            motor_state = THROTTLE_CONTROL_STATE;        // If we reached this point, exit kick start and brake start states
                        }
                        else {
                            IQ_input_x10 += (float) speedModeIQmax * K_volt * 5 * 10 / 100;     //  increment by 5% every loop
                        }
                        // set motor_state_counter = 1, this ensures kick start or brake start states will only exit when sufficient acceleration occurred 5 consecutive times
                        motor_state_counter = 1;
                    }
                }
            } //    End Kick start state and Brake start state
            /*** throttle control state ***/
            else {  // this statement is TURE when NOT in kick start mode or brake start mode, i.e. motor_state = THROTTLE_CONTROL_STATE

                /****   Acceleration factor limits the acceleration of the motor, it moderates IQ increment depending on acceleration  ***/
                // if acceleration equals to the defined threshold acceleration, increment should = 0
                // if acceleration is less than the threshold accel, increment should be positive if throttle demand is greater
                // if acceleration is greater than the threshold accel., increment should be negative to reduce acceleration.
                if (acceleration > accelerationThreshold) {  // when acceleration is too great, K_acc is -ve -> hence decrease IQ to slow down acceleration
                    if (acceleration > (float) accelerationThreshold /((float) MIN_ACCELERATION_FACTOR / 2.5 + 1)) {
                        K_acc = MIN_ACCELERATION_FACTOR;
                    }
                    else {
                        K_acc = ((float) accelerationThreshold / acceleration - 1) * 2.5;
                    }
                }
                else {
                    if (acceleration < (float) accelerationThreshold / ((float) MAX_ACCELERATION_FACTOR + 1)) {
                        K_acc = MAX_ACCELERATION_FACTOR; //
                    }
                    else {
                        K_acc = ((float) accelerationThreshold / acceleration - 1); // when acceleration <= accelerationThreshold, K_acc is +ve
                    }
                }
                /****  K_prop = throttleRPMdemand factor.  It approaches zero as rpm approaches the demand rpm.   ****/
                // K_prop can be positive or negative for controlling whether IQ should be incremented or decremented.
                // when rpm is less than the rpm demand -> K_prop is always +ve.
                K_prop = (float) (throttleRPMdemand - RPM_temp) / (allowableRPM - REG_MINP_RPM);
                K_prop = pow(K_prop, K_prop_EXPONENT);
                if (RPM_temp < 180) {
                    K_prop = K_prop * K_prop_COEFF01;   // IQ boost at very low rpm
                }
                if (K_prop < 0) {
                    K_prop = 0;
                }

                /**   K_Limit is a max speed / speed Limit factor.  It approaches zero when rpm approach speed limit  **/
                // K_Limit can be positive or negative values
                // Note:  @ (throttleRPMdemand > rpm_temp), rpm is always equal or smaller than allowableRPM, hence
                // when (throttleRPMdemand > RPM_temp), K_Limit is always positive and greater than zero.
//                K_Limit = (float) (allowableRPM - RPM_temp) / (allowableRPM - REG_MINP_RPM);
                K_Limit = (float) (throttleRPMdemand - RPM_temp) / (throttleRPMdemand - REG_MINP_RPM);
                K_Limit = pow(K_Limit, 0.7);
                if (K_Limit < 0) {
                    K_Limit = 0;
                }

                deltaIQ_x10 = (float) K_Limit * K_prop * K_acc * IQ_increment_x10 * K_volt;

                if (IQ_input_x10 + deltaIQ_x10 > (float) speedModeIQmax * K_volt * 10) {
                    IQ_input_x10 = (float) speedModeIQmax * K_volt * 10; //
                }
                else {
                    IQ_input_x10 += deltaIQ_x10;

                    if (ThrottleReleasedFlag == 1) {
                        if (IQ_input_x10 < (float) speedModeIQmax * K_volt * MIN_THROTTLE_START_IQ_PERCENT * 10 / 100) {
                            IQ_input_x10 = (float) speedModeIQmax * K_volt * MIN_THROTTLE_START_IQ_PERCENT * 10 / 100;
                        }
                        ThrottleReleasedFlag = 0;
                    }
                }
            }
        }
        else {  // (throttleRPMdemand <= REG_MINP_RPM) && (throttleRPMdemand <= RPM_temp)
            ThrottleReleasedFlag = 1;

            K_acc = 1;
            /*****   when (throttleRPMdemand < RPM_temp)
             *****   when rpm exceeds rpm demand -> decelerate by decrementing IQ ****/
//            K_prop = 1;
            if ((throttleRPMdemand0 - throttleRPMdemand) > 0.2 * allowableRPM) {
                K_prop = 1.5;
            }
            else {
                K_prop = (float) (RPM_temp - throttleRPMdemand) * K_prop_COEFF02 / (allowableRPM - REG_MINP_RPM);
            }
            if (K_prop < 0) {
            // we should never get here. We are only here because (RPM_temp >= throttleRPMdemand)
            // Just for security purposes
                K_prop = 0;
            }
            if (K_prop > 1.5) {
                K_prop = 1.5;
            }

            K_Limit = 1;


            deltaIQ_x10 = (float) K_prop * K_Limit * speedModeIQmax * MAX_IQ_DECREMENT_PERCENT * 10 / 100;

            if ( IQ_input_x10 <= deltaIQ_x10) {
                IQ_input_x10 = 0;
            }
            else {
                IQ_input_x10 -= deltaIQ_x10;
            }
        }
    }

/****    throttle percentage error protocol    ****/
    throttleErrorHandler();
    if (throttle_errorStatus) {
        return;
    }
    IQ_input = ((float) IQ_input_x10 / 10);

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
    if (RPM_temp) {   // if rpm is not zero
        if (((float) REG_MAXPOUT * K_volt / (RPM_temp * 2 * PI_CONSTANT / 60) / KT_CONSTANT / KIQ_CONSTANT) > speedModeIQmax * K_volt) {
            IQ_maxPout = speedModeIQmax * K_volt;
        }
        else {
            IQ_maxPout = (float) REG_MAXPOUT * K_volt / (RPM_temp * 2 * PI_CONSTANT / 60) / KT_CONSTANT / KIQ_CONSTANT;
        }
    }
    else {
        IQ_maxPout = speedModeIQmax * K_volt;
    }

    // IQ input should not continuously exceed allowable output power limit -> decrease IQ to IQ_maxPout
    if (IQ_input > IQ_maxPout) {
        if (IQ_input - IQ_maxPout > speedModeIQmax * K_volt / 100) { // give IQ_input value a soft decrease towards IQ_maxPout
            IQ_input -= speedModeIQmax * K_volt / 100;
        }
        else {
            IQ_input = IQ_maxPout;
        }
        IQ_input_x10 = IQ_input * 10;
    }
}

/*********************************************************************
 * @fn      throttleErrorHandler
 *
 * @brief   Protocol and Handler for erroneous throttle percentage
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
static void throttleErrorHandler()
{
    /********  IMPORTANT: Throttle Error Protocol *******************/
//    throttlePercentApplied = 200; // for testing Throttle Error Protocol only!!
//    throttlePercentApplied_x100 = throttlePercentApplied * 100; // for testing Throttle Error Protocol only!!
    if (throttlePercentApplied > 100) {  //calculation involving floating points and rounding might result in throttlePercentApplied to be slightly greater than 100%
        //throttlePercentApplied should never be greater than 100%.
        //Significant error has occurred if it reaches here -> set throttle_errorStatus = 1
        throttle_errorStatus = 1;
        brakeAndThrottle_errorMsg = THROTTLE_ERROR;
        //error -> set throttlePercentApplied immediately to 0 to stop motor.
        throttlePercentApplied = 0;
        throttlePercentApplied_x100 = throttlePercentApplied * 100;
    /* if throttle_errorStatus = 1 -> disable throttle input and zero Iq command to Motor Controller Unit */
        led_display_ErrorPriority(THROTTLE_ERROR_PRIORITY);   /* throttle error priority = 11 */

        /******  Dashboard services *************************************/
        /* updates error code Characteristic Value -> Mobile App */
        if (dashboardErrorCodePriority > THROTTLE_ERROR_PRIORITY) {
            dashboardErrorCodePriority = THROTTLE_ERROR_PRIORITY;
            ptr_bat_ADArray->dashboardErrorCode = THROTTLE_ERROR_CODE;
            ptr_charVal = (ptr_bat_profileCharVal->ptr_dash_charVal->ptr_dashErrorCode);
            profile_setCharVal(ptr_charVal, DASHBOARD_ERROR_CODE_LEN, ptr_bat_ADArray->dashboardErrorCode);
        }
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
uint8_t NNIQIncrement = NN_IQ_INCREMENTS_LEISURE;
extern void brake_and_throttle_getSpeedModeParams()
{
    switch(speedMode)
    {
    case BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE:                   // Amble mode
        {
            if (ControlLaw == BRAKE_AND_THROTTLE_DIRECTLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_DIRECT_MODE_REDUCTION_RATIO_AMBLE;
                speedModeRPMmax = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS;
                accelerationThreshold = DIRECT_MODE_AMBLE_ACCELERATION_THRESHOLD;
            }
            else if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_AMBLE;
                speedModeRPMmax = BRAKE_AND_THROTTLE_MAXSPEED_AMBLE;
                accelerationThreshold = NORMAL_MODE_AMBLE_ACCELERATION_THRESHOLD;
                NNIQIncrement = NN_IQ_INCREMENTS_AMBLE;
            }
            rampRate = BRAKE_AND_THROTTLE_RAMPRATE_AMBLE;
            break;
        }
    case BRAKE_AND_THROTTLE_SPEED_MODE_LEISURE:                 // Leisure mode
        {
            if (ControlLaw == BRAKE_AND_THROTTLE_DIRECTLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_DIRECT_MODE_REDUCTION_RATIO_LEISURE;
                speedModeRPMmax = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS;
                accelerationThreshold = DIRECT_MODE_AMBLE_ACCELERATION_THRESHOLD;
            }
            else if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_LEISURE;
                speedModeRPMmax = BRAKE_AND_THROTTLE_MAXSPEED_LEISURE;
                accelerationThreshold = NORMAL_MODE_LEISURE_ACCELERATION_THRESHOLD;
                NNIQIncrement = NN_IQ_INCREMENTS_LEISURE;
            }
            rampRate = BRAKE_AND_THROTTLE_RAMPRATE_LEISURE;
            break;
        }
    case BRAKE_AND_THROTTLE_SPEED_MODE_SPORTS:                  // Sports mode
        {
            if (ControlLaw == BRAKE_AND_THROTTLE_DIRECTLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_DIRECT_MODE_REDUCTION_RATIO_SPORTS;
                speedModeRPMmax = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS;
                accelerationThreshold = DIRECT_MODE_AMBLE_ACCELERATION_THRESHOLD;
            }
            else if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS;
                speedModeRPMmax = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS;
                accelerationThreshold = NORMAL_MODE_SPORTS_ACCELERATION_THRESHOLD;
                NNIQIncrement = NN_IQ_INCREMENTS_SPORTS;
            }
            rampRate = BRAKE_AND_THROTTLE_RAMPRATE_SPORTS;
            break;
        }
    default:
        break;
    }
    speedModeIQmax = reductionRatio * STM32MCP_TORQUEIQ_MAX / 100;
    IQ_increment_x10 = ((float)speedModeIQmax * speedModeRPMmax * 10 / NNIQIncrement / BRAKE_AND_THROTTLE_MAXSPEED_SPORTS);

    /* Send updated speed mode parameters to motor control unit */
    /*  these following codes are necessary for control law change  */
    ptr_bat_STM32MCPDArray->speed_mode = speedMode;
    ptr_bat_STM32MCPDArray->speed_mode_IQmax = speedModeIQmax;
    ptr_bat_STM32MCPDArray->ramp_rate = rampRate;
    ptr_bat_STM32MCPDArray->allowable_rpm = speedModeRPMmax;
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
    if (!brake_errorStatus) // no brake error
    {
        if (throttleADCsample <= 1.1 * THROTTLE_ADC_CALIBRATE_L)    // Fully release throttle to change speed mode,  no change when throttle is applied - will by-pass if throttle is not zero
        {
            if(speedMode == BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE)  // if Amble mode -> change to Leisure mode
            {
                speedMode = BRAKE_AND_THROTTLE_SPEED_MODE_LEISURE;
                if (ControlLaw == BRAKE_AND_THROTTLE_DIRECTLAW)
                {
                    reductionRatio = BRAKE_AND_THROTTLE_DIRECT_MODE_REDUCTION_RATIO_LEISURE;
                    speedModeRPMmax = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS;
                    accelerationThreshold = DIRECT_MODE_AMBLE_ACCELERATION_THRESHOLD;
                }
                else if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
                {
                    reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_LEISURE;
                    speedModeRPMmax = BRAKE_AND_THROTTLE_MAXSPEED_LEISURE;
                    accelerationThreshold = NORMAL_MODE_LEISURE_ACCELERATION_THRESHOLD;
                    NNIQIncrement = NN_IQ_INCREMENTS_LEISURE;
                }
                rampRate = BRAKE_AND_THROTTLE_RAMPRATE_LEISURE;

                // turn off auxiliary light
                *ptr_bat_auxiliaryLightStatus = 0;
            }
            else if(speedMode == BRAKE_AND_THROTTLE_SPEED_MODE_LEISURE) // if Leisure mode -> change to Sports mode
            {
                speedMode = BRAKE_AND_THROTTLE_SPEED_MODE_SPORTS;
                if (ControlLaw == BRAKE_AND_THROTTLE_DIRECTLAW)
                {
                    reductionRatio = BRAKE_AND_THROTTLE_DIRECT_MODE_REDUCTION_RATIO_SPORTS;
                    speedModeRPMmax = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS;
                    accelerationThreshold = DIRECT_MODE_AMBLE_ACCELERATION_THRESHOLD;
                }
                else if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
                {
                    reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS;
                    speedModeRPMmax = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS;
                    accelerationThreshold = NORMAL_MODE_SPORTS_ACCELERATION_THRESHOLD;
                    NNIQIncrement = NN_IQ_INCREMENTS_SPORTS;
                }
                rampRate = BRAKE_AND_THROTTLE_RAMPRATE_SPORTS;
            }
            else if(speedMode == BRAKE_AND_THROTTLE_SPEED_MODE_SPORTS) // if Sports mode -> change back to Amble mode
            {
                speedMode = BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE;
                if (ControlLaw == BRAKE_AND_THROTTLE_DIRECTLAW)
                {
                    reductionRatio = BRAKE_AND_THROTTLE_DIRECT_MODE_REDUCTION_RATIO_AMBLE;
                    speedModeRPMmax = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS;
                    accelerationThreshold = DIRECT_MODE_AMBLE_ACCELERATION_THRESHOLD;
                }
                else if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
                {
                    reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_AMBLE;
                    speedModeRPMmax = BRAKE_AND_THROTTLE_MAXSPEED_AMBLE;
                    accelerationThreshold = NORMAL_MODE_AMBLE_ACCELERATION_THRESHOLD;
                    NNIQIncrement = NN_IQ_INCREMENTS_AMBLE;
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
                reductionRatio = BRAKE_AND_THROTTLE_DIRECT_MODE_REDUCTION_RATIO_AMBLE;
                speedModeRPMmax = BRAKE_AND_THROTTLE_MAXSPEED_SPORTS;
                accelerationThreshold = DIRECT_MODE_AMBLE_ACCELERATION_THRESHOLD;
            }
            else if (ControlLaw == BRAKE_AND_THROTTLE_NORMALLAW)
            {
                reductionRatio = BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_AMBLE;
                speedModeRPMmax = BRAKE_AND_THROTTLE_MAXSPEED_AMBLE;
                accelerationThreshold = NORMAL_MODE_AMBLE_ACCELERATION_THRESHOLD;
                NNIQIncrement = NN_IQ_INCREMENTS_AMBLE;
            }
            rampRate = BRAKE_AND_THROTTLE_RAMPRATE_AMBLE;
            // turn on auxiliary light
            *ptr_bat_auxiliaryLightStatus = 1;
        }
    }

    speedModeIQmax = reductionRatio * STM32MCP_TORQUEIQ_MAX / 100;
    IQ_increment_x10 = ((float)speedModeIQmax * speedModeRPMmax * 10 / NNIQIncrement / BRAKE_AND_THROTTLE_MAXSPEED_SPORTS);

    /* Send updated speed mode parameters to motor control unit */
    ptr_bat_STM32MCPDArray->speed_mode = speedMode;
    ptr_bat_STM32MCPDArray->speed_mode_IQmax = speedModeIQmax;
    ptr_bat_STM32MCPDArray->ramp_rate = rampRate;
    ptr_bat_STM32MCPDArray->allowable_rpm = speedModeRPMmax;

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
 * @return  throttle percentage
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
 * @return  brake percentage
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

/*********************************************************************
 * @fn      bat_shutDownReadyRegister
 *
 * @brief   return the pointer register to the variable "shutDownReady"
 *
 * @param   None
 *
 * @return  &shutDownReady
 */
extern uint8_t* bat_shutDownReadyRegister()
{
    return (&shutDownReady);
}

/*********************************************************************
 * @fn      bat_auxiliaryLightStatusRegister
 *
 * @brief   Auxiliary light for future use
 *
 * @param   None
 *
 * @return  None
 */
extern uint8_t bat_auxiliaryLightStatusRegister(uint8_t *ptrAuxiliaryLightStatus)
{
    ptr_bat_auxiliaryLightStatus = ptrAuxiliaryLightStatus;
    if (speedMode == BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE) {
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
 * @brief   call to assign and register the pointer to power_On
 *
 * @param   a pointer to power_On, i.e. ptr_powerOn
 *
 * @return  None
 */
extern void bat_powerOnRegister(bool *ptrpowerOn)
{
    ptr_bat_POWER_ON = ptrpowerOn;
}

