/*
 * data_analytics.c
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */
/******************************************************************************
 * @file  data_analytics.c
 * @brief This file contains the functions for calculating:
 *     (1) distance travelled
 *     (2) power consumed
 *     (3) Average Speed
 *     (4) Average Battery Voltage
 *     (5) battery percentage
 *     (6) battery status
 *     (7) Instantaneous economy
 *     (8) Economy
 *     (9) Range
 *     (10) CO2 Saved
 *
 *     distance travelled and power consumed are stored in snvBuffer[]
 *     snvBuffer[] is stored in NVS when device shuts down.
 *
 *     The above performance data are packaged and sent to mobile app, to LED display,
 *     & to Cloud storage via mobile app and saved in flash memory
 *
 *****************************************************************************/

/*********************************************************************
* INCLUDES
*/
#include "Application/data_analytics.h"

#include "Application/general_purpose_timer.h"
#include "Application/snv_internal.h"
#include "Application/brake_and_throttle.h"
#include "Application/lights.h"
#include "Application/led_display.h"
#include "Application/periodic_communication.h"
#include "Application/power_on_time.h"

#include "Profiles/controller_profile.h"
#include "Profiles/dashboard_profile.h"
#include "Profiles/battery_profile.h"
#include "Profiles/profile_charVal.h"

/*********************************************************************
* LOCAL VARIABLES
*/

/*****  N = 2.  See General Purpose Timer.c for selecting the correct value of N *******/

static uint16_t data_analytics_sampling_time = DATA_ANALYTICS_INTERVAL;

/********   NVS  ********************/
static uint32_t (*ptr_snvBuffer)[SNV_BUFFER_SIZE];  // pointer callback register to snvBuffer

/****  Numerical Integration Simpson's 1/3 rule coefficients   ****/
uint8_t  coefficient_array[DATA_ANALYSIS_POINTS] = {0};           // same size as DATA_ANALYSIS_POINTS

//Default Unit Settings
uint8_t  UnitSelectDash = SI_UNIT;          // Keep the last units selected by user in memory, the same units is used on restart
static uint8_t  UnitSelectApp = SI_UNIT;    // Mobile app allow user to select the desired display unit - the App units and Dash units are NOT linked
static uint8_t  speedmode_init;
static uint8_t  lightmode_init;
static uint8_t  *ptr_lightmode;
static uint32_t *ptr_uptimeMinutes;

/***  Local variables declaration   ***/
uint8_t             controllerErrorCodePriority = SYSTEM_NORMAL_PRIORITY;
uint8_t             batteryErrorCodePriority = SYSTEM_NORMAL_PRIORITY;
static uint8_t      *ptr_da_dashboardErrorCodePriority;   // dashboardErrorCodePriority declared in brake_and_throttle.c

profileCharVal_t    *ptr_da_profileCharVal;
static uint8        *ptr_da_charVal;

/*** declaration of performance data from mcu sampling  *****/
uint16_t  rpm[DATA_ANALYSIS_POINTS] = {0};                        //revolutions per minute data collected every time interval
uint16_t  speed_cmps[DATA_ANALYSIS_POINTS] = {0};                 //rpm is converted to cm per second (cm/s)
uint16_t  batteryCurrent_mA[DATA_ANALYSIS_POINTS] = {0};          //battery current data collected at every hf communication interval
uint16_t  batteryVoltage_mV[DATA_ANALYSIS_POINTS] = {0};          //battery voltage data collected at every hf communication interval
uint16_t  phaseCurrent_mA[DATA_ANALYSIS_POINTS] = {0};           //phase current data collected at every hf communication interval
uint16_t  phaseVoltage_mV[DATA_ANALYSIS_POINTS] = {0};           //phase voltage data collected at every hf communication interval
uint8_t   heatSinkTempOffset50_C[DATA_ANALYSIS_POINTS] = {0};        // temperature can be negative. tempOffset50 = temperature + 50
uint8_t   motorTempOffset50_C[DATA_ANALYSIS_POINTS] = {0};           // temperature can be negative. tempOffset50 = temperature + 50

uint16_t  avgSpeed100kph = 0;    // output in km/hr
uint16_t  avgRPM = 0;


/*** ADArray is a temporary buffer/holder for organising BLE profile variables  *****/
AD_t            ADArray = {0};

uint16_t        dA_Count = 1;
static uint32_t UDDataCounter = 0;           // At new or reset, UDDataCounter = 0
static uint16_t UDIndex;                     // the last UDIndex saved
static uint16_t UDIndexPrev;

static float    lenConvFactorDash;
uint8_t  batteryStatus;
uint16_t avgBatteryVoltage_mV = LEVEL45;
uint16_t avgBusCurrent_mA = 0;
uint16_t avgPhaseVoltage_mV;
uint16_t avgPhaseCurrent_mA;
uint8_t  avgBatteryPercent = BATTERY_PERCENTAGE_INITIAL;
static bool     batteryLow = 0;

/**************************************/
//static uint8_t heatSinkOVTempState = 0;
//static uint8_t motorOVTempState = 0;
//static uint8_t dataAnalysis_batteryError = 0;
uint8_t  da_startup = 1;
uint8_t  UDTriggerCounter = 0;
uint32_t ADDataCounter = 0;
uint32_t sumDeltaMileage_dm;                 // unit in decimeters.  This is the previous data on the total distance travelled
uint32_t sumDeltaPowerConsumed_mWh;          // unit in milli-W-hr.  This is the previous data on the total power consumption
uint32_t totalMileagePrev_dm;                // unit in decimeters.  This is the previous data on the total distance travelled
uint32_t totalPowerConsumedPrev_mWh;         // unit in milli-W-hr.  This is the previous data on the total power consumption
uint32_t totalMileage0_dm;                   // unit in decimeters.  This is the oldest data on total distance travelled stored in storage array
uint32_t totalPowerConsumed0_mWh;            // unit in milli-W-hr.  This is the oldest data on total power consumed stored in storage array

/****  MCUD_t is a struct buffer/holder that organizes the performance data periodically sampled from controller for data analysis
 *     MCUD_t is declared in motor_control.c
 ***************************************************************************/
MCUD_t (*ptr_MCUDArray);

/*********************************************************************
* LOCAL Functions
*/
static void coefficient_array_init( void );
static void re_Initialize( void );
static void data_analytics_getSNVData( void );
static void data_analytics_setCharVal(void);

//Performance related Function declaration
static uint32_t computePowerConsumption( void ); // output in mW-hr
static uint32_t computeDistanceTravelled( void );// output in decimeter
static uint16_t  computeAvgSpeed(uint32_t deltaMileage_dm);   // output in km/hr
static uint8_t  computeAvgHeatSinkTemperature( void );         // output in degrees celsius
static void computeAvgVoltages( void ); // output in mV
static void computeAvgCurrents(void);
static uint16_t computeInstantEconomy(uint32_t deltaPowerConsumption_mWh, uint32_t deltaMileage_dm); // unit in W-hr / km x 100
static uint32_t computeEconomy( void );  // unit in W-hr / km x 100
static uint32_t computeRange( void ); // output in metres
static uint32_t computeCO2Saved( void ); // in g
static uint8_t  computeMotorTemperature( void ); // in degrees Celsius

/***********************************************************************************************************
 * @fn      data_analytics_setSNVBufferRegister
 *
 * @brief   register callback pointer to snvBuffer[]
 *
 * @param   ptrsnvbuf
 *
 * @return  Nil
******************************************************************************************************/
void data_analytics_setSNVBufferRegister(uint32_t (*ptrsnvbuf)[]){
    ptr_snvBuffer = ptrsnvbuf;
}

/*********************************************************************
  * @fun    data_analytics_MCUDArrayRegister
 *
 * @brief   Registering pointer to strut array MCUDArray
 *
 * @param   ptrMCUD
 *
 * @return  Nil
 */
extern void data_analytics_MCUDArrayRegister(MCUD_t (*ptrMCUD))
{
    ptr_MCUDArray = ptrMCUD;
}

/*********************************************************************
 * @fn      da_powerOnRegister
 *
 * @brief   call to assign and register the pointer to powerOn
 *
 * @param   a pointer to powerOn, i.e. ptr_powerOn
 *
 * @return  None
 */
static bool *ptr_da_POWER_ON;
extern void da_powerOnRegister(bool *ptrpoweron)
{
    ptr_da_POWER_ON = ptrpoweron;
}

/*********************************************************************
 * @fn      data_analytics_dashErrorCodeStatusRegister
 *
 * @brief   callback pointer to dashboardErrorCodePriority
 *
 * @param   a pointer to dashboardErrorCodePriority
 *
 * @return  None
 */
extern void data_analytics_dashErrorCodeStatusRegister(uint8_t *ptrdashboardErrorCodePriority)
{
    ptr_da_dashboardErrorCodePriority = ptrdashboardErrorCodePriority;
}

/******************************************************************************************************
 * @fun      dataAnalysis_Init
 *
 * @brief   Initialization and determines the starting conditions from saved data
 *
 * @param   Nil
 *
 * @return  Nil
******************************************************************************************************/
uint32_t resetUDDataCounter;

extern void data_analytics_init()
{
    resetUDDataCounter = (0xFFFFFFFF / UDARRAYSIZE) * UDARRAYSIZE;

    /******************************************************
     *  get pointer / registers to all BLE profile Characteristic Values
     ******************************************************/
    ptr_lightmode = lights_lightModeRegister();
    ptr_da_profileCharVal = profile_charVal_profileCharValRegister();
    ptr_uptimeMinutes = pot_uptimeMinute();
    brake_and_throttle_ADArrayRegister(&ADArray);

   /***************************************************
    *      Read data stored in NVS Internal
    ***************************************************/
    data_analytics_getSNVData();    // this function sets the variables read from NVS

    /* ***************************************************
     * initialize Simpsons 1/3 rule coefficient_array
     *****************************************************/
    coefficient_array_init();
    periodic_communication_MCUSampling();
    /****************************************************
     * At the instant of POWER ON, retrieve BATTERY status for LED display
     * dashboard will instruct motor controller to obtain a battery voltage and current measurement
     */
#ifdef MOTOR_CONNECT
    STM32MCP_getRegisterFrame(STM32MCP_MOTOR_1_ID,STM32MCP_BUS_VOLTAGE_REG_ID);
#endif // MOTOR_CONNECT

    uint16_t batteryVoltageStartUp_mV = ptr_MCUDArray->bat_voltage_mV; //LEVEL01; //LEVEL45;
    uint16_t batteryCurrentStartUp_mA = ptr_MCUDArray->bat_current_mA;//3000;
    uint16_t phaseVoltageStartUp_mV = ptr_MCUDArray->phase_voltage_mV;//30000;
    uint16_t phaseCurrentStartUp_mA = ptr_MCUDArray->phase_current_mA;//3000;
    uint8_t mTStartUp = ptr_MCUDArray->motorTempOffset50_Celcius;//65;                     //temperature + offset = 15+50;
    uint8_t hSTStartUp = ptr_MCUDArray->heatSinkTempOffset50_Celcius;//65;                    //temperature + offset = 15+50;

    /****** Initialize the following data arrays: ********
     * - RPM
     * - Speed
     * - Battery voltage
     * - Battery Current
     * - Phase voltage
     * - Phase current
     * - Heat sink Temperature
     * - Motor Temperature
     ******************************/
    for (uint8_t kk = 0; kk < DATA_ANALYSIS_POINTS; kk++) {
        rpm[ kk ] = 0;
        speed_cmps[ kk ] = round((float) rpm[ kk ] * 2 * PI_CONSTANT / 60 * WHEELRADIUS_CM); // Unit in cm / sec
        batteryCurrent_mA[ kk ] = batteryCurrentStartUp_mA;                                  // unit in mA = get battery current in mA
        batteryVoltage_mV[ kk ] = batteryVoltageStartUp_mV;                                  // unit in mV = get battery voltage in mV
        phaseCurrent_mA[ kk ] = phaseCurrentStartUp_mA;                                      // unit in mA = get phase current in mA
        phaseVoltage_mV[ kk ] = phaseVoltageStartUp_mV;                                      // unit in mV = get phase voltage in mV

        heatSinkTempOffset50_C[ kk ] = hSTStartUp;   // +50
        motorTempOffset50_C[ kk ] = mTStartUp;   // +50
    }
    // Reset and ensures dA_Count = 1 after initialization
    dA_Count = 1;

    /*********************************************************************************************
     * Initializing performance data
     * defining ADArray variables at initialization allow connectivity with Mobile App instantly.
     *********************************************************************************************/
    ADArray.accumPowerConsumption_mWh = totalPowerConsumedPrev_mWh;         // ADArray = App data strut
    ADArray.accumMileage_dm = totalMileagePrev_dm;                          // ADArray = App data strut
    ADArray.avgSpeed_100kph = 0;                                            // ADArray = App data strut
    ADArray.HeatSinkTempOffset50_C = computeAvgHeatSinkTemperature();       // ADArray = App data strut

    computeAvgVoltages();
    ADArray.avgBatteryVoltage_mV = avgBatteryVoltage_mV;                    // ADArray = App data strut
    ADArray.avgBusCurrent_mA = ptr_MCUDArray->bat_current_mA;
    ADArray.avgPhaseVoltage_mV = ptr_MCUDArray->phase_voltage_mV;
    ADArray.avgPhaseCurrent_mA = ptr_MCUDArray->phase_current_mA;

    ADArray.batteryPercentage = computeBatteryPercentage();                 // ADArray = App data strut
    ADArray.batteryStatus = determineBatteryStatus();                       // ADArray = App data strut
    ADArray.batteryCode = SYS_NORMAL_CODE;      // Initial error code set to normal
    ADArray.dashboardErrorCode = SYS_NORMAL_CODE;
    ADArray.controllerErrorCode = SYS_NORMAL_CODE;

    ADArray.instantEconomy_100Whpk = computeInstantEconomy(0, 0);           // ADArray = App data strut
    ADArray.economy_100Whpk = computeEconomy();                             // ADArray = App data strut
    ADArray.range_m = computeRange();                                       // ADArray = App data strut
    ADArray.co2Saved_g = computeCO2Saved();                                 // ADArray = App data strut
    ADArray.motorTempOffset50_C = computeMotorTemperature();                // ADArray = App data strut

    data_analytics_changeUnitSelectDash();      // Send Initial Unit Select to LED display

    /* send speedmode_init to led_display*/
    /* led_display_init() must be called before data_analytics_init() */
    led_display_setSpeedMode( speedmode_init );

    *ptr_lightmode = lightmode_init;

    avgBatteryPercent = ADArray.batteryPercentage;
    if ((batteryLow == 0) && (ADArray.batteryPercentage < BATTERY_PERCENTAGE_LL)){
        batteryLow = 1;
    }
    if ((batteryLow == 1) && (ADArray.batteryPercentage > BATTERY_PERCENTAGE_LH)){
        batteryLow = 0;
    }
}

/***********************************************************************************************************
 * @fn      data_analytics_getSNVData()
 *
 * @brief   Set and update data for data analysis
 *
 * @param   Nil
 *
 * @return  Nil
******************************************************************************************************/
static void data_analytics_getSNVData()
{
    ADDataCounter = (*ptr_snvBuffer)[0];

    UDDataCounter = (*ptr_snvBuffer)[1];
    UDIndexPrev = UDDataCounter % UDARRAYSIZE;      // UDIndexPrev is the remainder of UDDataCounter / UDARRAYSIZE
    if (UDIndexPrev >= (UDARRAYSIZE - 1)) {
            UDIndex = 0;
    }
    else {
            UDIndex = UDIndexPrev + 1;
    }
    //nvsBuffer option
    // SETSIZE = 2

    totalMileage0_dm = (*ptr_snvBuffer)[2 + UDIndex * SETSIZE + 0];
    totalPowerConsumed0_mWh = (*ptr_snvBuffer)[2 + UDIndex * SETSIZE + 1];
    totalMileagePrev_dm = (*ptr_snvBuffer)[2 + UDIndexPrev * SETSIZE + 0];
    totalPowerConsumedPrev_mWh = (*ptr_snvBuffer)[2 + UDIndexPrev * SETSIZE + 1];

    speedmode_init = (*ptr_snvBuffer)[SNV_BUFFER_SIZE - 4];
    UnitSelectDash = (*ptr_snvBuffer)[SNV_BUFFER_SIZE - 3];
    lightmode_init = (*ptr_snvBuffer)[SNV_BUFFER_SIZE - 2];
    *ptr_uptimeMinutes = (*ptr_snvBuffer)[SNV_BUFFER_SIZE - 1];
}

/******************************************************************************************************
 * @fn      dataAnalysis_sampling
 *
 * @brief   Simulates data obtaining from MCU
 *
 * @param   jj
 *
 * @return  Nil
******************************************************************************************************/
extern void data_analytics_sampling()
{
    dA_Count = ptr_MCUDArray->count_hf;

    rpm[dA_Count] = ptr_MCUDArray->speed_rpm; // unit in rpm,  188 rpm @ r = 0.1016m => 200 cm/sec = 7 km/hr
    speed_cmps[dA_Count] = round((float) rpm[dA_Count] * 2 * PI_CONSTANT / 60 * WHEELRADIUS_CM);      // Unit in cm / sec
    batteryCurrent_mA[dA_Count] = ptr_MCUDArray->bat_current_mA;
    batteryVoltage_mV[dA_Count] = ptr_MCUDArray->bat_voltage_mV;
    phaseCurrent_mA[dA_Count] = ptr_MCUDArray->phase_current_mA;
    phaseVoltage_mV[dA_Count] = ptr_MCUDArray->phase_voltage_mV;
    heatSinkTempOffset50_C[dA_Count] = ptr_MCUDArray->heatSinkTempOffset50_Celcius;     // +50
    motorTempOffset50_C[dA_Count] = ptr_MCUDArray->motorTempOffset50_Celcius;           // +50

//    uint16_t instant_rpm = rpm[dA_Count];
    uint16_t instant_speed_100kmph = ((float) speed_cmps[dA_Count] * 3.6);  // speed in 100 x kmph
    // update rpm and speed profile characteristic values
    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_motorSpeed);
    profile_setCharVal(ptr_da_charVal, CONTROLLER_MOTOR_SPEED_LEN, instant_speed_100kmph);
    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_motorRPM);
    profile_setCharVal(ptr_da_charVal, CONTROLLER_MOTOR_RPM_LEN, rpm[dA_Count]);

    /*****  Send speed to led display  *****/
    data_analytics_LEDSpeed();       // convert speed to the selected dashboard unit (dashSpeed) and refresh led display
}

/******************************************************************************************************
 * @fn      data_analytics_Main
 *
 * @brief   Main data analytics function
 *
 * @param   Nil
 *
 * @return  Nil
******************************************************************************************************/
uint8_t batteryLowCounter = 0;

extern void data_analytics_Main( void )
{
    if (da_startup)
    {
        da_startupAnalytics();
        da_startup = 0;
    }
    /******  if dA_Count = (DATA_ANALYSIS_POINTS -1), triggers data_analytics()  *********/
    if (dA_Count == (DATA_ANALYSIS_POINTS - 1)) // and also when Power OFF // Caution of the case where dA_Count >= DATA_ANALYSIS_POINTS & POWER OFF
    {
        /***** data_analytics() carries out all the data analytics *****/
        data_analytics();

        led_display_setBatteryStatus(ADArray.batteryStatus);                     // Send battery status and errorCode to led display
        avgBatteryVoltage_mV = ADArray.avgBatteryVoltage_mV;                     // declared globally for debugging only

        // send analytics data to Mobile App
        data_analytics_setCharVal();

        //batteryPercentage
        if ((batteryLow == 0) && (ADArray.batteryPercentage < BATTERY_PERCENTAGE_LL))
        {
            batteryLow = 1;
        }
        if ((batteryLow == 1) && (ADArray.batteryPercentage > BATTERY_PERCENTAGE_LH))
        {
            batteryLow = 0;
        }
        if (UDTriggerCounter >= UDTRIGGER)      // save data to NVS Buffer
        {
            data2snvBuffer(); // This is called to update the data in snvBuffer
        }
    }
}

/***************************************************************************************************
 * @fn      data_analytics
 *
 * @brief   Data Evaluation function:  Call by dataSim() and when Power-OFF
 *
 * @param   Nil
 *
 * @return  Nil
******************************************************************************************************/
//uint32_t deltaPowerConsumption_mWh, deltaMileage_dm;
extern void data_analytics()
{
    uint32_t deltaPowerConsumption_mWh, deltaMileage_dm;
    ADDataCounter++;
    ADArray.ADCounter = ADDataCounter;                // Why not ADDataCounter + 1?              // totalDataCount is total count of all computed datasets
    deltaPowerConsumption_mWh = computePowerConsumption();
    deltaMileage_dm = computeDistanceTravelled();
    sumDeltaPowerConsumed_mWh += deltaPowerConsumption_mWh;
    sumDeltaMileage_dm += deltaMileage_dm;
    ADArray.accumPowerConsumption_mWh = totalPowerConsumedPrev_mWh + sumDeltaPowerConsumed_mWh;
    ADArray.accumMileage_dm = totalMileagePrev_dm + sumDeltaMileage_dm;
    ADArray.avgSpeed_100kph = computeAvgSpeed(deltaMileage_dm);

    computeAvgVoltages();
    computeAvgCurrents();
    ADArray.avgBatteryVoltage_mV = avgBatteryVoltage_mV;
    ADArray.avgBusCurrent_mA = avgBusCurrent_mA;
    ADArray.avgPhaseVoltage_mV = avgPhaseVoltage_mV;
    ADArray.avgPhaseCurrent_mA = avgPhaseCurrent_mA;

    ADArray.batteryPercentage = computeBatteryPercentage(); // battery percentage must be called before battery status
    ADArray.batteryStatus = determineBatteryStatus();

    ADArray.HeatSinkTempOffset50_C = computeAvgHeatSinkTemperature();

    ADArray.instantEconomy_100Whpk = computeInstantEconomy(deltaPowerConsumption_mWh, deltaMileage_dm);
    ADArray.economy_100Whpk = computeEconomy();
    ADArray.range_m = computeRange();
    ADArray.co2Saved_g = computeCO2Saved();
    ADArray.motorTempOffset50_C = computeMotorTemperature();

    re_Initialize();    // Re-initialize rpm, speed, batteryVoltage and batteryCurrent arrays after data analysis

    UDTriggerCounter++;     // When UDTriggerCounter = UDTrigger, UDArray is saved to flash memory
}

/***************************************************************************************************
 * @fn      da_startupAnalytics
 *
 * @brief   Run at start up to evaluate battery initial percentage and status
 *
 * @param   Nil
 *
 * @return  Nil
******************************************************************************************************/
void da_startupAnalytics()
{
    dA_Count = (DATA_ANALYSIS_POINTS - 1);
    computeAvgVoltages();
    ADArray.avgBatteryVoltage_mV = avgBatteryVoltage_mV;
    ADArray.batteryPercentage = computeBatteryPercentage(); // battery percentage must be called before battery status
    ADArray.batteryStatus = determineBatteryStatus();
    dA_Count = 1;
}
/***************************************************************************************************
 * @fn      computePowerConsumption
 *
 * @brief   This function calculates the change in power consumption of the e_scooter
 *          over the time interval using Simpson's 1/3 Rule
 *
 * @param   AccumPowerConsumed
 *
 * @return  energy consumption value (unit milli W-hr) in type: uint32_t
******************************************************************************************************/
uint32_t computePowerConsumption()
{
    uint32_t temp_deltaPowerConsumption_mWh = 0;
    for( uint8_t ii = 0; ii < DATA_ANALYSIS_POINTS; ii++ )
    {
        uint32_t deltaPower_mWh = batteryVoltage_mV[ii] * batteryCurrent_mA[ii] / 1000;         // look out for possible byte size limitation issue
        temp_deltaPowerConsumption_mWh += coefficient_array[ii] * deltaPower_mWh;
    }
    temp_deltaPowerConsumption_mWh = round((float) temp_deltaPowerConsumption_mWh / 3000 * data_analytics_sampling_time / 3600);       // output in milli-W-hr

    return (temp_deltaPowerConsumption_mWh);   //  -> convert to the desired unit before displaying on App
}

/***************************************************************************************************
 * @fn      computeDistanceTravelled
 *
 * @brief   This function calculates the change of distance_travelled of the e_scooter
 *          over a period using Simpson's Rule.  The distance is in decimeter.
 *
 * @param   AccumMileage
 *
 * @return  distanceTravelled (unit in decimeters, i.e.  0.1 meters) in type: uint32_t
******************************************************************************************************/
uint32_t computeDistanceTravelled()
{
    uint32_t deltaDistanceTravelled_dm = 0;
    for( uint8_t ii = 0; ii < DATA_ANALYSIS_POINTS; ii++ )
    {
         deltaDistanceTravelled_dm += coefficient_array[ii] * speed_cmps[ii];            // for computational accuracy reasons, calculations are performed in centimeter/second
    }
    deltaDistanceTravelled_dm = round((float) deltaDistanceTravelled_dm * data_analytics_sampling_time / 30000);// (1/3) x (1/1000) x (1/10) = 1/30000: output is then converted to decimeter

    return (deltaDistanceTravelled_dm); // -> convert to the desired unit before displaying on App
}

/***************************************************************************************************
 * @fn      computeAverageSpeed
 *
 * @brief   This function calculates the average speed over the given time interval in km/hr x 100
 *          This function is called only when dA_Count = (DATA_ANALYSIS_POINTS -)
 *
 * @param   DeltaDistanceTravelled
 *
  * @return  avgSpeed in km/hr x 100
******************************************************************************************************/
uint16_t computeAvgSpeed(uint32_t deltaMileage_dm)
{
    uint16_t avgSpeed_100kph;
    if (dA_Count == (DATA_ANALYSIS_POINTS - 1))
    {
        // (deltaMileage_dm / 10000) / ( Dt / 1000 / 3600) x 100 =  deltaMileage_dm / Dt * 360 * 100 (100 km/hr)
        avgSpeed_100kph = (uint16_t) round ((float) deltaMileage_dm * 36000 / (data_analytics_sampling_time * (DATA_ANALYSIS_POINTS - 1))); // output in km/hr
        avgRPM = round ((float) avgSpeed_100kph / 3.6 / WHEELRADIUS_CM *60 / 2 / PI_CONSTANT);
    }
    avgSpeed100kph = avgSpeed_100kph;

    return (avgSpeed_100kph);                                // output rounded off to nearest km/hr
}

/**********************************************************************************************************
 * @fn      computeAverageHST
 *
 * @brief   This function calculates the average heat sink (motor controller) temperature over the given
 *          time interval in degrees celsius
 *          This function is called only when dA_Count = (DATA_ANALYSIS_POINTS -)
 *
 * @param   Nil
 *
  * @return  avghst in degree celsius
******************************************************************************************************/
uint8_t computeAvgHeatSinkTemperature()
{
    uint16_t sumHeatSinkTempOffset50_C = 0; // output in degree celsius
    uint8_t avgheatSinkTempOffset50_C = 0;

    /**** Whenever this func is called, dA_Count = (DATA_ANALYSIS_POINTS - 1)  ****/
    if (dA_Count == (DATA_ANALYSIS_POINTS - 1))
    {
        for(uint8_t ii = 0; ii < DATA_ANALYSIS_POINTS; ii++)
        {
            sumHeatSinkTempOffset50_C += heatSinkTempOffset50_C[ii];                             // sum all measurements
        }
        avgheatSinkTempOffset50_C = round((sumHeatSinkTempOffset50_C)/ DATA_ANALYSIS_POINTS);
        if (avgheatSinkTempOffset50_C > CRIT_HEATSINKTEMPOFFSET50C)
        {
            /* Action: Restrict speed mode to AMBLE mode or restrict IQ to 0 ....
             * until temperature drop below the safe threshold  */
            led_display_ErrorPriority(CONTROLLER_TEMP_ERROR_PRIORITY);

            if (controllerErrorCodePriority > CONTROLLER_TEMP_ERROR_PRIORITY)
            {
                controllerErrorCodePriority = CONTROLLER_TEMP_ERROR_PRIORITY;
                ADArray.controllerErrorCode = CONTROLLER_TEMP_ERROR_CODE;
            }
        }
    }

    return (avgheatSinkTempOffset50_C); // output in degree celsius
}

/**********************************************************************************************************
 * @fn      computeMotorTemperature
 *
 * @brief   This function calculates the average Motor temperature over the given time interval in degrees celsius
 *
 * @param   Nil
 *
  * @return  avgMotorTemperature in degree celsius
******************************************************************************************************/
uint8_t computeMotorTemperature()
{
    uint16_t sumMotorTempOffset50_C = 0; // output in degree celsius
    uint8_t avgMotorTempOffset50_C = 0;

    /**** Whenever this func is called, dA_Count = (DATA_ANALYSIS_POINTS - 1)  ****/
    if (dA_Count == (DATA_ANALYSIS_POINTS - 1))
    {
        for(uint8_t ii = 0; ii  < DATA_ANALYSIS_POINTS; ii++)
        {
             sumMotorTempOffset50_C += motorTempOffset50_C[ii];                             // sum all measurements
        }
        avgMotorTempOffset50_C = round((sumMotorTempOffset50_C) / DATA_ANALYSIS_POINTS);

        if (avgMotorTempOffset50_C > CRIT_MOTORTEMPOFFSET50C)
        {
            // Action: Restrict speed mode to AMBLE mode or restrict IQ to 0 until temperature drop below the safe threshold
            led_display_ErrorPriority(MOTOR_TEMP_ERROR_PRIORITY);

            if (controllerErrorCodePriority > MOTOR_TEMP_ERROR_PRIORITY)
            {
                controllerErrorCodePriority = MOTOR_TEMP_ERROR_PRIORITY;
                ADArray.controllerErrorCode = MOTOR_TEMP_ERROR_CODE; // error code algorithm to be defined
            }
        }
    }

    return (avgMotorTempOffset50_C); // output in degree celcius
}

/***************************************************************************************************
 * @fn      computeAvgVoltages
 *
 * @brief   This function calculates the average of the battery voltage over the given time interval in mV.
 *
 * @param   batteryVoltage_mV in mV
 *
 * @return  AvgBatteryVoltage in milli-Volt
******************************************************************************************************/
void computeAvgVoltages()
{
    uint32_t    sumBatteryVoltage_mV = 0;
    uint32_t    sumPhaseVoltage_mV = 0;

    /**** Whenever this func is called, dA_Count = (DATA_ANALYSIS_POINTS - 1)  ****/
    if (dA_Count == (DATA_ANALYSIS_POINTS - 1))
    {
        for(uint8_t ii = 0; ii < DATA_ANALYSIS_POINTS; ii++)
        {
            sumBatteryVoltage_mV += batteryVoltage_mV[ii];                               // Average is in mV
            sumPhaseVoltage_mV += phaseVoltage_mV[ii];
        }
        avgBatteryVoltage_mV = round((float) sumBatteryVoltage_mV / DATA_ANALYSIS_POINTS);            // output in mV
        avgPhaseVoltage_mV = round((float) sumPhaseVoltage_mV / DATA_ANALYSIS_POINTS);            // output in mV

        if (avgBatteryVoltage_mV < BATTERY_CRITICALLY_LOW)  // If battery voltage is critically low
        {
            led_display_ErrorPriority(BATTERY_CRITICALLY_LOW_PRIORITY);   // battery low buzzer alert
            /***  battery critically low is a warning, not error  **/
            if (batteryErrorCodePriority > BATTERY_CRITICALLY_LOW_PRIORITY)
            {
                batteryErrorCodePriority = BATTERY_CRITICALLY_LOW_PRIORITY;
                ADArray.batteryCode = BATTERY_VOLTAGE_CRIT_LOW_CODE;
            }
            /* battery level critically low -> power shut down */
            /****  POWER OFF    ****/
            batteryLowCounter++;
            if (batteryLowCounter >= 5)   // 100 * DATA_ANALYTICS_INTERVAL = 30 seconds -> Power Off
            {
                *ptr_da_POWER_ON = 0;   // Set POWER ON to 0, i.e., Power Off
            }
        }
        else if (avgBatteryVoltage_mV > BATTERY_CEILING_VOLTAGE)    // if voltage exceeded ceiling voltage
        {
            /* battery over-voltage -> disable system */
            led_display_ErrorPriority(BATTERY_VOLTAGE_ERROR_PRIORITY);

            if (batteryErrorCodePriority > BATTERY_VOLTAGE_ERROR_PRIORITY)
            {
                batteryErrorCodePriority = BATTERY_VOLTAGE_ERROR_PRIORITY;
                ADArray.batteryCode = BATTERY_VOLTAGE_ERROR_CODE;
            }
            batteryLowCounter = 0;
        }
        else    // else voltage is within normal range
        {
            batteryErrorCodePriority = SYSTEM_NORMAL_PRIORITY;
            ADArray.batteryCode = SYS_NORMAL_CODE;
            batteryLowCounter = 0;
        }
    }
}

static void computeAvgCurrents()
{
    uint32_t    sumBusCurrent_mA = 0;
    uint32_t    sumPhaseCurrent_mA = 0;

    /**** Whenever this func is called, dA_Count = (DATA_ANALYSIS_POINTS - 1)  ****/
    if (dA_Count == (DATA_ANALYSIS_POINTS - 1))
    {
        for(uint8_t ii = 0; ii < DATA_ANALYSIS_POINTS; ii++)
        {
            sumBusCurrent_mA += batteryCurrent_mA[ii];                               // Average is in mV
            sumPhaseCurrent_mA += phaseCurrent_mA[ii];
        }
        avgBusCurrent_mA = round((float) sumBusCurrent_mA / DATA_ANALYSIS_POINTS);            // output in mA
        avgPhaseCurrent_mA = round((float) sumPhaseCurrent_mA / DATA_ANALYSIS_POINTS);
    }
}

/***************************************************************************************************
 * @fn      computeBatteryPercentage
 *
 * @brief   This function computes the battery percentage based on the average battery voltage.
 *
 * @param   Nil
 *
 * @return  avgBatteryPercent = battery_battery_level in battery.h
******************************************************************************************************/
uint8_t computeBatteryPercentage()
{
    int16_t     instantBatteryLevel = 0;
    uint16_t     sumBatteryLevel = 0;

    /**** Whenever this func is called, dA_Count = (DATA_ANALYSIS_POINTS - 1)  ****/
    if (dA_Count == (DATA_ANALYSIS_POINTS - 1))
    {
        for(uint8_t ii = 0; ii < DATA_ANALYSIS_POINTS; ii++)
        {
            /* Calculates instantaneous percentage with compensation for voltage drop */
            instantBatteryLevel = ( batteryVoltage_mV[ii] - BATTERY_MIN_VOLTAGE) * 100 /((BATTERY_MAX_VOLTAGE - batteryCurrent_mA[ii] * VOLTAGE_DROP_COEFFICIENT) - BATTERY_MIN_VOLTAGE);
            if (instantBatteryLevel > 100)
            {
                instantBatteryLevel = 100;              // battery % cannot be greater than 100%
            }
            else if (instantBatteryLevel < 0)
            {
                instantBatteryLevel = 0;                // negative % will be displayed as 0%
            }
            sumBatteryLevel += instantBatteryLevel;
        }

        avgBatteryPercent = round((float) sumBatteryLevel / DATA_ANALYSIS_POINTS);                      // output in %

    }

    return (avgBatteryPercent);
}

/***************************************************************************************************
 * @fn      determineBatteryStatus
 *
 * @brief   This function returns the battery status based on the average battery voltage.
 *
 * @param   avgBatteryVoltage
 *
 * @return  batteryStatus = battery_battery_status in battery.h
******************************************************************************************************/
uint8_t determineBatteryStatus()
{
    if (avgBatteryPercent > LEVEL45PERCENT) {batteryStatus = GLOWING_AQUA;}
    else if (avgBatteryPercent <= LEVEL45PERCENT && avgBatteryPercent > LEVEL34PERCENT) {batteryStatus = GLOWING_GREEN;}
    else if (avgBatteryPercent <= LEVEL34PERCENT && avgBatteryPercent > LEVEL23PERCENT) {batteryStatus = YELLOW;}
    else if (avgBatteryPercent <= LEVEL23PERCENT && avgBatteryPercent > LEVEL12PERCENT) {batteryStatus = ORANGE;}
    else if (avgBatteryPercent <= LEVEL12PERCENT && avgBatteryPercent > LEVEL01PERCENT) {batteryStatus = RED;}
    else {
        batteryStatus = FLASHING_RED;
    }

    led_display_setBatteryStatus(batteryStatus);

    return (batteryStatus);
}
/***************************************************************************************************
 * @fn      computeInstantEconomy
 *
 * @brief   Calculate the instantaneous instantEconomy - Not used for now.
 *
 * @param   None
 *
 * @return  instantEconomy (in W-hr/km)
***************************************************************************************************/
uint16_t computeInstantEconomy(uint32_t deltaPowerConsumption_mWh, uint32_t deltaMileage_dm)
{
    uint16_t instantEconomy_100Whpk = 0;                    // unit in W-hr / km x 100
    if (deltaMileage_dm <= 0)
    {
        instantEconomy_100Whpk = 50000;
        return instantEconomy_100Whpk;
    }                                                       //******** Safeguard from stack overflow due to division by 0
    instantEconomy_100Whpk = (float)(deltaPowerConsumption_mWh * 1000) / deltaMileage_dm;  // unit in W-hr / km x 100

    if (instantEconomy_100Whpk > 50000)
    {
    // Safeguard from data truncation in case economy is greater than declared variable size
        instantEconomy_100Whpk = 50000;
        return instantEconomy_100Whpk;
    }

    return (instantEconomy_100Whpk);                          // Unit in W-hr / km x 100    -> convert to the desired unit before displaying on App
}

/***************************************************************************************************
 * @fn      computeEconomy
 *
 * @brief   This function calculates the economy (i.e.moving average of Whr/km x 100) over the most recent
 *          1.5 hours (1.5 = data_analytics_sampling_time x DATA_EVALUATING_POINTS x NUMINDEX / 1000 / 3600 x 100).
 *
 * @param   None
 *
 * @return  economy (in W-hr/km x 100)
***************************************************************************************************/
uint32_t computeEconomy()
{
    uint32_t overall_economy_100Whpk = 0;                            // unit in W-hr / km x 100
    if ((ADArray.accumMileage_dm - totalMileage0_dm) <= 0)
    { // Safeguard from stack overflow due to division by 0
        overall_economy_100Whpk = 50000;
        return overall_economy_100Whpk;
    }

    overall_economy_100Whpk = (float)(ADArray.accumPowerConsumption_mWh - totalPowerConsumed0_mWh) * 1000 /
                                                    (ADArray.accumMileage_dm - totalMileage0_dm);     // Unit in W-hr / km x 100
    if (overall_economy_100Whpk > 50000)
    {                                   // Safeguard from data truncation in case economy is greater than declared variable size
        overall_economy_100Whpk = 50000;
        return overall_economy_100Whpk;
    }

    return (overall_economy_100Whpk);  // Unit in W-hr / km x 100    -> convert to the desired unit before displaying on App
}

/***************************************************************************************************
 * @fn      computeRange
 *
 * @brief   This function calculates the range remaining in meters
 *
 * @param   batteryCap, avgWhr (in Whr/km)
 *
 * @return  Range
***************************************************************************************************/
uint32_t computeRange()
{
    uint32_t range_m = 0;
    if (ADArray.economy_100Whpk <= 0)
    {                     // Safeguard from stack overflow due to division by 0
        range_m = 0;
        return range_m;   // output in meters  -> convert to the desired unit before displaying on App
    }                                                       // output in meters
    range_m = ((float) ADArray.batteryPercentage * BATTERY_MAX_CAPACITY * BCF / ADArray.economy_100Whpk );

    return (range_m);     // output in metres  -> convert to the desired unit before displaying on App
}
/******************************************************************************************************
 * @fn      computeCO2Saved
 *
 * @brief   This function calculates the hypothetical CO2Saved when compared to driving an average car
 *
 * @param   AccumPowerConsumed, AccumMileage
 *
 * @return  co2Saved
******************************************************************************************************/
uint32_t co2Saved_g;                                        // in grams
uint32_t computeCO2Saved()
{
    co2Saved_g = 0;                                         // in grams
    if (ADArray.accumMileage_dm <= 0)
    {
        co2Saved_g = 0;                                     // Safeguard from stack overflow due to division by 0
        return co2Saved_g;                                  // in grams -> convert to the desired unit before displaying on App
    }
    co2Saved_g = ((float) ADArray.accumMileage_dm * 0.10) * (COEFF01 - ((float) (ADArray.accumPowerConsumption_mWh) /
                                                                        (float) (ADArray.accumMileage_dm)) * 10 * COEFF02);  // result is in grams
    return (co2Saved_g);                                      // in grams -> convert to the desired unit before displaying on App
}

/******************************************************************************************************
 * @fn      data2snvBuffer
 *
 * @brief   Save data to snvBuffer and save snvBuffer to nvsinternal:  Called by data_analytics() and when Power-OFF
 *
 * @param   Nil
 *
 * @return  Nil
******************************************************************************************************/
extern void data2snvBuffer()
{
    // UDIndex is tracked to ensure the data is placed at the correct array location
    UDDataCounter++;

    /******************************************************************************************************
     * Stored data to snvBuffer
     ******************************************************************************************************/
    (*ptr_snvBuffer)[0] = ADArray.ADCounter;
    (*ptr_snvBuffer)[1] = UDDataCounter;
    // SETSIZE = 2
    (*ptr_snvBuffer)[2 + SETSIZE * UDIndex + 0] = ADArray.accumMileage_dm;
    (*ptr_snvBuffer)[2 + SETSIZE * UDIndex + 1] = ADArray.accumPowerConsumption_mWh;

    (*ptr_snvBuffer)[SNV_BUFFER_SIZE - 4] = brake_and_throttle_getSpeedMode();
    (*ptr_snvBuffer)[SNV_BUFFER_SIZE - 3] = UnitSelectDash;
    (*ptr_snvBuffer)[SNV_BUFFER_SIZE - 2] = LIGHT_MODE_AUTO; //*ptr_lightmode;
    (*ptr_snvBuffer)[SNV_BUFFER_SIZE - 1] = *ptr_uptimeMinutes;

    /******************************************************************************************************
     * Reset of UDDataCounter can only be reset correctly at multiples of UDARRAYSIZE
     * Each count equals to [(DATA_ANALYSIS_POINTS -1) * GPT_TIME * 2 * UDTRIGGER] minutes
     * If each count equals 10 mins, it will take 81715.5 years to reach this count.
     * We therefore should never reach the need for reset in reality.
     ******************************************************************************************************/
    if (UDDataCounter >= resetUDDataCounter )
    {
            UDDataCounter = 0;  // Reset UDDataCounter
    }
    /**  Re-initialise UDARRAY Data  **/
    data_analytics_getSNVData();

    /******************************************************************************************************
     * Whenever data2snvBuffer() is triggered, reset UDTriggerCounter to zero
     * reset sumDeltaPowerConsumed_mWh and sumDeltaMileage_dm to zero
     ******************************************************************************************************/
    UDTriggerCounter = 0;
    sumDeltaPowerConsumed_mWh = 0;
    sumDeltaMileage_dm = 0;
}

/*************************************************************************************************************
 * @fn      data_analytics_LEDSpeed
 *
 * @brief   calculate speeds, send RPM & speed to client (App) and set speed display on dashboard
 *
 * @param   dA_Count
 *
 * @return  Nil
******************************************************************************************************/
uint8_t dashSpeed;  // for debugging only
uint16_t rawSpeed_100kph;

extern void data_analytics_LEDSpeed()
{
    rawSpeed_100kph = ((float) speed_cmps[dA_Count] * 3.6 );                    // in 100*km/hr
    dashSpeed = ((float) speed_cmps[dA_Count] * 0.036 * lenConvFactorDash);     // in km/hr or mph

    /* Send dashSpeed to LED display */
    led_display_setDashSpeed(dashSpeed);
}


/*********************************************************************
 * @fn      data_analytics_ptrUnitSelectDash
 *
 * @brief   call data_analytics_ptrUnitSelectDash() to give the "pointer to UnitSelectDash" to the calling function
 *
 * @param   None
 *
 * @return  "pointer to UnitSelectDash"
 *********************************************************************/
extern void* data_analytics_ptrUnitSelectDash()
{
    return (&UnitSelectDash);
}


/*********************************************************************
 * @fn      data_analytics_changeUnitSelectDash
 *
 * @brief   call this function to change/toggle Unit and set led display
 *
 * @param   None
 *
 * @return  None
 *********************************************************************/
extern void data_analytics_changeUnitSelectDash()
{
    switch(UnitSelectDash)
    {
    case SI_UNIT:
        {
            lenConvFactorDash = 1;
            break;
        }
    case IMP_UNIT:
        {
            lenConvFactorDash = KM2MILE;
            break;
        }
    default:
        break;
    }
    // send UnitSelectDash to led_display.c
    led_display_setUnitSelectDash(UnitSelectDash);  // calling here will execute led display immediately without wait
}


/***************************************************************************/
/* External functions call this function to return the initial speed mode  */
/***************************************************************************/
extern uint8_t data_analytics_getSpeedmodeInit()
{
    return (speedmode_init);
}
/***************************************************************************/
/* External functions call this function to return the initial dashboard unit  */
/***************************************************************************/
extern uint8_t data_analytics_getDashunitInit()
{
    return (UnitSelectDash);
}
/***************************************************************************/
/* External functions call this function to return the initial light mode  */
/***************************************************************************/
extern uint8_t data_analytics_getLightmodeInit()
{
    return (lightmode_init);
}

/***************************************************************************
 * @fn      coefficient_array_init
 *
 * @brief   Used to initialize the coefficient_array used in Simpson's 1/3 rule
 *
 * @param   Nil
 *
 * @return  Nil
******************************************************************************************************/
static void coefficient_array_init()
{ //change number of data points if necessary

    for(uint8_t kk = 0; kk < DATA_ANALYSIS_POINTS; kk++)
    {
        if( kk == 0 || kk == ( DATA_ANALYSIS_POINTS - 1 ))
        {
            //*((*ptrc) + kk) = 1;
            coefficient_array[kk] = 1;
        }
        else
        {
            if (kk % 2 == 1)
            {
                coefficient_array[kk] = 4;
            }
            else
            {
                coefficient_array[kk] = 2;
            }
        }
    }
}

/***************************************************************************************************
 * @fn      re_Initialize
 *
 * @brief   This function re-initialize the data arrays after each data analysis loop.
 *
 * @param   none
 *
 * @return  none
******************************************************************************************************/
static void re_Initialize()
{
    //  Re-initialize arrays after completing each computation loop
    for(uint8_t kk = 0; kk < DATA_ANALYSIS_POINTS; kk++)
    { // carry over the last data set to the 1st position [0] of the new dataset and reset all other to zero
        if(kk == 0)
        {
            rpm[0] = rpm[DATA_ANALYSIS_POINTS - 1];
            speed_cmps[0] = speed_cmps[DATA_ANALYSIS_POINTS - 1];
            batteryCurrent_mA[0] = batteryCurrent_mA[DATA_ANALYSIS_POINTS - 1];
            batteryVoltage_mV[0] = batteryVoltage_mV[DATA_ANALYSIS_POINTS - 1];
            phaseCurrent_mA[0] = phaseCurrent_mA[DATA_ANALYSIS_POINTS - 1];
            phaseVoltage_mV[0] = phaseVoltage_mV[DATA_ANALYSIS_POINTS - 1];
            heatSinkTempOffset50_C[0] = heatSinkTempOffset50_C[DATA_ANALYSIS_POINTS - 1];        // temperature can be negative
            motorTempOffset50_C[0] = motorTempOffset50_C[DATA_ANALYSIS_POINTS - 1];
        }
        else
        {
            rpm[kk] = 0;
            speed_cmps[kk] = 0;
            batteryCurrent_mA[kk] = 0;
            batteryVoltage_mV[kk] = 0;
            phaseCurrent_mA[kk] = 0;
            phaseVoltage_mV[kk] = 0;
            heatSinkTempOffset50_C[kk] = 0;        // temperature can be negative
            motorTempOffset50_C[kk] = 0;
        }
    }
}

/***************************************************************************************************
 * @fn      data_analytics_setCharVal
 *
 * @brief   This function set the characteristic values in profiles -> sends data to Mobile Application
 *
 * @param   Nil
 *
 * @return  nil
******************************************************************************************************/
static void data_analytics_setCharVal()
{
    /************************  Dashboard services   *************************************/
    ptr_da_charVal = (ptr_da_profileCharVal->ptr_dash_charVal->ptr_ADdataID);
    profile_setCharVal(ptr_da_charVal, DASHBOARD_ADCOUNTER_LEN, ADArray.ADCounter);

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_dash_charVal->ptr_dashErrorCode);
    profile_setCharVal(ptr_da_charVal, DASHBOARD_ERROR_CODE_LEN, ADArray.dashboardErrorCode);
    //light_mode setCharVal is performed in lights.c
    //light status setCharVal is performed in lights.c
    //speed mode setCharVal is performed in brake_and_throttle.c
    //power on time setCharVal is performed in power_on_time.c
    //device uptime setCharVal is performed in power_on_time.c

    /************************  Controller services     *************************************/
//    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_motorSpeed);
//    profile_setCharVal(ptr_da_charVal, CONTROLLER_MOTOR_SPEED_LEN, ADArray.avgSpeed_100kph);
//
//    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_motorRPM);
//    profile_setCharVal(ptr_da_charVal, CONTROLLER_MOTOR_RPM_LEN, avgRPM);

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_voltage);
    profile_setCharVal(ptr_da_charVal, CONTROLLER_VOLTAGE_LEN, ADArray.avgPhaseVoltage_mV);

    /** current is very dynamic. It does not make much sense to make current available to the user.  **/
//    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_current);
//    profile_setCharVal(ptr_da_charVal, CONTROLLER_CURRENT_LEN, ADArray.phaseCurrent_mA);

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_totalDistance);
    profile_setCharVal(ptr_da_charVal, CONTROLLER_TOTAL_DISTANCE_TRAVELLED_LEN, ADArray.accumMileage_dm);

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_totalEnergy);
    profile_setCharVal(ptr_da_charVal, CONTROLLER_TOTAL_ENERGY_CONSUMPTION_LEN, ADArray.accumPowerConsumption_mWh);

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_instantEconomy);
    profile_setCharVal(ptr_da_charVal, CONTROLLER_INSTANT_ECONOMY_LEN, ADArray.instantEconomy_100Whpk);

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_overallEfficiency);
    profile_setCharVal(ptr_da_charVal, CONTROLLER_OVERALL_EFFICIENCY_LEN, ADArray.economy_100Whpk);

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_range);
    profile_setCharVal(ptr_da_charVal, CONTROLLER_RANGE_LEN, ADArray.range_m);

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_co2saved);
    profile_setCharVal(ptr_da_charVal, CONTROLLER_CO2SAVED_LEN, ADArray.co2Saved_g);

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_heatSinkTempOffset50);
    profile_setCharVal(ptr_da_charVal, CONTROLLER_HEAT_SINK_TEMPERATURE_LEN, ADArray.HeatSinkTempOffset50_C); // subtract 50 to display on APP

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_motorTempOffset50);
    profile_setCharVal(ptr_da_charVal, CONTROLLER_MOTOR_TEMPERATURE_LEN, ADArray.motorTempOffset50_C); // subtract 50 to display on APP

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_cont_charVal->ptr_controllerErrorCode);
    profile_setCharVal(ptr_da_charVal, CONTROLLER_ERROR_CODE_LEN, ADArray.controllerErrorCode);

    /*************************************  Battery services   *************************************/
    ptr_da_charVal = (ptr_da_profileCharVal->ptr_batt_charVal->ptr_batterayErrorCode);
    profile_setCharVal(ptr_da_charVal, BATTERY_BATTERY_ERROR_CODE_LEN, ADArray.batteryCode);

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_batt_charVal->ptr_batteryLevel);
    profile_setCharVal(ptr_da_charVal, BATTERY_BATTERY_LEVEL_LEN, ADArray.batteryPercentage);

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_batt_charVal->ptr_batteryStatus);
    profile_setCharVal(ptr_da_charVal, BATTERY_BATTERY_STATUS_LEN, ADArray.batteryStatus);

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_batt_charVal->ptr_batteryTempOffset50);
    profile_setCharVal(ptr_da_charVal, BATTERY_BATTERY_TEMPERATURE_LEN, ADArray.batteryTempOffset50_C); // subtract 50 to display on APP

    ptr_da_charVal = (ptr_da_profileCharVal->ptr_batt_charVal->ptr_batteryVoltage);
    profile_setCharVal(ptr_da_charVal, BATTERY_BATTERY_VOLTAGE_LEN, ADArray.avgBatteryVoltage_mV);

    /** current is very dynamic. It does not make much sense to make current available to the user.  **/
//    ptr_da_charVal = (ptr_da_profileCharVal->ptr_batt_charVal->ptr_batteryCurrent);
//    profile_setCharVal(ptr_da_charVal, BATTERY_BATTERY_CURRENT_LEN, ADArray.avgBusCurrent_mA);
}


