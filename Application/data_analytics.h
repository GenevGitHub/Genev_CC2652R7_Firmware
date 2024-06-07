/*
 * data_analytics.h
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#ifndef APPLICATION_DATA_ANALYTICS_H_
#define APPLICATION_DATA_ANALYTICS_H_

#ifdef __cplusplus
extern "C"
{
#endif
/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <ti/sysbios/knl/Task.h>
#include <math.h>

#include "Hardware/gGo_device_params.h"

#include "Application/motor_control.h"

#include "Profiles/dashboard_profile.h"
#include "Profiles/controller_profile.h"
#include "Profiles/battery_profile.h"
#include "Profiles/profile_charVal.h"

/*********************************************************************
* CONSTANTS
*/
/*********************************************************************************************
 *  NVS Data Information
 *********************************************************************************************/
#define SETSIZE                         2
#define UDARRAYSIZE                     13              // Number of Usage Dataset stored in flash memory
#define UDTRIGGER                      100              // Number of integrations between saving data to snv_internal_80
// DATA_ANALYSIS_POINTS must be an ODD number because numerical integration uses Simpson's 1/3 rule
#define DATA_ANALYSIS_POINTS            21              // Number of elements in each numerical integration

// Note:    Data Analysis Interval = (DATA_ANALYSIS_POINTS - 1) x DATA_ANALYSIS_SAMPLING_TIME
//          Averge_economy refresh interval =  Data Analysis Interval x UDTRIGGER
//          Total time interval stored in memory = Averge_economy refresh interval x UDARRAYSIZE
//          (DATA_ANALYSIS_POINTS-1) x DATA_ANALYSIS_SAMPLING_TIME = Integration_time.  Integration_time x UDTRIGGER = NVS_Data_Interval.  NVS_Data_Interval x UDARRAYSIZE = Total_Data_Interval
// Option 1:  (21-1) x 400ms = 8000ms.  8000ms x 45 = 360000ms = 6 minutes. 6 minutes x 20 = 120 minutes = 2 hours
// Option 2:  (13-1) x 400ms = 4800ms.  4800ms x 75 = 360000ms = 6 minutes. 6 minutes x 10 = 60 minutes (1 hour)
// Option 3:  (13-1) x 400ms = 4800ms.  4800ms x 125 = 600000ms = 10 minutes. 10 minutes x 6 = 60 minutes = 1 hour
// Option 4:  (17-1) x 300ms = 4800ms.  4800ms x 75 = 360000ms = 6 minutes. 6 minutes x 10 = 60 minutes = 1 hour
// Option 5:  (21-1) x 2 x 150ms = 6000ms.  6000ms x 100 = 600000ms = 10 minutes. 10 minutes x 13 = 130 minutes = 2 hour and 10 minutes

//typedef
// This set of data is stored in ram, and to be stored in flash (NVS) memory
typedef struct userData{
        uint32_t UDCounter;                             // to Cloud - require device parameters
        uint32_t totalPowerConsumption_mWh;             // to Cloud, App display input - require device parameters
        uint32_t totalMileage_dm;                       // to Cloud, App display input - require device parameters
}UD_t;

// This set of data is temporary on the dashboard - this set of data is sent to the APP for displaying when connected with BLE
typedef struct appData{                                 // is appData needed here??
        uint32_t    ADCounter;                             // length = 4 . to Cloud - require device parameters

        uint32_t    accumPowerConsumption_mWh;             // length = 4 . to Cloud, App display input - require device parameters
        uint32_t    accumMileage_dm;                       // length = 4 . to Cloud, App display input - require device parameters
        uint32_t    range_m;                               // length = 4 . App display input - require device parameters
        uint32_t    co2Saved_g;                            // length = 4 . App display input
        uint32_t    economy_100Whpk;                       // length = 4 . App display input - require retrieving saved data
        uint16_t    instantEconomy_100Whpk;                // length = 2 . Disable for now  // App display input - require retrieving saved data
        uint16_t    phaseVoltage_mV;                       // Controller Phase Current
        uint16_t    phaseCurrent_mA;                       // Controller Phase Current - no need to display current on the mobile app

        uint16_t    avgBatteryVoltage_mV;                  // length = 2 . to Cloud - require device parameters
        uint16_t    avgBatteryCurrent_mA;
        uint8_t     controllererrorCode;                             // length = 2 . to Cloud, Both LED display and App Display (May have to separate error code into their respective services)
        uint8_t     avgSpeed_kph;                           // length = 1 . to Cloud - require device parameters
        uint8_t     HeatSinkTempOffset50_C;                // temperature can be sub-zero
        uint8_t     motorTempOffset50_C;                      // temperature can be sub-zero
        uint8_t     dashboarderrorCode;
        uint8_t     batteryerrorCode;
        uint8_t     batteryPercentage;                      // length = 1 (0-100%). App display input - require device parameters
        uint8_t     batteryStatus;                          // length = 1 . Both LED display and App display input - require device parameters
        uint8_t     batteryTempOffset50_C;

        uint8_t     speedmode;
        uint8_t     lightmode;
        uint8_t     lightstatus;

}AD_t;        // 4-4-4-4-4-2-2-2-2-2-2-1-1-1-1-1 decreasing byte size minimizes the amount of struct padding


/*********************************************************************
 * FUNCTIONS
 *********************************************************************/
extern void data_analytics_init();
extern void data_analytics_buzzerStatusRegister(uint8_t *ptrbuzzerStatus);

extern void data_analytics_setReadBuffer(uint32_t (*ptrsnvbuf)[]);
extern uint8_t data_analytics_getSpeedmodeInit();
extern uint8_t data_analytics_getLightmodeInit();
extern uint8_t data_analytics_getDashunitInit();

extern void data_analytics_MCUDArrayRegister(MCUD_t (*ptrMCUD));
extern void data_analytics_dashErrorCodeStatusRegister(uint8_t *ptrdashboardErrorCodeStatus);

//Battery status related Function declaration
extern uint8_t computeBatteryPercentage( void );
extern uint8_t determineBatteryStatus( void );

extern uint8_t data_analytics_getBatteryPercentage(void);
//extern uint8_t data_analytics_getUnitSelectDash( void );
extern void* data_analytics_ptrUnitSelectDash();
extern void data_analytics_changeUnitSelectDash(void);

//Global Functions declaration
extern void data_analytics_init( void );
static void coefficient_array_init( void );
extern void data_analytics_LEDSpeed();
extern void data_analytics( void );
extern void data2UDArray( void );

//static void dataSim();
static void re_Initialize( void );
static void data_analytics_setUDArrayData( void );
extern void data_analytics_sampling();

extern void data_analytics_Main( void );
//extern void get_ADArrayData(uint8_t paramID);
static void data_analytics_setCharVal(void);

extern void data_analytics_setError( uint8_t errorCode, uint8_t errorStatus );

extern void data_analytics_timerInterruptHandler( void );

//Performance related Function declaration
extern uint32_t computePowerConsumption( void ); // output in mW-hr
extern uint32_t computeDistanceTravelled( void );// output in decimeter
extern uint8_t  computeAvgSpeed(uint32_t deltaMileage_dm);   // output in km/hr
extern int8_t   computeAvgHeatSinkTemperature( void );         // output in degrees celsius
extern uint32_t computeAvgBatteryVoltage( void ); // output in mV
extern uint16_t computeInstantEconomy(uint32_t deltaPowerConsumption_mWh, uint32_t deltaMileage_dm); // unit in W-hr / km x 100
extern uint32_t computeEconomy( void );  // unit in W-hr / km x 100
extern uint32_t computeRange( void ); // output in metres
extern uint32_t computeCO2Saved( void ); // in g
extern int8_t   computeMotorTemperature( void ); // in degrees Celsius

extern uint8_t data_analytics_getInitSpeedMode(void);
extern void data_analytics_errorStatus( uint8_t errorStatus );


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_DATA_ANALYTICS_H_ */
