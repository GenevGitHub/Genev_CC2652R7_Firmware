/*
 * battery_profile.h
 *
 * Description:    This file contains the Battery service definitions and
 *                 prototypes.
 *
 *  Created on: 16 May 2024
 *      Author: Chee
 */

#ifndef PROFILES_BATTERY_PROFILE_H_
#define PROFILES_BATTERY_PROFILE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <bcomdef.h>

#include "Hardware/gGo_device_params.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
* CONSTANTS
*/
// Service UUID
//#define BATTERY_SERV_UUID                       0x5810
#define BATTERY_PROFILE_SERV_UUID                 0x180F

//  Characteristic definition - in percentage
#define BATTERY_BATTERY_LEVEL                   0
//  Characteristic defines <- should display battery voltage in mV
#define BATTERY_BATTERY_VOLTAGE                 1
//  Characteristic defines <- currently no battery temperature sensor / measurement
#define BATTERY_BATTERY_TEMPERATURE             2
//  Characteristic defines <- currently no battery error code
#define BATTERY_BATTERY_ERROR_CODE              3
//  Characteristic defines -> display battery status from 1 to 5
#define BATTERY_BATTERY_STATUS                  4
//  Characteristic defines <- should display battery current in mA
#define BATTERY_BATTERY_CURRENT                 5

#define BATTERY_BATTERY_LEVEL_UUID              0x2A19  // battery level in percentage
#define BATTERY_BATTERY_VOLTAGE_UUID            0x2B18
#define BATTERY_BATTERY_TEMPERATURE_UUID        0x2A6E
#define BATTERY_BATTERY_ERROR_CODE_UUID         0x2A43  // 0x5804    // To be assigned/created UUID code
#define BATTERY_BATTERY_STATUS_UUID             0x2BED  // battery level status
#define BATTERY_BATTERY_CURRENT_UUID            0x2AEE

#define BATTERY_BATTERY_LEVEL_LEN               1
#define BATTERY_BATTERY_VOLTAGE_LEN             2
#define BATTERY_BATTERY_CURRENT_LEN             2
#define BATTERY_BATTERY_TEMPERATURE_LEN         1
#define BATTERY_BATTERY_ERROR_CODE_LEN          1
#define BATTERY_BATTERY_STATUS_LEN              1

///**************************  Characteristic Value ******************************/
//// Characteristic "Battery_Level" Value variable
//static uint8 Battery_Battery_LevelVal[BATTERY_BATTERY_LEVEL_LEN] = {100};  //{0};
//// Characteristic "BATTERY_VOLTAGE" Value variable
//static uint8 Battery_Battery_VoltageVal[BATTERY_BATTERY_VOLTAGE_LEN] = {0x88, 0x90};  // (Low Byte, High Byte} = {0x88, 0x90} = 37,000 mV : voltage is in mV
//// Characteristic "BATTERY_TEMPERATURE" Value variable
//static uint8 Battery_Battery_TemperatureVal[BATTERY_BATTERY_TEMPERATURE_LEN] = {40};
//// Characteristic "BATTERY_ERROR_CODE" Value variable
//static uint8 Battery_Battery_Error_CodeVal[BATTERY_BATTERY_ERROR_CODE_LEN] = {0};
//// Characteristic "BATTERY_STATUS" Value variable
//static uint8 Battery_Battery_StatusVal[BATTERY_BATTERY_STATUS_LEN] = {5};   // = GLOWING_AQUA in dataAnalysis.h
//// Characteristic "BATTERY_CURRENT" Value variable
//static uint8 Battery_Battery_CurrentVal[BATTERY_BATTERY_CURRENT_LEN] = {0xA0, 0x0F};


// Battery Error Code
#define BATTERY_NORMAL                          0x00
#define BMS_COMMUNICATION_ERROR                 0x1C
#define BATTERY_TEMPERATURE_ABNORMAL            0x1A
#define BATTERY_OVER_CURRENT                    0x1E

/*********************************************************************
 * TYPEDEFS
 */
typedef struct batteryCharVal{
    uint8*    ptr_batteryVoltage;
    uint8*    ptr_batteryLevel;
//    uint8*    ptr_batteryCurrent;
    uint8*    ptr_batteryStatus;
    uint8*    ptr_batteryTempOffset50;
    uint8*    ptr_batterayErrorCode;
}batteryCharVal_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*BatteryChange_t)( uint8 paramID );

typedef struct
{
  BatteryChange_t        pfnChangeCb;  // Called when characteristic value changes
} BatteryCBs_t;

/*********************************************************************
 * API FUNCTIONS
 */

extern void Battery_profile_init(void);
extern void* Battery_CharValRegister(void);

/*
 * Battery_AddService- Initializes the Battery service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t Battery_AddService( void );

/*
 * Battery_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Battery_RegisterAppCBs( BatteryCBs_t *appCallbacks );

/*
 * Battery_SetParameter - Set a Battery parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Battery_SetParameter( uint8 param, uint8 len, void *value );

/*
 * Battery_GetParameter - Get a Battery parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Battery_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* PROFILES_BATTERY_PROFILE_H_ */
