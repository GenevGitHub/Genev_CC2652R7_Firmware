/*
 * controller_profile.h
 *
 * Description:    This file contains the Controller service definitions and
 *                 prototypes.
 *
 *  Created on: 16 May 2024
 *      Author: Chee
 */

#ifndef PROFILES_CONTROLLER_PROFILE_H_
#define PROFILES_CONTROLLER_PROFILE_H_

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
#define CONTROLLER_SERV_UUID                        0x181C

//  Characteristic definition
#define CONTROLLER_VOLTAGE                          0
#define CONTROLLER_CURRENT                          1
#define CONTROLLER_HEAT_SINK_TEMPERATURE            2
#define CONTROLLER_ERROR_CODE                       3
#define CONTROLLER_MOTOR_RPM                        4
#define CONTROLLER_MOTOR_SPEED                      5
#define CONTROLLER_TOTAL_DISTANCE_TRAVELLED         6
#define CONTROLLER_TOTAL_ENERGY_CONSUMPTION         7
#define CONTROLLER_OVERALL_EFFICIENCY               8
#define CONTROLLER_RANGE                            9
#define CONTROLLER_CO2SAVED                         10
#define CONTROLLER_INSTANT_ECONOMY                  11
#define CONTROLLER_MOTOR_TEMPERATURE                12

#define CONTROLLER_VOLTAGE_UUID                     0x2B18  // 0x2AE1
#define CONTROLLER_CURRENT_UUID                     0x2AEE
#define CONTROLLER_HEAT_SINK_TEMPERATURE_UUID       0x2A6E
#define CONTROLLER_ERROR_CODE_UUID                  0x2A43  // 0x2A3F
#define CONTROLLER_MOTOR_RPM_UUID                   0x7805
#define CONTROLLER_MOTOR_SPEED_UUID                 0x7806
#define CONTROLLER_TOTAL_DISTANCE_TRAVELLED_UUID    0x7807
#define CONTROLLER_TOTAL_ENERGY_CONSUMPTION_UUID    0x2AF2  // 0x7805  //
#define CONTROLLER_OVERALL_EFFICIENCY_UUID          0X7808
#define CONTROLLER_RANGE_UUID                       0x7809
#define CONTROLLER_CO2SAVED_UUID                    0x780A
#define CONTROLLER_INSTANT_ECONOMY_UUID             0x780B
#define CONTROLLER_MOTOR_TEMPERATURE_UUID           0x2A1C  // 0x2A6E

#define CONTROLLER_VOLTAGE_LEN                      2
#define CONTROLLER_CURRENT_LEN                      2
#define CONTROLLER_HEAT_SINK_TEMPERATURE_LEN        1
#define CONTROLLER_ERROR_CODE_LEN                   1
#define CONTROLLER_MOTOR_RPM_LEN                    2
#define CONTROLLER_MOTOR_SPEED_LEN                  2
#define CONTROLLER_TOTAL_DISTANCE_TRAVELLED_LEN     4
#define CONTROLLER_TOTAL_ENERGY_CONSUMPTION_LEN     4
#define CONTROLLER_OVERALL_EFFICIENCY_LEN           4
#define CONTROLLER_RANGE_LEN                        4
#define CONTROLLER_CO2SAVED_LEN                     4
#define CONTROLLER_INSTANT_ECONOMY_LEN              2
#define CONTROLLER_MOTOR_TEMPERATURE_LEN            1

// Controller Error Codes
#define CONTROLLER_NORMAL                           0x00
#define PHASE_CURRENT_ABNORMAL                      0x2A
#define MOSFET_ABNORMAL                             0x2E
//#define OPAMP_ABNORAML
#define GATE_DRIVER_ABNORMAL                        0x2C
#define HEATSINK_TEMPERATURE_ABNORMAL               0x2F
//  Motor Error Codes
#define MOTOR_NORMAL                                0x00
#define HALL_SENSOR_ABNORMAL                        0x3A
#define MOTOR_TEMPERATURE_ABNORMAL                  0x3C
/*********************************************************************
 * TYPEDEFS
 */
typedef struct controllerCharVal{
    uint8*    ptr_totalDistance;
    uint8*    ptr_totalEnergy;
    uint8*    ptr_overallEfficiency;
    uint8*    ptr_range;
    uint8*    ptr_co2saved;
    uint8*    ptr_voltage;
    uint8*    ptr_current;
    uint8*    ptr_motorRPM;
    uint8*    ptr_motorSpeed;
    uint8*    ptr_instantEconomy;
    uint8*    ptr_heatSinkTempOffset50;
    uint8*    ptr_motorTempOffset50;
    uint8*    ptr_controllerErrorCode;
}controllerCharVal_t;


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */
// Callback when a characteristic value has changed
typedef void (*ControllerChange_t)( uint8 paramID );

typedef struct
{
  ControllerChange_t        pfnChangeCb;  // Called when characteristic value changes
} ControllerCBs_t;

/*********************************************************************
 * API FUNCTIONS
 */
extern void* Controller_CharValRegister(void);
extern void Controller_profile_init();


/*
 * Controller_AddService- Initializes the Battery service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t Controller_AddService( void );

/*
 * Controller_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Controller_RegisterAppCBs( ControllerCBs_t *appCallbacks );

/*
 * Controller_SetParameter - Set a Battery parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Controller_SetParameter( uint8 param, uint8 len, void *value );

/*
 * Controller_GetParameter - Get a Battery parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Controller_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* PROFILES_CONTROLLER_PROFILE_H_ */
