/*
 * dashboard_profile.h
 *
 *Description:    This file contains the Dashboard service definitions and
 *                 prototypes.
 *
 *  Created on: 16 May 2024
 *      Author: Chee
 */

#ifndef PROFILES_DASHBOARD_PROFILE_H_
#define PROFILES_DASHBOARD_PROFILE_H_

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
#define DASHBOARD_SERV_UUID                     0x6800  //0x1811  // alert notification

//  Characteristic definition - on Client(App) side
#define DASHBOARD_ERROR_CODE                    0
#define DASHBOARD_SPEED_MODE                    1
#define DASHBOARD_LIGHT_STATUS                  2
#define DASHBOARD_LIGHT_MODE                    3
#define DASHBOARD_POWER_ON_TIME                 4
#define DASHBOARD_ADCOUNTER                     5

#define DASHBOARD_SPEED_MODE_UUID               0x6801
#define DASHBOARD_LIGHT_STATUS_UUID             0x6802
#define DASHBOARD_LIGHT_MODE_UUID               0x6803
#define DASHBOARD_POWER_ON_TIME_UUID            0x6804
#define DASHBOARD_ADCOUNTER_UUID                0x6805
#define DASHBOARD_ERROR_CODE_UUID               0x6806  //0x2A43

#define DASHBOARD_ERROR_CODE_LEN                1
#define DASHBOARD_SPEED_MODE_LEN                1
#define DASHBOARD_LIGHT_STATUS_LEN              1
#define DASHBOARD_LIGHT_MODE_LEN                1
#define DASHBOARD_POWER_ON_TIME_LEN             2
#define DASHBOARD_ADCOUNTER_LEN                 4

// Dashboard Error Codes
#define DASHBOARD_NORMAL                        40
#define DASHBOARD_COMMUNICATION_ERROR           41
#define THROTTLE_SENSOR_ABNORMAL                42
#define BRAKE_SENSOR_ABNORMAL                   43

/*********************************************************************
 * TYPEDEFS
 */
typedef struct dashboardCharVal{
    uint8*    ptr_ADdataID;
    uint8*    ptr_powerOnTime;
    uint8*    ptr_dashErrorCode;
    uint8*    ptr_speedMode;
    uint8*    ptr_lightStatus;
    uint8*    ptr_lightMode;
}dashboardCharVal_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*DashboardChange_t)( uint8 paramID ); //void and pointer function with param

typedef struct
{
  DashboardChange_t        pfnChangeCb;  // Called when characteristic value changes
} DashboardCBs_t;

/*********************************************************************
 * API FUNCTIONS
 */

extern void Dashboard_profile_init(void);
extern void* Dashboard_CharValRegister(void);

/*
 * Dashboard_AddService- Initializes the Dashboard service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t Dashboard_AddService( void );

/*
 * Dashboard_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Dashboard_RegisterAppCBs( DashboardCBs_t *appCallbacks );

/*
 * Dashboard_SetParameter - Set a Dashboard parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Dashboard_SetParameter( uint8 param, uint8 len, void *value );

/*
 * Dashboard_GetParameter - Get a Dashboard parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Dashboard_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* PROFILES_DASHBOARD_PROFILE_H_ */
