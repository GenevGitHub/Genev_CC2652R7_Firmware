/*
 * profile_charVal.h
 *
 *  Created on: 30 May 2024
 *      Author: Chee
 */

#ifndef PROFILES_PROFILE_CHARVAL_H_
#define PROFILES_PROFILE_CHARVAL_H_

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
#include <string.h>
#include <bcomdef.h>        // contains #include "hal_types.h" that defines uint8

#include "Hardware/gGo_device_params.h"

#include "Profiles/controller_profile.h"
#include "Profiles/dashboard_profile.h"
#include "Profiles/battery_profile.h"

/*********************************************************************
* CONSTANTS
*/

/*********************************************************************
 * TYPEDEFS
 */
typedef struct profileCharVal{
    dashboardCharVal_t*     ptr_dash_charVal;
    controllerCharVal_t*    ptr_cont_charVal;
    batteryCharVal_t*       ptr_batt_charVal;
}profileCharVal_t;

/*********************************************************************
 * MACROS
 */



/*********************************************************************
 * API FUNCTIONS
 */
extern void profile_charVal_init();
extern void* profile_charVal_profileCharValRegister();
extern void profile_setCharVal(uint8 *ptr_CharVal, uint8_t payloadLength, uint32_t payload);


#ifdef __cplusplus
}
#endif

#endif /* PROFILES_PROFILE_CHARVAL_H_ */
