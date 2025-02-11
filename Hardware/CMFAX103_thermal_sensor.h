/*
 * CMFAX103_thermal_sensor.h
 *
 *  Created on: 23 Jan 2025
 *      Author: Chee
 */

#ifndef HARDWARE_CMFAX103_THERMAL_SENSOR_H_
#define HARDWARE_CMFAX103_THERMAL_SENSOR_H_

#ifdef _cplusplus
extern "C"
{
#endif


/*********************************************************************
 * INCLUDES
 */

/* Library Header files */
#include <stdio.h>
#include <stdint.h>
#include <math.h>

/*********************************************************************************************
 *  Constants
 *********************************************************************************************/

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * MACROS
 */



/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/
extern int motorTempOffset50C(uint32_t R_value);


#ifdef _cplusplus
}
#endif


#endif /* HARDWARE_CMFAX103_THERMAL_SENSOR_H_ */
