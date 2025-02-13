
/*
 * NTCG163JF103FT.h
 *
 *  NTCG163JF103FT Thermal Sensor
 *
 *  Created on: 24 Jan 2025
 *      Author: Chee
 */

#ifndef HARDWARE_NTCG163JF103FT_H_
#define HARDWARE_NTCG163JF103FT_H_

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
#define TEMP0               298.15
#define RESISTANCE0         10000
#define BCONSTANT_AV        3425.5
#define UPPER_RESISTANCE    251200  // correspond to -40 degree C
#define LOWER_RESISTANCE    500     // correspond to 130 degree C
#define ABSOLUTE_TEMP       273.15

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
extern int heatSinkTempOffset50C(uint32_t R_value);


#ifdef _cplusplus
}
#endif






#endif /* HARDWARE_NTCG163JF103FT_H_ */
