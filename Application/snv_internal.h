/*
 * snv_internal.h
 *
 *  Created on: 30 Apr 2024
 *      Author: Chee
 */

#ifndef APPLICATION_SNV_INTERNAL_H_
#define APPLICATION_SNV_INTERNAL_H_

#ifdef _cplusplus
extern "C"
{
#endif


/*********************************************************************
 * INCLUDES
 */

/* Library Header files */
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "Hardware/gGo_device_params.h"

/*********************************************************************************************
 *  Constants
 *********************************************************************************************/
#define SNV_BUFFER_SIZE     32

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
extern void snv_internal_dummyUDArray();  // Usage Data (UD) Array
extern uint8_t snv_internal_getInitSpeedMode(void);
extern uint8_t snv_internal_getInitLightMode(void);
extern uint8_t snv_internal_getInitLightMode(void);
extern void* snv_internal_setReadBuffer(uint32_t (*ptr_readBuffer)[]);
extern void* snv_internal_getUDBuffer();


#ifdef _cplusplus
}
#endif

#endif /* APPLICATION_SNV_INTERNAL_H_ */
