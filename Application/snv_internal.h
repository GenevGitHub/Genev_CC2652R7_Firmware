/*
 *  snv_internal.h
 *  Brief:  When using and working with BLE functions, NVS must be accessed using
 *  OSAL_SNV library.  OSAL SNV operations are defined through ICALL
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
#include "Application/simple_peripheral.h"

/*********************************************************************************************
 *  Constants
 *********************************************************************************************/
#define SNV_BUFFER_SIZE     32

#define RESETCODE01              123456789
#define RESETCODE02              987654321

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
extern void snv_internal_resetSNVdata();  // Usage Data (UD) Array
extern uint8_t snv_internal_getInitSpeedMode(void);
extern uint8_t snv_internal_getInitLightMode(void);
extern uint8_t snv_internal_getInitLightMode(void);
extern void* snv_internal_setReadBuffer(uint32_t (*ptr_snvBuffer)[]);


#ifdef _cplusplus
}
#endif

#endif /* APPLICATION_SNV_INTERNAL_H_ */
