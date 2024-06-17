/*
 * snv_internal.c
 *
 *  Created on: 30 Apr 2024
 *      Author: Chee
 */

/*********************************************************************
 * INCLUDES
 */
#include "snv_internal.h"

#include "Hardware/gGo_device_params.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define DUMMY_NVS                    1
//#undef  DUMMY_NVS

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/


/*********************************************************************
 * LOCAL VARIABLES
 */

/* Buffer placed in RAM to hold bytes read from non-volatile storage. */
static uint32_t UDBuffer[SNV_BUFFER_SIZE] = {0};
static uint32_t (*ptrReadBuffer)[SNV_BUFFER_SIZE];
/*
 * Some devices have a minimum FLASH write size of 4-bytes (1 word). Trying
 * to write a non-multiple of 4 amount of data will fail. This array is
 * rounded up (to next multiple of 4) to meet this requirement. Refer to NVS
 * driver documentation for more details.
 */


/*********************************************************************
* PUBLIC FUNCTIONS
*/

/** set/get address register of readBuffer and turns the address of UDBuffer to the calling function  **/
extern void* snv_internal_setReadBuffer(uint32_t (*ptr_readBuffer)[])
{
    ptrReadBuffer = ptr_readBuffer;
    return (&UDBuffer);
}

extern void* snv_internal_getUDBuffer()
{
    return (&UDBuffer);
}

/***********************************************************************************************************
 * @fn      snv_internal_getInitSpeedMode
 *
 * @brief   get initial speed mode
 *
 * @param   Nil
 *
 * @return  speed_mode_init
******************************************************************************************************/
extern uint8_t snv_internal_getInitSpeedMode(void)
{
    return (*ptrReadBuffer)[SNV_BUFFER_SIZE - 4]; //[28]
}

extern uint8_t snv_internal_getInitDashUnit(void)
{
    return (*ptrReadBuffer)[SNV_BUFFER_SIZE - 3]; //[29]
}

extern uint8_t snv_internal_getInitLightMode(void)
{
    return (*ptrReadBuffer)[SNV_BUFFER_SIZE - 2]; //[30]
}
/*************************************************************
 *  Reset NVS memory
 */
extern void snv_internal_resetSNVdata()
{
    /**********************       Data organization    *************************
     *  UDBuffer size N = 32
     *  UDBuffer[0] stores the ADDataCounter, which is the cumulative count of number of integration completed
     *  UDBuffer[1] store the UDCounter, which is the number of UDTrigger
     *  UDBuffer[2] to UDbuffer[N - 7] stores the previous 10 Accumulated Distance traveled and Accumulated Energy Consumed data
     *  UDBuffer[N - 6] to UDbuffer[N - 5] stores a pair of codes for triggering reset
     *  UDBuffer[N - 6] has a value of 123456789
     *  UDbuffer[N - 5] has a value of 987654321
     *  UDBuffer[N - 4] stores the selected speed mode
     *  UDBuffer[N - 3] stores the selected dashboard unit
     *  UDBuffer[N - 2] stores the selected light mode
     *  UDBuffer[N - 1] is reserved and current not in use
     *
     **/

// ************* dummy data for resetting snv internal memory or testing assigning data to array storage **************
// **** NOTE:  osal_snv_read at every power on -> at first time initialization, the nvs should be zeros.

        /***** RESET NVS CASE: case 00 *****/
#ifndef DUMMY_NVS
    UDBuffer[26] = RESETCODE01;        // reset code 1
    UDBuffer[27] = RESETCODE02;        // reset code 2
#endif  // DUMMY_NVS

        /***** TEST CASE: case 01 (SETSIZE = 2) *****/
#ifdef DUMMY_NVS

        UDBuffer[0]  = 1439;      // ADDataCounter = number of integration completed (each integration contains N = data_analysis_points)
        UDBuffer[1]  = 51;        // UDCounter = number of UDTrigger.  UDCounter plus 1 when ADDataCounter = multiples of UDTrigger
        UDBuffer[2]  = 21221;     //1 Distance traveled
        UDBuffer[3]  = 29589;     //1 Energy Consumed
        UDBuffer[4]  = 23705;
        UDBuffer[5]  = 33246;     //2
        UDBuffer[6]  = 25602;
        UDBuffer[7]  = 36127;     //3
        UDBuffer[8]  = 27584;
        UDBuffer[9]  = 38703;     //4
        UDBuffer[10] = 28530;
        UDBuffer[11] = 40640;    //5
        UDBuffer[12] = 30549;
        UDBuffer[13] = 42959;    //6
        UDBuffer[14] = 32503;
        UDBuffer[15] = 46034;    //7
        UDBuffer[16] = 34485;
        UDBuffer[17] = 48962;    //8
        UDBuffer[18] = 36366;
        UDBuffer[19] = 51871;    //9
        UDBuffer[20] = 38138;
        UDBuffer[21] = 54542;    //10
        UDBuffer[22] = 39700;        //
        UDBuffer[23] = 56000;        //11
        UDBuffer[24] = 41500;        //
        UDBuffer[25] = 58000;        //12
        UDBuffer[26] = RESETCODE01;        // reset code 1
        UDBuffer[27] = RESETCODE02;        // reset code 2
        UDBuffer[28] = 0;        // speed mode {0 = Amble, 1 = Leisure, 2 = Sports}
        UDBuffer[29] = 0;        // dashboard unit {0 = kmph, 1 = mph}
        UDBuffer[30] = 2;        // light mode {0 = Off, 1 = On, 2 = Auto}
        UDBuffer[31] = 99;        // device uptime

#endif  // DUMMY_NVS

}
