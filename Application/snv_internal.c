/*
 *  snv_internal.c
 *  Brief:  When using and working with BLE functions, NVS must be accessed using
 *  OSAL_SNV library.  OSAL SNV operations are defined through ICALL
 *
 *  Created on: 30 Apr 2024
 *      Author: Chee
 */

/*********************************************************************
 * INCLUDES
 */
#include "snv_internal.h"

#include "Hardware/gGo_device_params.h"
#include "Application/lights.h"
#include "Application/brake_and_throttle.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

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
static uint32_t UDBuffer[SNV_BUFFER_SIZE] = {0};    // UDBuffer contains the reset data
static uint32_t (*ptrReadBuffer)[SNV_BUFFER_SIZE];  // snvBuffer contains the data stored in memory
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
extern void* snv_internal_setReadBuffer(uint32_t (*ptr_snvBuffer)[])
{
    ptrReadBuffer = ptr_snvBuffer;
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
     *  UDBuffer[N - 6] has a value of RESETCODE01
     *  UDbuffer[N - 5] has a value of RESETCODE02
     *  UDBuffer[N - 4] stores the selected speed mode
     *  UDBuffer[N - 3] stores the selected dashboard unit
     *  UDBuffer[N - 2] stores the selected light mode
     *  UDBuffer[N - 1] is reserved and current not in use
     *
     **/

// ************* Data for resetting snv internal memory or testing assigning data to array storage **************
// **** NOTE:  osal_snv_read at every power on -> at first time initialization, the nvs should be zeros.

    /***** ZERO NVS CASE: case 00 *****/
#ifdef ZERO_NVS    // ZERO_NVS defined in simple peripheral.h
UDBuffer[26] = RESETCODE01;        // reset code 1
UDBuffer[27] = RESETCODE02;        // reset code 2
UDBuffer[28] = BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE;                  // speed mode {0 = Amble, 1 = Leisure, 2 = Sports}
UDBuffer[30] = LIGHT_MODE_AUTO;                  // light mode {0 = Off, 1 = On, 2 = Auto}
#endif  // ZERO_NVS

    /***** TEST CASE: case 01 (SETSIZE = 2) *****/
#ifdef DUMMY_NVS    // DUMMY_NVS defined in simple peripheral.h

        UDBuffer[0]  = 1439;      // ADDataCounter = number of integration completed (each integration contains N = data_analysis_points)
        UDBuffer[1]  = 47;        // UDCounter = number of UDTrigger.  UDCounter % UDARRAYSIZE must correspond to the location of the very last saved dataset
                                    // in this dummy data, 47 % UDARRAYSIZE (12) = 11.  Therefore, set 11 is the location of the previous saved dataset
        UDBuffer[2]  = 21221;     // set 0 Distance traveled
        UDBuffer[3]  = 29589;     // set 0 Energy Consumed
        UDBuffer[4]  = 23705;     // set 1 Distance traveled
        UDBuffer[5]  = 33246;
        UDBuffer[6]  = 25602;     // set 2 Distance traveled
        UDBuffer[7]  = 36127;
        UDBuffer[8]  = 27584;     // set 3 Distance traveled
        UDBuffer[9]  = 38703;
        UDBuffer[10] = 28530;     // set 4 Distance traveled
        UDBuffer[11] = 40640;
        UDBuffer[12] = 30549;     // set 5 Distance traveled
        UDBuffer[13] = 42959;
        UDBuffer[14] = 32503;     // set 6 Distance traveled
        UDBuffer[15] = 46034;
        UDBuffer[16] = 34485;     // set 7 Distance traveled
        UDBuffer[17] = 48962;
        UDBuffer[18] = 36366;     // set 8 Distance traveled
        UDBuffer[19] = 51871;
        UDBuffer[20] = 38138;     // set 9 Distance traveled
        UDBuffer[21] = 54542;
        UDBuffer[22] = 39700;     // set 10 Distance traveled
        UDBuffer[23] = 56000;
        UDBuffer[24] = 41500;     // set 11 Distance traveled
        UDBuffer[25] = 58000;        //12

        UDBuffer[26] = RESETCODE01;        // reset code 1
        UDBuffer[27] = RESETCODE02;        // reset code 2

        UDBuffer[28] = 0;        // speed mode {0 = Amble, 1 = Leisure, 2 = Sports}
        UDBuffer[29] = 0;        // dashboard unit {0 = kmph, 1 = mph}
        UDBuffer[30] = 2;        // light mode {0 = Off, 1 = On, 2 = Auto}
        UDBuffer[31] = 99;        // device uptime

#endif  // DUMMY_NVS

}
