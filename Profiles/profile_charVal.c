/*
 * profile_charVal.c
 *
 *  Created on: 30 May 2024
 *      Author: Chee
 */

/*********************************************************************
 * INCLUDES
 */

#include "Profiles/profile_charVal.h"


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define BITSPERBYTE                 8

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/
profileCharVal_t profileCharVal = {0};


/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
* PUBLIC FUNCTIONS
*/

extern void* profile_charVal_profileCharValRegister()
{
    return (&profileCharVal);
}


extern void profile_charVal_init()
{
    /******************************************************
     *  get pointers / registers
     ******************************************************/
    profileCharVal.ptr_dash_charVal = Dashboard_CharValRegister();
    profileCharVal.ptr_cont_charVal = Controller_CharValRegister();
    profileCharVal.ptr_batt_charVal = Battery_CharValRegister();
}
/*********************************************************************
 * @fn      profile_setCharVal
 *
 * @brief   this function converts and places the integer "payload" (max length 4 bytes i.e. uint32_t)
 *          into an array of 1 byte elements.  The array can have up to max 4 element of 1 byte each.
 *
 * @params  ptr_CharVal, payloadLength (1 to 4), payload (max 4 bytes)
 *
 * @return  Nil
 */
extern void profile_setCharVal(uint8 *ptr_CharVal, uint8_t payloadLength, uint32_t payload){

    for (uint8_t ii = 0; ii < payloadLength; ii++)
    {
        uint8_t bb = ii * BITSPERBYTE;
        (ptr_CharVal)[ii] = ((payload >> bb) & 0xFF);
    }
}

