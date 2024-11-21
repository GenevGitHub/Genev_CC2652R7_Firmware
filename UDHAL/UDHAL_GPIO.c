/*
 * UDHAL_GPIO.c
 *
 *  Created on: 4 May 2024
 *      Author: Chee
 */
/*********************************************************************
 * INCLUDES
 */
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <ti/drivers/GPIO.h>
//#include <ti/drivers/Board.h>
#include "ti_drivers_config.h"

#include "Application/ALS_control.h"
#include "Application/multi_purpose_button.h"
#include "Hardware/veml6030.h"
#include "UDHAL/UDHAL_GPIO.h"
/*********************************************************************
 * LOCAL VARIABLES
 */
/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * Marco
 */
/*********************************************************************
 * @fn      UDHAL_GPIO_init
 *
 * @brief   It is used to initialize the used registers in the motor control registers
 *
 * @param   None
 *
 * @return  None
 */
extern void UDHAL_GPIO_init()
{
    GPIO_init();

}
/*********************************************************************
 * @fn      UDHAL_GPIO_params_init
 *
 * @brief   It is used to initialize the used registers in the motor control registers
 *
 * @param   None
 *
 * @return  None
 */
//uint8_t pullUp = 0xFF;

void UDHAL_GPIO_params_init()
{
    /* Configuration of PIN for Multi-Purpose Button  */
    GPIO_setConfig(CONFIG_GPIO_MPB,  GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_BOTH_EDGES);    //
    GPIO_setCallback(CONFIG_GPIO_MPB, &UDHAL_GPIO_InterruptFxn);                      // GPIO Callback -> pin -> interrupt function
    GPIO_enableInt(CONFIG_GPIO_MPB);

#ifdef veml6030
    // ALS_control_getIntR();
    if (ALS_control_getIntR() == INT_EN){
        /* Use for veml6030 interrupt pin  */
        GPIO_setConfig(CONFIG_GPIO_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
        GPIO_setCallback(CONFIG_GPIO_0, &UDHAL_GPIO_InterruptFxn);
        GPIO_enableInt(CONFIG_GPIO_0);
    }
#endif

}

/*********************************************************************
 * @fn      UDHAL_GPIO_write
 *
 * @brief   To write the logic Level of the GPIO
 *
 * @param   index: The pin number
 *          value: Either High or Low
 *
 * @return  None
 */
uint_fast8_t UDHAL_GPIO_read(uint_least8_t index)
{
     return GPIO_read(index);
}

/*********************************************************************
 * @fn      UDHAL_GPIO_write
 *
 * @brief   To write the logic Level of the GPIO
 *
 * @param   index: The pin number
 *          value: Either High or Low
 *
 * @return  None
 */
void UDHAL_GPIO_write(uint_least8_t index, uint8_t value)
{
    GPIO_write(index, value);
}

/*********************************************************************
 * @fn      UDHAL_GPIO_InterruptFxn
 *
 * @brief   Whenever the button is pressed, it goes here. You may change the function name if your device has strict naming on ISR.
 *          However, two action must be performed.
 *          1. Add the function singleButton_processButtonEvt(uint8_t logicLevel) here/
 *          2. Read the GPIO logic level and pass it to this function
 *
 * @param   index
 *
 * @return  none
 */
#ifdef veml6030
uint8_t veml6030_callback_count = 0;
uint8_t interrupt_status = 0;
uint8_t gpioReadInt = 1;
#endif

void UDHAL_GPIO_InterruptFxn(uint_least8_t index)
{
   uint_fast8_t logicLevel;
   switch(index)
   {
   case CONFIG_GPIO_MPB:
   {
       //uint_fast8_t
       logicLevel = UDHAL_GPIO_read(CONFIG_GPIO_MPB);
       mpb_processButtonEvt(logicLevel);
       break;
   }
#ifdef veml6030
   case CONFIG_GPIO_0:
   {
       veml6030_callback_count++;
       gpioReadInt = GPIO_read(CONFIG_GPIO_0);
       interrupt_status = 1;
       break;
   }
#endif
   default:

       break;
   }
}



