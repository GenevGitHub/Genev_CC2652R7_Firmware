/*
 * buzzer.c
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */
/*********************************************************************
* INCLUDES
*/
#include <stdlib.h>
/* Driver Header files */
#include "Application/buzzer.h"
#include "Application/general_purpose_timer.h"
#include "Application/led_display.h"

#include "UDHAL/UDHAL_PWM.h"

/*********************************************************************
* LOCAL VARIABLES
*/
uint8_t  *ptr_buzzer_errorPriority;

/*********************************************************************
* LOCAL FUNCTIONS
*/
/*********************************************************************
 * @fn      buzzer_Init
 *
 * @brief   It is used to initialize the library
 *
 * @param   none
 *
 * @return  none
 */
void buzzer_init()
{
    ptr_buzzer_errorPriority = led_display_errorPriorityRegister();
}

/***********************************************************************************************************
 * @fn      buzzer_buzzerStatusRegister
 *
 * @brief   register and assign pointer of buzzerStatus to ptr_buzzerStatus
 *
 * @param   ptrbuzzerStatus
 *
 * @return  Nil
******************************************************************************************************/
static uint8_t *ptr_buzzer_buzzerStatus;

extern void buzzer_buzzerStatusRegister(uint8_t *ptrbuzzerStatus)
{
    ptr_buzzer_buzzerStatus = ptrbuzzerStatus;
}

/*********************************************************************
 * @fn      buzzer_buttonHandler
 *
 * @brief   buzzer execution when single button is pressed
 *
 * @param   buttonStatus
 *
 * @return  none
 */
uint8_t buzzerStart = 0;
void buzzer_buttonHandler(uint8_t buttonStatus)
{
    /* Start buzzer if Buzzer is OFF */
    // if buttonStatus is not 0, sound the buzzer
    if ((buttonStatus != 0) && (buzzerStart == 0)){
        UDHAL_PWM_setBUZDutyAndPeriod(BUZZER_MPB_DUTYPERCENT, BUZZER_MPB_FREQUENCY);
        buzzerStart = 2;                   // buzzerStart = 2 means buzzer is activated due to falling edge of single button press
    }
    /* Stop buzzer if buzzer is ON */
    if (buttonStatus == 0){
        if (buzzerStart >= 2){
            UDHAL_PWM_setBUZDutyAndPeriod(0, BUZZER_MPB_FREQUENCY);   // set duty to 0
            buzzerStart = 0;
        }
    }

}


/***************************************************************************************************
 * @fn      buzzer_ErrorHandler
 *
 * @brief   Call to manage buzzer error handler when error exists
 *
 * @param   Nil
 *
 * @return  Nil
******************************************************************************************************/
bool buzzer_buzzerOn = 0;
uint16_t buzzer_buzzerCounter = 0;
uint16_t buzzer_buzzerTrigger = BUZZER_WARNING_BEEP_PERIOD / GPT_TIME;

extern void buzzer_ErrorHandler( void )
{

#ifdef CC2652R7_LAUNCHXL  //
    if (!(*ptr_buzzer_buzzerStatus) && !(buzzer_buzzerCounter) && (*ptr_buzzer_errorPriority <= BATTERY_CRITICALLY_LOW_WARNING) && !(buzzer_buzzerOn))
    {
//        buzzer_errorHandler(BUZZER_WARNING_DUTYPERCENT, BUZZER_WARNING_FREQUENCY);
        UDHAL_PWM_setBUZDutyAndPeriod(BUZZER_WARNING_DUTYPERCENT, BUZZER_WARNING_FREQUENCY);

        buzzer_buzzerOn = 1;
    }
    else if (buzzer_buzzerOn == 1)
    {
//        buzzer_errorHandler(0, BUZZER_WARNING_FREQUENCY);
        UDHAL_PWM_setBUZDutyAndPeriod(0, BUZZER_WARNING_FREQUENCY);

        buzzer_buzzerOn = 0;
    }

    buzzer_buzzerCounter++;

    if (buzzer_buzzerCounter >= buzzer_buzzerTrigger)
    {
        buzzer_buzzerCounter = 0;
    }
#endif  // CC2652R7_LAUNCHXL

}
