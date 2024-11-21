/*
 * UDHAL.c
 *
 *  Created on: 4 May 2024
 *      Author: Chee
 */
/*********************************************************************
 * INCLUDES
 */
#include "UDHAL/UDHAL.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static sysFatalError_t UDHAL_sysFatalError;


extern void* UDHAL_sysFatalErrorRegister(){
    return (&UDHAL_sysFatalError);
}

/*********************************************************************
 * GLOBAL FUNCTION
 */
uint8_t UDHAL_UARTOpenStatus;

uint8_t UDHAL_init()
{
//    uint32_t bootSource = 0xFF;
//    bootSource = SysCtrlResetSourceGet();


    uint8_t UDHAL_ADC_throttleOpenStatus;
    uint8_t UDHAL_ADC_brakeOpenStatus;
    uint8_t UDHAL_I2COpenStatus;
    uint8_t UDHAL_PWMOpenStatus;
//    uint8_t UDHAL_UARTOpenStatus;

    /* Initiate GPIO */
    UDHAL_GPIO_init();
    UDHAL_GPIO_params_init();

    /* Create UART for usage */
    UDHAL_UART_init();
    UDHAL_UARTOpenStatus = UDHAL_UART_params_init();
    if (UDHAL_UARTOpenStatus)
    {
        UDHAL_sysFatalError.UARTfailure = UDHAL_SUCCESS;
    }
    else
    {
        UDHAL_sysFatalError.UARTfailure = UDHAL_FAILURE;

    }

    UDHAL_ADC_init();
    UDHAL_ADC_throttleOpenStatus = UDHAL_ADC_throttleOpen();
    UDHAL_ADC_brakeOpenStatus = UDHAL_ADC_brakeOpen();
    if ((UDHAL_ADC_throttleOpenStatus) && (UDHAL_ADC_brakeOpenStatus))
    {
        UDHAL_sysFatalError.ADCfailure = UDHAL_SUCCESS;
    }
    else
    {
        UDHAL_sysFatalError.ADCfailure = UDHAL_FAILURE;
    }

    /* Create I2C for usage */
    UDHAL_I2C_init();
    UDHAL_I2COpenStatus = UDHAL_I2C_paramInit();
    if (UDHAL_I2COpenStatus)
    {
        UDHAL_sysFatalError.I2Cfailure = UDHAL_SUCCESS;  //
    }
    else
    {
        UDHAL_sysFatalError.I2Cfailure = UDHAL_FAILURE;
    }

    /* Create PWM for usage */
    UDHAL_PWM_init();
    UDHAL_PWMOpenStatus = UDHAL_PWM_paramInit();
    if(UDHAL_PWMOpenStatus)
    {
        UDHAL_sysFatalError.PWMfailure = UDHAL_SUCCESS;
    }
    else
    {
        UDHAL_sysFatalError.PWMfailure = UDHAL_FAILURE;
    }

    // we might choose to initiate Timer2 and Timer3 here for simplicity,
    // but initiating timer2 in STM32MCP_init() and MPB_init() seems
    // to be more logical.

    /* Create clock for STM32 heartbeat */
    UDHAL_TIMER2_init();
    UDHAL_TIMER2_params_init();

    /* Create clock for multi-purpose button interrupt */
    UDHAL_TIMER3_init();
    UDHAL_TIMER3_params_init();

    return (UDHAL_I2COpenStatus);
}



/*Peripheral De-Initialization*/
void UDHAL_Boot()
{
    /*Initialize the timer for Boot*/
    UDHAL_TIMER3_init();              // single button source --> dependency --> GPIO must be initialized
    UDHAL_TIMER3_params_init();       // single button header --> dependency --> GPIO must be initialized
}
