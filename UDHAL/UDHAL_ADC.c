/*
 * UDHAL_ADC.c
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#include "UDHAL_ADC.h"

ADC_Handle adc_throttle;
ADC_Params params_throttle;
uint8_t adc_throttle_status = 0;

ADC_Handle adc_brake;
ADC_Params params_brake;
uint8_t adc_brake_status = 0;

/*********************************************************************
 * Routine:     UDHAL_ADC_init()
 *
 * Description: Initialization ADC
 *
 * Arguments:   None
 *
 * Return:      None
*********************************************************************/
extern void UDHAL_ADC_init(){
    ADC_init();
}

/*********************************************************************
 * Routine:     UDHAL_ADC_throttleOpen()
 *
 * Description: Configures and Opens Throttle ADC GPIO pin
 *
 * Arguments:   None
 *
 * Return:      None
*********************************************************************/
extern uint8_t UDHAL_ADC_throttleOpen(){
    ADC_Params_init(&params_throttle);
    adc_throttle = ADC_open(CONFIG_ADC_THR, &params_throttle);
    if (adc_throttle == NULL) {
        adc_throttle_status = 0;    // failed to open
    }
    else {
        adc_throttle_status = 1;
    }
    return (adc_throttle_status);
}
/*********************************************************************
 * Routine:     UDHAL_ADC_brakeOpen()
 *
 * Description: Configures and Opens Brake ADC GPIO pin
 *
 * Arguments:   None
 *
 * Return:      None
*********************************************************************/
extern uint8_t UDHAL_ADC_brakeOpen(){
    ADC_Params_init(&params_brake);
    adc_brake = ADC_open(CONFIG_ADC_BRK, &params_brake);
    if (adc_brake == NULL) {
        adc_brake_status = 0;    // failed to open
    }
    else {
        adc_brake_status = 1;
    }
    return (adc_brake_status);
}

/*********************************************************************
 * Routine:     UDHAL_ADC_Convert()
 *
 * Description: Configures and Opens Throttle ADC GPIO pin
 *
 * Arguments:   None
 *
 * Return:      None
*********************************************************************/

uint16_t (*ptr_adcValues)[2];
uint8_t adc_convert_status = 0;

extern void UDHAL_ADC_Convert(){

    if (ADC_convert(adc_throttle, ((*ptr_adcValues)+0)) == ADC_STATUS_SUCCESS) {
        adc_convert_status = 1;
    }
    else {  /* ADC convert failed - ERROR */
        adc_convert_status = 0;
        (*ptr_adcValues)[0] = 0;
    }
    if (ADC_convert(adc_brake, ((*ptr_adcValues)+1)) == ADC_STATUS_SUCCESS) {
        adc_convert_status = 1;
    }
    else { /* ADC convert failed - ERROR */
        adc_convert_status = 0;
        (*ptr_adcValues)[1] = 0;
    }
}

/*********************************************************************
 * Routine:     UDHAL_ADC_close()
 *
 * Description: closes ADC GPIO pin of adcHanlde
 *
 * Arguments:   adcHandle
 *
 * Return:      None
*********************************************************************/
extern void UDHAL_ADC_close(ADC_Handle adcHandle){
    ADC_close(adcHandle);
}


/************************************************************
 * Passes the pointer to the array ADCValues[]
 ************************************************************/
extern void UDHAL_ADC_ptrADCValues(uint16_t (*ptrADCvalues)[]){
    ptr_adcValues = ptrADCvalues;
}

