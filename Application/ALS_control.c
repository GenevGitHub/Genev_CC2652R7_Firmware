/*
 * ALS_control.c
 *
 * @brief   This file contains the Code for controlling ambient light sensor
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
/* Driver configuration */
#include <ti_drivers_config.h>

#include "Application/ALS_control.h"
#include "Application/lights.h"

#include "UDHAL/UDHAL_PWM.h"

#include "Hardware/gGo_device_params.h"

/* Hardware Header files */
#ifdef veml6030
#include <Hardware/veml6030.h>
#endif

#ifdef veml3235
#include <Hardware/veml3235.h>
#endif

static uint16_t lux_threshold = LUXTHRESHOLD;

uint8_t I2C_transfer_count = 0;

#ifdef veml6030
/***** Configuration and Setting parameters *****/
static uint8_t ALSGain = Gain_2x;
static uint16_t ALSIT = IT_50;
static uint8_t Pers = PERS1;
static uint8_t IntR = INT_DIS;
static uint8_t PowerSavingMode = PSM1;
static uint8_t PowerSavingEnable = PSM_EN;
static uint8_t ALSSD = ALS_POWERON;
static uint16_t HighThresholdLux = 2000;   //default 0xFFFF
static uint16_t LowThresholdLux = 400;     // 0x1388 = 5000, 0x1770 = 6000, 0x1964 = 6500, 0x1B58 = 7000

#endif

#ifdef veml3235
/***** Configuration and Setting parameters *****/
static uint8_t ALSGain = Gain_2x;
static uint16_t ALSIT = IT_50;
static uint8_t ALSDG = 1;
static uint8_t ALSSD = ALS_POWERON;
#endif

float lux_result = 0xFFFF;
static uint8_t SampleSize = ALS_NUMSAMPLES;         // Min Sample Size is 1, Max Sample Size is 8
uint8_t newLightStatus = 0;

//static
uint8_t flagb = 0x00;
static uint8_t sampleBits = 0x00;
static uint8_t ii = 0;

/* Local function declaration */
void ALS_control_setLight();


/** @fun:   ALS_control_flagbRegister
 *
 *  @brief  return the pointer to flagb to the calling function
 *
 * **/
extern void* ALS_control_flagbRegister(void)
{
    return (&flagb);
}

/***************************************************
 * ALS_control_init()
 */
uint8_t ALS_control_init()
{
    /* Power up light sensors */
#ifdef veml6030
    /***** Enable register on veml6030 sensor  ****/
    veml6030_init(ALSGain, ALSIT, Pers, IntR, ALSSD);
    veml6030_resolution(ALSGain, ALSIT);
    if (IntR == INT_EN){
        veml6030_PSM(PowerSavingMode, PowerSavingEnable);         // use with Interrupt enable. Enable power saving mode and set power saving mode
        veml6030_setIntThreshold(VEML6030_ALS_WH, HighThresholdLux);  // use with Interrupt enable. set high threshold
        veml6030_setIntThreshold(VEML6030_ALS_WL, LowThresholdLux);     // use with Interrupt enable. set low threshold to 400 lux
    }
    I2C_transfer_count++;
#endif

#ifdef veml3235
    /***** Enable register on veml3235 sensor  ****/
    veml3235_init(ALSGain, ALSIT, ALSDG, ALSSD);
    veml3235_resolution(ALSGain, ALSIT, ALSDG);
    I2C_transfer_count++;
#endif

    /* Set up sampleBits for any Sample Size between 1 to 8 */
    for (uint8_t jj = 0; jj < SampleSize; jj++ ){
        sampleBits = sampleBits | (0x01 << jj);
    }

    return (sampleBits);
}

/***************************************************
 * ALS_control_calculateLux()
 */
uint8_t ALS_control_calculateLux()
{
#ifdef veml6030
    /***** Read channel data registers on veml6030 *****/
    veml6030_read(VEML6030_ALS);
    I2C_transfer_count++;
    veml6030_read(VEML6030_WHITE);
    I2C_transfer_count++;
    if (IntR == INT_EN){
        veml6030_read(VEML6030_ALS_INT);
        I2C_transfer_count++;
    }
    lux_result = veml6030_calculateLux();
//    veml6030_read(VEML6030_ID);
//    I2C_transfer_count++;
#endif

#ifdef veml3235
    /***** Read channel data registers on veml3235 *****/
    veml3235_read(VEML3235_ALS);
    I2C_transfer_count++;
    veml3235_read(VEML3235_WHITE);
    I2C_transfer_count++;
    lux_result = veml3235_calculateLux();
//    veml3235_read(VEML3235_ID);
//    I2C_transfer_count++;
#endif

    ALS_control_setLight();

    return (newLightStatus);
}

/***************************************************
 * ALS_control_setLight()
 */
void ALS_control_setLight()
{
    /* bit wise persistence approach   */
    // Advantage of bit-wise approach is No FOR loop is required and array is not needed for storing the sample result
    flagb = flagb & (~(0x01 << ii) & sampleBits);
    flagb = flagb | ((lux_result < lux_threshold) << ii);

    if (flagb >= sampleBits){
        if (!newLightStatus){
//            UDHAL_PWM_setHLDutyAndPeriod(99);   // set head-light on @ 99% pwmPeriod , LIGHT_PWM_DUTY = 75
            newLightStatus = LIGHT_STATUS_ON;                    // light = ON
        }
    }
    else if (flagb == 0){
        if (newLightStatus){
//            UDHAL_PWM_setHLDutyAndPeriod(0);    // set head-light off
            newLightStatus = LIGHT_STATUS_OFF;                    //light = OFF
        }
    }
    ii++;
    if (ii >= SampleSize){
        ii = 0;
    }
}

/******************************************************************
 * ALS_control_getIntR
 * Call this function to get the active Interrupt configuration
 *
 *  */
#ifdef veml6030
extern uint8_t ALS_control_getIntR(){
    return (IntR);
}
#endif //veml6030

