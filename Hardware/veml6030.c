/*
 * veml6030.c
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */
#include <stdint.h>
#include "Hardware/veml6030.h"

float resolution;

#ifdef veml6030

static uint_least8_t targetAddress = VEML6030_ADDRL;
static uint8_t Bit3to0 = 0x03;
static uint8_t Bit7to4 = 0x0C;
static uint16_t Bit15to14 = 0xC000;
static uint16_t LowByte = 0x00FF;
static uint16_t HighByte = 0xFF00;

static ALSManager_t *veml6030_ALSManager;
uint8_t veml6030_i2cTransferStatus;
static uint8_t writeBuffer[3];
static uint8_t readBuffer[2];
static size_t writeBufferSize;
static size_t readBufferSize;

/*********************************************************************
 * @fn      veml6030_registerALS
 *
 * @brief   Receives the ALSI2C pointer (address)
 *
 * @param   None
 *
 * @return  None
 *********************************************************************/
void veml6030_registerALS( ALSManager_t *ALSI2C )
{
    veml6030_ALSManager = ALSI2C;
}

/*********************************************************************
 * Routine:     void veml6030_init()
 *
 * Description: initiate and Power On veml6030
 *
 * Arguments:   None
 *
 * Return:      None
*********************************************************************/
extern void veml6030_init(uint8_t ALS_Gain, uint16_t ALS_IT, uint8_t Persistance, uint8_t Interrupt, uint8_t ALS_SD){
    /***** Command/Configuration register of veml6030 sensor *****/
    writeBufferSize = 3;
    readBufferSize = 0;
    /* write buffer structure (Low Byte, High Byte) */
    writeBuffer[0] = VEML6030_ALS_CONF;
    writeBuffer[1] = ((ALS_IT & Bit3to0) << 6) | (Persistance << 4) | (Interrupt << 1) | ALS_SD;    // veml6030 Command Register low byte
    writeBuffer[2] = (ALS_Gain << 3) | ((ALS_IT & Bit7to4) >> 2);                                  // veml6030 Command Register high byte
    readBuffer[0] = 0;
    readBuffer[1] = 0;
    veml6030_i2cTransferStatus = veml6030_ALSManager -> ALS_transfer( targetAddress, &writeBuffer, writeBufferSize, &readBuffer, readBufferSize );
}

/*********************************************************************
 * Routine:     void veml6030_PSM()
 *
 * Description: configure power saving on veml6030
 *
 * Arguments:   HighLow, PSM_mode, PSM_EN
 *
 * Return:      None
*********************************************************************/
extern void veml6030_PSM(uint8_t PSM_mode, uint8_t PSM_enable){
    /***** Command/Configuration register of veml6030 sensor *****/
    writeBufferSize = 3;
    readBufferSize = 0;
    /* write buffer structure (Low Byte, High Byte) */
    writeBuffer[0] = VEML6030_PWR_SAVE;                         // veml6030 Power Saving Register
    writeBuffer[1] = (PSM_mode << 1) | (PSM_enable);                                  // veml6030 Command Register high byte
    writeBuffer[2] = 0x00;
    readBuffer[0] = 0;
    readBuffer[1] = 0;
    veml6030_i2cTransferStatus = veml6030_ALSManager -> ALS_transfer( targetAddress, &writeBuffer, writeBufferSize, &readBuffer, readBufferSize );
}

/*********************************************************************
 * Routine:     void veml6030_setIntThreshold()
 *
 * Description: configure interrupt high threshold and low threshold on veml6030
 *
 * Arguments:   HighLow, ThresholdLSB, ThresholdMSB
 *
 * Return:      None
*********************************************************************/
uint16_t Hightreshold;
uint16_t Lowtreshold;
extern void veml6030_setIntThreshold(uint8_t HighLow, uint16_t threshold_lux){
    /***** Configuration interrupt high and low thresholds of veml6030 sensor *****/
    uint16_t threshold_value;
    /* Check saturation */
    if (threshold_lux / resolution < 0xFFFF){
        threshold_value = threshold_lux / resolution;
    }
    else {
        threshold_value = 0xFFFF;
    }
    if (HighLow == VEML6030_ALS_WH){
        Hightreshold = threshold_value;
    }
    else if (HighLow == VEML6030_ALS_WL){
        Lowtreshold = threshold_value;
    }

    writeBufferSize = 3;
    readBufferSize = 0;
    /* write buffer structure (Low Byte, High Byte) */
    writeBuffer[0] = HighLow;                         // veml6030 Power Saving Register
    writeBuffer[1] = (threshold_value & 0xFF);           // the result is 8 bits       //threshold_value & LowByte;                                  // veml6030 Command Register high byte
    writeBuffer[2] = ((threshold_value >> 8) & 0xFF);    // the result is 8 bits       //(threshold_value & HighByte) >> 8;
    readBuffer[0] = 0;
    readBuffer[1] = 0;
    veml6030_i2cTransferStatus = veml6030_ALSManager -> ALS_transfer( targetAddress, &writeBuffer, writeBufferSize, &readBuffer, readBufferSize );
}

/*********************************************************************
 * Routine:     void veml6030_read()
 *
 * Description: Read Register On veml6030
 *
 * Arguments:   read_reg
 *
 * Return:      None
*********************************************************************/
uint16_t ALS_data = 0;
uint16_t White_data = 0;
uint16_t Int_trigger = 0;
uint16_t deviceID = 0;
extern void veml6030_read(uint8_t read_reg){
    // define read channel
    writeBufferSize = 1;
    readBufferSize = 2;
    writeBuffer[0] = read_reg;             // read ALS register =  0x04 or read white register = 0x05
    writeBuffer[1] = 0;
    readBuffer[0] = 0;
    readBuffer[1] = 0;
    // i2c transfer
    veml6030_i2cTransferStatus = veml6030_ALSManager -> ALS_transfer( targetAddress, &writeBuffer, writeBufferSize, &readBuffer, readBufferSize );
    switch (read_reg){
    case VEML6030_ALS:
        ALS_data = ((uint16_t) readBuffer[1] << 8) | readBuffer[0];
        break;
    case VEML6030_WHITE:
        White_data = ((uint16_t) readBuffer[1] << 8) | readBuffer[0];
        break;
    case VEML6030_ALS_INT:
        Int_trigger = (((uint16_t) readBuffer[1] << 8) | readBuffer[0]) & 0xC000;   // keep only bit 15 and bit 14, else zeros.
        break;
    case VEML6030_ID:
        deviceID = (((uint16_t) readBuffer[1] << 8) | readBuffer[0]);
        break;
    default:

        break;
    }
}
/*********************************************************************
 * Routine:     float veml6030_calculateLux()
 *
 * Description: calculate lux
 *
 * Arguments:   None
 *
 * Return:      lux
*********************************************************************/
float lux_uncorrect[2] = {0};
float lux_correct[2] = {0};
extern float veml6030_calculateLux(){
    // define calculation of Lux
    lux_uncorrect[0] = ALS_data * resolution;
    lux_uncorrect[1] = White_data * resolution;

    if (lux_uncorrect[0] > 1200){
        lux_correct[0] = COEFFICIENTA * lux_uncorrect[0] * lux_uncorrect[0] * lux_uncorrect[0] * lux_uncorrect[0] +
                COEFFICIENTB * lux_uncorrect[0] * lux_uncorrect[0] * lux_uncorrect[0] + COEFFICIENTC * lux_uncorrect[0] * lux_uncorrect[0] +
                COEFFICIENTD * lux_uncorrect[0];
    }
    else {
        lux_correct[0] = lux_uncorrect[0];
    }
    if (lux_uncorrect[1] > 1200){
        lux_correct[1] = COEFFICIENTA * lux_uncorrect[1] * lux_uncorrect[1] * lux_uncorrect[1] * lux_uncorrect[1] +
                COEFFICIENTB * lux_uncorrect[1] * lux_uncorrect[1] * lux_uncorrect[1] + COEFFICIENTC * lux_uncorrect[1] * lux_uncorrect[1] +
                COEFFICIENTD * lux_uncorrect[1];
    }
    else {
        lux_correct[1] = lux_uncorrect[1];
    }
    return lux_correct[0];
}



/*********************************************************************
 * Routine:     float veml6030_resolution()
 *
 * Description: determines resolution based on gain and integration time
 *
 * Arguments:   Gain, Integration time
 *
 * Return:      resolution
*********************************************************************/
extern void veml6030_resolution(uint8_t ALSGain_select, uint16_t ALSIT_select){
    float gain;
    switch (ALSGain_select){
    case Gain_1x:
        gain = 1;
        break;
    case Gain_2x:
        gain = 2;
        break;
    case Gain_0_125x:
        gain = 0.125;
        break;
    case Gain_0_25x:
        gain = 0.25;
        break;
    default:
        gain = 1;
        break;
    }
    switch (ALSIT_select){
    case IT_100:
        resolution = RES_Gain_2x_IT_100 * 2 / gain;
        break;
    case IT_200:
        resolution = RES_Gain_2x_IT_200 * 2 / gain;
        break;
    case IT_400:
        resolution = RES_Gain_2x_IT_400 * 2 / gain;
        break;
    case IT_800:
        resolution = RES_Gain_2x_IT_800 * 2 / gain;
        break;
    case IT_50:
        resolution = RES_Gain_2x_IT_50 * 2 / gain;
        break;
    case IT_25:
        resolution = RES_Gain_2x_IT_25 * 2 / gain;
        break;
    default:
        resolution = RES_Gain_2x_IT_100 * 2 / gain;
        break;
    }
}
/*********************************************************************
 * Routine:     float veml6030_shutDown()
 *
 * Description: Shut down veml6030
 *
 * Arguments:   Nil
 *
 * Return:      Nil
*********************************************************************/
extern void veml6030_shutDown( void ){
    /***** Command/Configuration register of veml6030 sensor *****/
    writeBufferSize = 3;
    readBufferSize = 0;
    /* write buffer structure (Low Byte, High Byte) */
    writeBuffer[0] = VEML6030_ALS_CONF;
    writeBuffer[0] = ALS_SHUTDOWN;                      // veml6030 Command Register shut down (0x0001) low byte
    writeBuffer[1] = 0x00;                              // veml6030 Command Register shut down (0x0001) high byte
    readBuffer[0] = 0;
    readBuffer[1] = 0;
    veml6030_i2cTransferStatus = veml6030_ALSManager -> ALS_transfer( targetAddress, &writeBuffer, writeBufferSize, &readBuffer, readBufferSize );
}



#endif
