/*
 * veml3235.c
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#include <stdint.h>
#include "veml3235.h"

#ifdef veml3235

static uint_least8_t targetAddress = VEML3235_ADDR;
uint8_t Bit3to0 = 0x03;
uint8_t Bit7to4 = 0x0C;
uint16_t Bit15to14 = 0xC000;
uint16_t LowByte = 0x00FF;
uint16_t HighByte = 0xFF00;

static ALSManager_t *veml3235_ALSManager;
uint8_t veml3235_i2cTransferStatus;
uint8_t writeBuffer[3];
uint8_t readBuffer[2];
static size_t writeBufferSize;
static size_t readBufferSize;

float resolution3235;
/*********************************************************************
 * @fn      veml3235_registerALS
 *
 * @brief   Receives the ALSI2C pointer (address)
 *
 * @param   None
 *
 * @return  None
 *********************************************************************/
void veml3235_registerALS( ALSManager_t *ALSI2C )
{
    veml3235_ALSManager = ALSI2C;
}

/*********************************************************************
 * Routine:     void veml3235_init()
 *
 * Description: initiate and Power On veml3235
 *
 * Arguments:   None
 *
 * Return:      None
*********************************************************************/
extern void veml3235_init(uint8_t ALS_Gain, uint16_t ALS_IT, uint8_t ALS_DG, uint8_t ALS_SD){
    /***** Command/Configuration register of veml3235 sensor *****/
    writeBufferSize = 3;
    readBufferSize = 0;
    /* write buffer structure (Low Byte, High Byte) */
    writeBuffer[0] = VEML3235_ALS_CONF;
    writeBuffer[1] = (ALS_IT << 4) | (ALS_SD & 0x01);                               // veml3235 Command Register low byte
    writeBuffer[2] = (ALS_SD & 0x80) | (ALS_DG << 5) | (ALS_Gain << 3) | 0x01;      // veml3235 Command Register high byte
    readBuffer[0] = 0;
    readBuffer[1] = 0;
    veml3235_i2cTransferStatus = veml3235_ALSManager -> ALS_transfer( targetAddress, &writeBuffer, writeBufferSize, &readBuffer, readBufferSize );
}


/*********************************************************************
 * Routine:     void veml3235_read()
 *
 * Description: Read Register On veml3235
 *
 * Arguments:   read_reg
 *
 * Return:      None
*********************************************************************/
uint16_t ALS_data = 0;
uint16_t White_data = 0;
uint16_t Int_trigger = 0;
uint16_t deviceID = 0;
extern void veml3235_read(uint8_t read_reg){
    // define read channel
    writeBufferSize = 1;
    readBufferSize = 2;
    writeBuffer[0] = read_reg;             // read ALS register =  0x04 or read white register = 0x05
    writeBuffer[1] = 0;
    readBuffer[0] = 0;
    readBuffer[1] = 0;
    // i2c transfer
    veml3235_i2cTransferStatus = veml3235_ALSManager -> ALS_transfer( targetAddress, &writeBuffer, writeBufferSize, &readBuffer, readBufferSize );
    switch (read_reg){
    case VEML3235_ALS:
        ALS_data = ((uint16_t) readBuffer[1] << 8) | readBuffer[0];
        break;
    case VEML3235_WHITE:
        White_data = ((uint16_t) readBuffer[1] << 8) | readBuffer[0];
        break;
    case VEML3235_ID:
        deviceID = (((uint16_t) readBuffer[1] << 8) | readBuffer[0]);
        break;
    default:
        ALS_data = 0xFFFF;
        White_data = 0xFFFF;
        deviceID = 0xFFFF;
        break;
    }
}

/*********************************************************************
 * Routine:     float veml3235_calculateLux()
 *
 * Description: calculate lux
 *
 * Arguments:   None
 *
 * Return:      lux
*********************************************************************/
float lux_values[2] = {0};
extern float veml3235_calculateLux(){
    // define calculation of Lux
    lux_values[0] = ALS_data * resolution3235;
    lux_values[1] = White_data * resolution3235;

    return lux_values[0];
}

/*********************************************************************
 * Routine:     float veml3235_resolution()
 *
 * Description: determines resolution based on gain and integration time
 *
 * Arguments:   Gain, Integration time
 *
 * Return:      resolution3235
*********************************************************************/
extern void veml3235_resolution(uint8_t ALSGain_select, uint16_t ALSIT_select, uint8_t ALSDG_select){
    float gain;
    uint8_t DG_value;
    if (ALSDG_select == DG0){
        DG_value = DG0_VALUE;
    }
    else if (ALSDG_select == DG1){
        DG_value = DG1_VALUE;
    }
    else {
        DG_value = 1;
    }
    switch (ALSGain_select){
    case Gain_1x:
        gain = 1;
        break;
    case Gain_2x:
        gain = 2;
        break;
    case Gain_4x:
        gain = 4;
        break;
    default:
        gain = 1;
        break;
    }
    switch (ALSIT_select){
    case IT_100:
        resolution3235 = RES_Gain_2x_IT_100 * 2 / gain / DG_value;
        break;
    case IT_200:
        resolution3235 = RES_Gain_2x_IT_200 * 2 / gain / DG_value;
        break;
    case IT_400:
        resolution3235 = RES_Gain_2x_IT_400 * 2 / gain / DG_value;
        break;
    case IT_800:
        resolution3235 = RES_Gain_2x_IT_800 * 2 / gain / DG_value;
        break;
    case IT_50:
        resolution3235 = RES_Gain_2x_IT_50 * 2 / gain / DG_value;
        break;
    default:
        resolution3235 = RES_Gain_2x_IT_100 * 2 / gain / DG_value;
        break;
    }
}

#endif

