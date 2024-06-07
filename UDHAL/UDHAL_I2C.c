/*
 * UDHAL_I2C.c
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */
#include "UDHAL_I2C.h"
#include "Application/ALS_control.h"
#include "Application/led_display.h"

#ifdef veml6030
#include <Hardware/veml6030.h>
#endif

#ifdef veml3235
#include <Hardware/veml3235.h>
#endif

I2C_Handle      i2cHandle = NULL;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction = {0};
uint8_t         i2cOpenStatus = 0;
uint8_t         i2cTransferStatus = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void UDHAL_I2C_open();
static uint8_t UDHAL_I2C_transfer(uint_least8_t slaveAddress, void *writeBuffer, size_t writeSize, void *readBuffer, size_t readSize);
static void UDHAL_I2C_close();
static void UDHAL_I2C_ErrorHandler(I2C_Transaction *transaction);

/*********************************************************************
 * Marco
 */
static ALSManager_t ALS_I2C =
{
    UDHAL_I2C_open,
    UDHAL_I2C_transfer,
    UDHAL_I2C_close
};
/*********************************************************************
 * Marco
 */
static led_display_ledDisplayManager_t ledDisplay_I2C =
{
    UDHAL_I2C_open,
    UDHAL_I2C_transfer,
    UDHAL_I2C_close
};
/*********************************************************************
 * Routine:     void UDHAL_I2C_init()
 *
 * Description: Initialization I2C
 *
 * Arguments:   None
 *
 * Return:      None
*********************************************************************/
void UDHAL_I2C_init()
{
    I2C_init();

#ifdef veml6030
    veml6030_registerALS(&ALS_I2C);
#endif

#ifdef veml3235
    veml3235_registerALS(&ALS_I2C);
#endif

    led_display_registerLedDisplay(&ledDisplay_I2C);     // LED Display
}
/*********************************************************************
 * Routine:     void UDHAL_I2C1_paramInit()
 *
 * Description: Set I2C parameters
 *
 * Arguments:   None
 *
 * Return:      None
*********************************************************************/
uint8_t UDHAL_I2C_paramInit()
{
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;                        // the amount of data transferred per second
    if (i2cHandle == NULL)
    {
        UDHAL_I2C_open();
    }
    return i2cOpenStatus;
}
/*********************************************************************
 * Routine:     void UDHAL_I2C1_open()
 *
 * Description: open I2C communication
 *
 * Arguments:   None
 *
 * Return:      None
*********************************************************************/
void UDHAL_I2C_open()
{
    i2cHandle = I2C_open(CONFIG_I2C, &i2cParams);
    if (i2cHandle == NULL){
        i2cOpenStatus = 0;
    }
    else {
        i2cOpenStatus = 1;
    }
}
/*********************************************************************
 * @fn:     UDHAL_I2C_getI2CStatus
 *
 * @brief:  return I2C open status
 *
 * Arguments:   None
 *
 * Return:      None
*********************************************************************/
I2C_Handle UDHAL_I2C_getI2CStatus()
{
    return (i2cHandle);
}

uint8_t UDHAL_I2C_getI2COpenStatus() {
    return (i2cOpenStatus);
}
/*********************************************************************
 * @fn:     UDHAL_I2C_close
 *
 * @brief: close I2C communication
 *
 * Arguments:   None
 *
 * Return:      None
*********************************************************************/
void UDHAL_I2C_close()
{
    I2C_close(i2cHandle);
    i2cHandle = NULL;
}
/*********************************************************************
 * Routine:     uint8_t UDHAL_I2C_transfer()
 *
 * Description: I2C communication transfer
 *
 * Arguments:   None
 *
 * Return:      None
*********************************************************************/
uint8_t UDHAL_I2C_transfer(uint_least8_t slaveAddress, void *writeBuffer,
                         size_t writeBufferSize, void *readBuffer, size_t readBufferSize)
{
    if (i2cHandle != NULL)
    {
        i2cTransaction.targetAddress = slaveAddress;
        i2cTransaction.writeBuf   = writeBuffer;
        i2cTransaction.writeCount = writeBufferSize;
        i2cTransaction.readBuf    = readBuffer;
        i2cTransaction.readCount  = readBufferSize;
        if (I2C_transfer(i2cHandle, &i2cTransaction)){
            // transfer successful
            i2cTransferStatus = 1;
        }
        else {
            i2cTransferStatus = 0;
            UDHAL_I2C_ErrorHandler(&i2cTransaction);
//            led_display_ErrorPriority(I2C_OPEN_NULL);  //  error protocol here
        }
    }
    return i2cTransferStatus;
}
/*********************************************************************
 *  ======== i2cErrorHandler ========
 *********************************************************************/
uint8_t UDHAL_I2C_ErrorStatus = 0;
static void UDHAL_I2C_ErrorHandler(I2C_Transaction *transaction)
{
    switch (transaction->status)
    {
        case I2C_STATUS_TIMEOUT:
            UDHAL_I2C_ErrorStatus = 1;
            break;
        case I2C_STATUS_CLOCK_TIMEOUT:
            UDHAL_I2C_ErrorStatus = 2;
            break;
        case I2C_STATUS_ADDR_NACK:
            UDHAL_I2C_ErrorStatus = 3;
            break;
        case I2C_STATUS_DATA_NACK:
            UDHAL_I2C_ErrorStatus = 4;
            break;
        case I2C_STATUS_ARB_LOST:
            UDHAL_I2C_ErrorStatus = 5;
            break;
        case I2C_STATUS_INCOMPLETE:
            UDHAL_I2C_ErrorStatus = 6;
            break;
        case I2C_STATUS_BUS_BUSY:
            UDHAL_I2C_ErrorStatus = 7;
            break;
        case I2C_STATUS_CANCEL:
            UDHAL_I2C_ErrorStatus = 8;
            break;
        case I2C_STATUS_INVALID_TRANS:
            UDHAL_I2C_ErrorStatus = 9;
            break;
        case I2C_STATUS_ERROR:
            UDHAL_I2C_ErrorStatus = 10;
            break;
        default:
            UDHAL_I2C_ErrorStatus = 11;
            break;
    }
}



