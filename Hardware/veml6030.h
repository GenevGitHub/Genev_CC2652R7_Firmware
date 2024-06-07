/*
 * veml6030.h
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#ifndef HARDWARE_VEML6030_H_
#define HARDWARE_VEML6030_H_

#ifdef _cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "Hardware/gGo_device_params.h"

#include "Application/ALS_control.h"

//https://www.vishay.com/docs/84366/veml6030.pdf
////////////////////////////
#ifdef veml6030
// VEML6030 Command Codes //
////////////////////////////
#define  VEML6030_ALS_CONF        0x00 // command codes
#define  VEML6030_ALS_WH          0x01
#define  VEML6030_ALS_WL          0x02
#define  VEML6030_PWR_SAVE        0x03
#define  VEML6030_ALS             0x04
#define  VEML6030_WHITE           0x05
#define  VEML6030_ALS_INT         0x06
#define  VEML6030_ID              0x07

#define VEML6030_ADDRL            0x10 // 0x10 when address pin LOW, 0x48 when HIGH
#define VEML6030_ADDRH            0x48

#define  IT_100         0x00  //   100 ms, 0000b /ALS integration time setting
#define  IT_200         0x01  //   200 ms, 0001b
#define  IT_400         0x02  //   400 ms, 0010b
#define  IT_800         0x03  //   800 ms, 0011b
#define  IT_50          0x08  //    50 ms, 1000b
#define  IT_25          0x0C  //    25 ms, 1100b

#define Gain_1x         0x00 // 1x gain
#define Gain_2x         0x01 // 2x gain
#define Gain_0_125x     0x02 // 1/8 x gain
#define Gain_0_25x      0x03 // 1/4 x gain

#define PERS1           0x00 // 1
#define PERS2           0x01 // 2
#define PERS4           0x10 // 4
#define PERS8           0x11 // 8

#define PSM1            0x00 // 1
#define PSM2            0x01 // 2
#define PSM3            0x10 // 3
#define PSM4            0x11 // 4

#define INT_EN          1 // Interrupt enable
#define INT_DIS         0 // Interrupt disable

#define PSM_EN          1 // Power Saving enable
#define PSM_DI          0 // Power Saving disable

#define ALS_POWERON     0 // ALS power on
#define ALS_SHUTDOWN    1 // ALS shut down

#define RES_Gain_2x_IT_100  0.0336
#define RES_Gain_2x_IT_200  0.0168
#define RES_Gain_2x_IT_400  0.0084
#define RES_Gain_2x_IT_800  0.0042
#define RES_Gain_2x_IT_50   0.0672
#define RES_Gain_2x_IT_25   0.1344

#define COEFFICIENTA    0.00000000000060135
#define COEFFICIENTB    -0.0000000093924
#define COEFFICIENTC    0.000081488
#define COEFFICIENTD    1.0023
/*********************************************************************
 * @Structure veml6030_ALSManager_t
 *
 * @brief     It defines a set of function pointer that the the library can access and control the device peripheral to manipulate the ALS
 *
 * @data      ALS_open: Called when the application wants to open the ALS
 *            ALS_close: Called when the application wants to close the ALS
 *            ALS_transfer: Called when the application wants to transfer data to the ALS
 *********************************************************************/
typedef void (*ALS_open)(void);
typedef uint8_t (*ALS_transfer)(uint_least8_t slave_address, void *writeBuffer, size_t writeSize, void *readBuffer, size_t readSize);
typedef void (*ALS_close)(void);

typedef struct{
    ALS_open            ALS_open;
    ALS_transfer        ALS_transfer;
    ALS_close           ALS_close;
}ALSManager_t;

extern void veml6030_registerALS( ALSManager_t *ALSI2C );

extern void veml6030_init(uint8_t ALS_Gain, uint16_t ALS_IT, uint8_t Pers, uint8_t Interrupt, uint8_t ALS_SD);
extern void veml6030_shutDown(void);
extern void veml6030_PSM(uint8_t PSM_mode, uint8_t PSM_enable);
extern void veml6030_setIntThreshold(uint8_t HighLow, uint16_t threshold_value);
extern void veml6030_resolution(uint8_t ALSGain_select, uint16_t ALSIT_select);
extern void veml6030_read(uint8_t read_reg);
extern float veml6030_calculateLux(void);


#endif

#ifdef _cplusplus
}
#endif





#endif /* HARDWARE_VEML6030_H_ */
