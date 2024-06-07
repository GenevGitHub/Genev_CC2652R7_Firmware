/*
 * veml3235.h
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#ifndef HARDWARE_VEML3235_H_
#define HARDWARE_VEML3235_H_

#ifdef _cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include<stdio.h>
#include<stdint.h>
#include<stdlib.h>

#include "Hardware/gGo_device_params.h"

#include "Application/ALS_control.h"

//https://www.vishay.com/docs/80131/veml3235.pdf
////////////////////////////
#ifdef veml3235
// VEML3235 Command Codes //
////////////////////////////
#define  VEML3235_ALS_CONF        0x00 // command codes
#define  VEML3235_RESERVE         0x02
#define  VEML3235_ALS             0x04
#define  VEML3235_WHITE           0x05
#define  VEML3235_ID              0x09

#define VEML3235_ADDR             0x10

#define  IT_50          0x00  //    50 ms, 000b
#define  IT_100         0x01  //   100 ms, 001b /ALS integration time setting
#define  IT_200         0x02  //   200 ms, 010b
#define  IT_400         0x03  //   400 ms, 011b
#define  IT_800         0x04  //   800 ms, 100b

#define DG0             0   // x1
#define DG1             1   // x2

#define DG0_VALUE       1   // x1
#define DG1_VALUE       2   // x2

#define Gain_1x         0x00 // 1x gain
#define Gain_2x         0x01 // 2x gain
#define Gain_4x         0x03 // 4x gain

#define ALS_POWERON     0x00 // ALS, BG and LDO power on (0000 0000)
#define ALS_POWEROFF    0x01 // BG and LDO shut down (0000 0001)
#define ALS_SHUTDOWN    0x81 // ALS, BG and LDO shut down (1000 0001)

#define RES_Gain_2x_IT_50   0.13632
#define RES_Gain_2x_IT_100  0.06816
#define RES_Gain_2x_IT_200  0.03408
#define RES_Gain_2x_IT_400  0.01704
#define RES_Gain_2x_IT_800  0.00852

/*********************************************************************
 * @Structure veml3235_ALSManager_t
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

extern void veml3235_registerALS( ALSManager_t *ALSI2C );

extern void veml3235_init(uint8_t ALS_Gain, uint16_t ALS_IT, uint8_t ALS_DG, uint8_t ALS_SD);
extern void veml3235_shutDown(void);
extern void veml3235_resolution(uint8_t ALSGain_select, uint16_t ALSIT_select, uint8_t ALSDG_select);
extern void veml3235_read(uint8_t read_reg);
extern float veml3235_calculateLux(void);

#endif

#ifdef _cplusplus
}
#endif

#endif /* HARDWARE_VEML3235_H_ */
