/*
 * IS31FL3236A.h
 *
 * IS31FL3236A 36-CHANNEL LED DRIVER; SELECTABLE PWM FREQUENCY
 *
 *  Created on: 12 Jul 2023
 *      Author: Jermyn & Chee
 */

#ifndef HARDWARE_IS31FL3236A_H_
#define HARDWARE_IS31FL3236A_H_


#ifdef _cplusplus
extern "C"
{
#endif
//
/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
//#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

#include "Hardware/gGo_device_params.h"


/*********************************************************************
 * CONSTANTS
 */
/*********************************************************************
* CONSTANTS
*/
#define IS31FL3236A_ADDR            0x3C    // AD to GND = 0x78, AD to VCC = 0x7E, AD to SCL = 0x7A, AD to SDA = 0x7C
#define IS31FL3236A_RESET_REG       0x4F    // Reset all registers into default value
#define IS31FL3236A_ON_REG          0x00    // Set software shutdown mode
#define IS31FL3236A_ENABLE_REG      0x4A    // Set all channels enable
#define IS31FL3236A_UPDATE_REG      0x25    // Load PWM Register and LED Control Register’s data
#define IS31FL3236A_PWM_FREQUENCY   0x4B    // Set all channels operating frequency


#define ZERO_ONE_CUSTOM             0x07
#define ZERO_ZERO_CUSTOM            0x00

//PWM Brightness control
#define PWM_CUSTOM                  0xC8 // 0xE6 = 230, 0xC8 = 200, 0xAF = 175, 0x80 = 128, 0x32 = 50 //255 STEPS Original PWM_CUSTOM: 0xCF
#define PWM_LOW                     0x80 //0x9F
#define PWM_ZERO                    0x00


//Brightness 32 Gamma Steps
extern const uint8_t brightness_32[];

//Brightness 64 Gamma Steps
extern const uint8_t brightness_64[];

//Testing all leds
extern const uint8_t led_array[][2];

//LED Current control(affect brightness)
//status_buf: last bit must be 1 to let led on, 6th and 7th bits set output current
#define I_MAX_OUT                    0x01 //MAX current 23mA
#define I_MAX2_OUT                   0x03 //MAX/2
#define I_MAX3_OUT                   0x05 //MAX/3
#define I_MAX4_OUT                   0x07 //MAX/4
#define I_OUT                        0x05

//LED control register address

/* Address for LED Status */
#define IS31FL3236A_LED43_ADDR       0x26
#define IS31FL3236A_LED44_ADDR       0x27

#define IS31FL3236A_LED1_ADDR        0x28
#define IS31FL3236A_LED2_ADDR        0x29
#define IS31FL3236A_LED3_ADDR        0x2A
#define IS31FL3236A_LED4_ADDR        0x2B
#define IS31FL3236A_LED5_ADDR        0x2C
#define IS31FL3236A_LED6_ADDR        0x2D
#define IS31FL3236A_LED7_ADDR        0x2E
#define IS31FL3236A_LED8_ADDR        0x2F
#define IS31FL3236A_LED9_ADDR        0x30
#define IS31FL3236A_LED10_ADDR       0x31
#define IS31FL3236A_LED11_ADDR       0x32
#define IS31FL3236A_LED12_ADDR       0x33
#define IS31FL3236A_LED13_ADDR       0x34

#define IS31FL3236A_LED19_ADDR       0x39
#define IS31FL3236A_LED20_ADDR       0x3A
#define IS31FL3236A_LED21_ADDR       0x3B
#define IS31FL3236A_LED22_ADDR       0x3C

#define IS31FL3236A_LED27_ADDR       0x41
#define IS31FL3236A_LED28_ADDR       0x42
#define IS31FL3236A_LED29_ADDR       0x43
#define IS31FL3236A_LED30_ADDR       0x44
#define IS31FL3236A_LED31_ADDR       0x45
#define IS31FL3236A_LED32_ADDR       0x46
#define IS31FL3236A_LED33_ADDR       0x47
#define IS31FL3236A_LED34_ADDR       0x48
#define IS31FL3236A_LED35_ADDR       0x49

/* Address for PWM value of LED */
#define IS31FL3236A_LED43_PWM_ADDR   0x01
#define IS31FL3236A_LED44_PWM_ADDR   0x02

#define IS31FL3236A_LED1_PWM_ADDR    0x03
#define IS31FL3236A_LED2_PWM_ADDR    0x04
#define IS31FL3236A_LED3_PWM_ADDR    0x05
#define IS31FL3236A_LED4_PWM_ADDR    0x06
#define IS31FL3236A_LED5_PWM_ADDR    0x07
#define IS31FL3236A_LED6_PWM_ADDR    0x08
#define IS31FL3236A_LED7_PWM_ADDR    0x09
#define IS31FL3236A_LED8_PWM_ADDR    0x0A
#define IS31FL3236A_LED9_PWM_ADDR    0x0B
#define IS31FL3236A_LED10_PWM_ADDR   0x0C
#define IS31FL3236A_LED11_PWM_ADDR   0x0D
#define IS31FL3236A_LED12_PWM_ADDR   0x0E
#define IS31FL3236A_LED13_PWM_ADDR   0x0F

#define IS31FL3236A_LED19_PWM_ADDR   0x14
#define IS31FL3236A_LED20_PWM_ADDR   0x15
#define IS31FL3236A_LED21_PWM_ADDR   0x16
#define IS31FL3236A_LED22_PWM_ADDR   0x17

#define IS31FL3236A_LED27_PWM_ADDR   0x1C
#define IS31FL3236A_LED28_PWM_ADDR   0x1D
#define IS31FL3236A_LED29_PWM_ADDR   0x1E
#define IS31FL3236A_LED30_PWM_ADDR   0x1F
#define IS31FL3236A_LED31_PWM_ADDR   0x20
#define IS31FL3236A_LED32_PWM_ADDR   0x21
#define IS31FL3236A_LED33_PWM_ADDR   0x22
#define IS31FL3236A_LED34_PWM_ADDR   0x23
#define IS31FL3236A_LED35_PWM_ADDR   0x24

/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 *  Global Function declaration
 *
**********************************************************************/



#ifdef _cplusplus
}
#endif



#endif /* HARDWARE_IS31FL3236A_H_ */
