/*
 * led_display.h
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#ifndef APPLICATION_LED_DISPLAY_H_
#define APPLICATION_LED_DISPLAY_H_

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
#include <ti/sysbios/knl/Task.h>

#include "Hardware/gGo_device_params.h"
/*********************************************************************
* CONSTANTS
*/
#define LED_POWER_LIGHT_ON                          1
#define LED_POWER_LIGHT_OFF                         0

#define LEDSPEED_LOWCOUNT                           5
#define LEDSPEED_HIGHCOUNT                          16

/* ********************************************************************
 * TYPEDEFS
*/
/*********************************************************************
 * @Structure led_display_LedDisplayManager_t
 *
 * @brief     It defines a set of function pointer that the the library can access and control the device peripheral to manipulate the ALS
 *
 * @data      led_display_open: Called when the application wants to open the ALS
 *            led_display_close: Called when the application wants to close the ALS
 *            led_display_transfer: Called when the application wants to transfer data to the ALS

 */

typedef void (*led_display_open)(void);
typedef uint8_t (*led_display_transfer)(uint_least8_t slave_address, void *writeBuffer, size_t writeSize, void *readBuffer, size_t readSize);
typedef void (*led_display_close)(void);

typedef struct
{
    led_display_open            led_display_open;
    led_display_transfer        led_display_transfer;
    led_display_close           led_display_close;
}led_display_ledDisplayManager_t;

/* ********************************************************************
* MACROS
*/

/* ********************************************************************
 * API FUNCTIONS
 */
extern void led_display_registerLedDisplay( led_display_ledDisplayManager_t *ledDisplayI2C );

/*********************************************************************
 *  Global Function declaration
 *
*/
extern void led_display_init( void );
extern void led_display_deinit(void);

extern void led_display_setAllOn( void );                                // Led All On
extern void led_display_setAllOff( void );                               // Led All Off
extern void led_display_setDashSpeed( uint8_t dashSpeed );               // Set Speed Digit 1 and Digit 2
extern void led_display_changeDashSpeed();
extern void led_display_setBatteryStatus( uint8_t batteryStatus );       // Set battery level
extern void led_display_changeBatteryStatus(uint32_t eventcounter);
extern void led_display_setSpeedMode( uint8_t speedMode );               // Set speed mode
extern void led_control_setControlLaw(uint8_t controlLaw);                    // Set control law value
extern void led_display_changeSpeedMode(uint32_t eventcounter);
extern void led_display_setUnitSelectDash( uint8_t UnitSelectDash );     // Set Unit
extern void led_display_changeUnit();
extern void led_display_changeBLE(uint32_t eventcounter);
extern uint8_t led_display_ErrorPriority(uint8_t error_code);
extern uint8_t led_display_ErrorDisplay();
extern void led_display_setLightMode( uint8_t light_mode );           // Set light mode
extern void led_display_changeLightMode();
extern void led_display_setLightStatus( uint8_t light_status );          // Set Light Status
extern void led_display_changeLightStatus();
extern void led_display_setLEDPower( uint8_t ledPower );                 // Set LED Power Level / Brightness
extern void led_display_changeLEDPower();
extern void* led_display_errorPriorityRegister();

extern void led_display_opcodeRegister( uint8_t *ptr_opcode );
extern void led_display_advertiseFlagRegister(uint8_t *ptr_advertiseFlag);

typedef void (*IS31FL3236A_Function)(uint8_t status_buf, uint8_t brightness_buf);
typedef enum {
    BRIGHTNESS,
    SPORTS_MODE, LEISURE_MODE, AMBLE_MODE, BLUETOOTH_LED, LIGHT_ON, AUTO_MODE, ATTENTION,
    BAR_1, BAR_2, BAR_3, BAR_4, BAR_5, KMPH, MPH,
    DIGIT_1_PIN_44, DIGIT_1_PIN_3, DIGIT_1_PIN_5, DIGIT_1_PIN_30, DIGIT_1_PIN_31, DIGIT_1_PIN_32, DIGIT_1_PIN_33,
    DIGIT_2_PIN_8, DIGIT_2_PIN_9,DIGIT_2_PIN_12, DIGIT_2_PIN_27, DIGIT_2_PIN_28,  DIGIT_2_PIN_29,DIGIT_2_PIN_13,
    ALL_BARS, BARS_4_ON, BARS_3_ON, BARS_2_ON, BARS_1_ON, BARS_0_ON,
    DIGIT_1_NO_0, DIGIT_1_NO_1, DIGIT_1_NO_2, DIGIT_1_NO_3, DIGIT_1_NO_4,
    DIGIT_1_NO_5, DIGIT_1_NO_6, DIGIT_1_NO_7, DIGIT_1_NO_8, DIGIT_1_NO_9,
    DIGIT_2_NO_0, DIGIT_2_NO_1, DIGIT_2_NO_2, DIGIT_2_NO_3, DIGIT_2_NO_4,
    DIGIT_2_NO_5, DIGIT_2_NO_6, DIGIT_2_NO_7, DIGIT_2_NO_8, DIGIT_2_NO_9,
    DIGIT_1_A, DIGIT_2_A, DIGIT_1_C, DIGIT_2_C, DIGIT_1_E, DIGIT_2_E,
    DIGIT_1_F, DIGIT_2_F, DIGIT_1_H, DIGIT_2_H, DIGIT_1_J, DIGIT_2_J,
    DIGIT_1_L, DIGIT_2_L, DIGIT_1_P, DIGIT_2_P, DIGIT_1_U, DIGIT_2_U,
    DIGIT_1_u, DIGIT_2_u, DIGIT_1_n, DIGIT_2_n, DIGIT_1_b, DIGIT_2_b,
    DIGIT_1_c, DIGIT_2_c, DIGIT_1_h, DIGIT_2_h, DIGIT_1_d, DIGIT_2_d,
    DIGIT_1_q, DIGIT_2_q, DIGIT_1_, DIGIT_2_,
    FUNCTION_COUNT
} IS31FL3236A_FunctionIndex;

extern IS31FL3236A_Function functionTable[FUNCTION_COUNT];

#ifdef _cplusplus
}
#endif


#endif /* APPLICATION_LED_DISPLAY_H_ */
