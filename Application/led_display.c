/*
 * led_display.c
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

/*********************************************************************
* INCLUDES
*/
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include "Application/led_display.h"
#include "Hardware/IS31FL3236A.h"

/*********************************************************************
* LOCAL VARIABLES
*/
uint8_t ledSetpower;
uint8_t ledSpeed;
uint8_t ledBatteryStatus;
uint8_t ledSpeedMode;
uint8_t ledUnitSelectDash;
uint8_t ledBLEStatus;
uint8_t ledErrorCode;
uint8_t ledLightMode;
uint8_t ledLightStatus;
uint8_t led_display_i2cTransferStatus;
uint8_t ledLightMode_old;
uint8_t ledBLESelect_old;
uint8_t ledUnitSelect_old;
uint8_t ledSpeedModeSelect_old;
uint8_t led_controlLaw_ledStatus;
uint8_t led_controlLaw_old;
uint8_t ledSpeed_old;
uint8_t ledBatteryStatus_old;
uint8_t ledError_old;
uint8_t ledLightStatus_old;
uint8_t ledPower_old;
uint8_t ledBrightness;
uint8_t ledBrightness_old;
uint8_t led_error_priority;
uint8_t led_error_code_old;
uint8_t battery_bar1_status;
uint8_t BLE_flash_status;
uint8_t led_controlLaw;

static uint8_t      speedmodeIsLocked = 0;  //Chee added 20250110

uint8_t led_allOn = 0;

/*********************************************************************
*
* LOCAL FUNCTIONS
*/
static led_display_ledDisplayManager_t *led_display_ledDisplayManager;


//LED basic functions
void reset_led_driver()
{
    uint8_t writeBuf_reset[2] = {IS31FL3236A_RESET_REG, 0x00};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_reset, sizeof(writeBuf_reset), NULL, 0);
}

void turn_on_led_driver()
{
    uint8_t writeBuf_shutdown[2] = {IS31FL3236A_ON_REG, ZERO_ONE_CUSTOM};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_shutdown, sizeof(writeBuf_shutdown), NULL, 0);
}

void turn_off_led_driver()
{
    uint8_t writeBuf_shutdown[2] = {IS31FL3236A_ON_REG, ZERO_ZERO_CUSTOM};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_shutdown, sizeof(writeBuf_shutdown), NULL, 0);
}

void disable_channels()
{
    uint8_t writeBuf_disable[2] = {IS31FL3236A_ENABLE_REG, 0x01};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_disable, sizeof(writeBuf_disable), NULL, 0);
}

void enable_channels()
{
    uint8_t writeBuf_enable[2] = {IS31FL3236A_ENABLE_REG, 0x00};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_enable, sizeof(writeBuf_enable), NULL, 0);
}


void update_bit()
{
    uint8_t writeBuf_update[2] = {IS31FL3236A_UPDATE_REG, 0x00};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_update, sizeof(writeBuf_update), NULL, 0);
}

void PWM_Frequency_22k()
{
    uint8_t writeBuf_22k[2] = {IS31FL3236A_PWM_FREQUENCY, 0x01};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_22k, sizeof(writeBuf_22k), NULL, 0);
}
void PWM_Frequency_3k()
{
    uint8_t writeBuf_3k[2] = {IS31FL3236A_PWM_FREQUENCY, 0x00};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_3k, sizeof(writeBuf_3k), NULL, 0);
}

void IS31FL3236A_Sports_Mode_pin(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED43_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED43_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}

void IS31FL3236A_Leisure_Mode_pin(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED35_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED35_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}

void IS31FL3236A_Amble_Mode_pin(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED34_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED34_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}

void IS31FL3236A_Sports_Mode(uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Sports_Mode_pin(status_buf, brightness_buf);
    IS31FL3236A_Leisure_Mode_pin(status_buf, PWM_ZERO);
    IS31FL3236A_Amble_Mode_pin(status_buf, PWM_ZERO);
    update_bit();
}
void IS31FL3236A_Leisure_Mode(uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Sports_Mode_pin(status_buf, PWM_ZERO);
    IS31FL3236A_Leisure_Mode_pin(status_buf, brightness_buf);
    IS31FL3236A_Amble_Mode_pin(status_buf, PWM_ZERO);
    update_bit();
}
void IS31FL3236A_Amble_Mode(uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Sports_Mode_pin(status_buf, PWM_ZERO);
    IS31FL3236A_Leisure_Mode_pin(status_buf, PWM_ZERO);
    IS31FL3236A_Amble_Mode_pin(status_buf, brightness_buf);
    update_bit();
}

void IS31FL3236A_Bluetooth_LED(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED19_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED19_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);
    update_bit();
}

void IS31FL3236A_Light_On(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED20_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED20_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);
    update_bit();
}

void IS31FL3236A_Auto_Mode_pin(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED21_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED21_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);
    update_bit();
}

void IS31FL3236A_Auto_Mode( uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Light_On(status_buf, brightness_buf);
    IS31FL3236A_Auto_Mode_pin(status_buf, brightness_buf);
}

void IS31FL3236A_Attention(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED22_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED22_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);
    update_bit();
}

void IS31FL3236A_1_Bar(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED1_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED1_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}

void IS31FL3236A_2_Bar(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED2_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED2_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}

void IS31FL3236A_3_Bar(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED6_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED6_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}

void IS31FL3236A_4_Bar(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED10_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED10_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}

void IS31FL3236A_5_Bar(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED11_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED11_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}

void IS31FL3236A_kmph_pin(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED4_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED4_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);
}

void IS31FL3236A_mph_pin(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED7_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED7_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);
}
void IS31FL3236A_kmph(uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_kmph_pin(status_buf, brightness_buf);        // mph Off
    IS31FL3236A_mph_pin(status_buf, PWM_ZERO);               // kmph On
    update_bit();
}
void IS31FL3236A_mph(uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_kmph_pin(status_buf, PWM_ZERO);              // kmph Off
    IS31FL3236A_mph_pin(status_buf, brightness_buf);         // mph On
    update_bit();
}

void IS31FL3236A_Digit_1_PIN_44(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED44_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED44_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}
void IS31FL3236A_Digit_1_PIN_3(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED3_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED3_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}
void IS31FL3236A_Digit_1_PIN_5(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED5_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED5_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}
void IS31FL3236A_Digit_1_PIN_30(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED30_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED30_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}
void IS31FL3236A_Digit_1_PIN_31(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED31_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED31_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}
void IS31FL3236A_Digit_1_PIN_32(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED32_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED32_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}
void IS31FL3236A_Digit_1_PIN_33(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED33_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED33_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}

void IS31FL3236A_Digit_2_PIN_8(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED8_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED8_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}
void IS31FL3236A_Digit_2_PIN_9(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED9_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED9_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}
void IS31FL3236A_Digit_2_PIN_12(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED12_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED12_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}
void IS31FL3236A_Digit_2_PIN_27(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED27_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED27_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}
void IS31FL3236A_Digit_2_PIN_28(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED28_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED28_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}
void IS31FL3236A_Digit_2_PIN_29(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED29_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED29_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);

}
void IS31FL3236A_Digit_2_PIN_13(uint8_t status_buf, uint8_t brightness_buf)
{
    uint8_t writeBuf_pwm[2] = {IS31FL3236A_LED13_PWM_ADDR, brightness_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_pwm, sizeof(writeBuf_pwm), NULL, 0);
    uint8_t writeBuf_led[2] = {IS31FL3236A_LED13_ADDR, status_buf};
    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &writeBuf_led, sizeof(writeBuf_led), NULL, 0);
    update_bit();
}

void IS31FL3236A_All_Bars (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_1_Bar(status_buf,brightness_buf);
    IS31FL3236A_2_Bar(status_buf,brightness_buf);
    IS31FL3236A_3_Bar(status_buf,brightness_buf);
    IS31FL3236A_4_Bar(status_buf,brightness_buf);
    IS31FL3236A_5_Bar(status_buf,brightness_buf);
    update_bit();
}

void IS31FL3236A_4_Bars (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_1_Bar(status_buf,brightness_buf);
    IS31FL3236A_2_Bar(status_buf,brightness_buf);
    IS31FL3236A_3_Bar(status_buf,brightness_buf);
    IS31FL3236A_4_Bar(status_buf,brightness_buf);
    IS31FL3236A_5_Bar(status_buf,PWM_ZERO);
    update_bit();
}
void IS31FL3236A_3_Bars (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_1_Bar(status_buf,brightness_buf);
    IS31FL3236A_2_Bar(status_buf,brightness_buf);
    IS31FL3236A_3_Bar(status_buf,brightness_buf);
    IS31FL3236A_4_Bar(status_buf,PWM_ZERO);
    IS31FL3236A_5_Bar(status_buf,PWM_ZERO);
    update_bit();
}
void IS31FL3236A_2_Bars (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_1_Bar(status_buf,brightness_buf);
    IS31FL3236A_2_Bar(status_buf,brightness_buf);
    IS31FL3236A_3_Bar(status_buf,PWM_ZERO);
    IS31FL3236A_4_Bar(status_buf,PWM_ZERO);
    IS31FL3236A_5_Bar(status_buf,PWM_ZERO);
    update_bit();
}
void IS31FL3236A_1_Bars (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_1_Bar(status_buf,brightness_buf);
    IS31FL3236A_2_Bar(status_buf,PWM_ZERO);
    IS31FL3236A_3_Bar(status_buf,PWM_ZERO);
    IS31FL3236A_4_Bar(status_buf,PWM_ZERO);
    IS31FL3236A_5_Bar(status_buf,PWM_ZERO);
    update_bit();
}
void IS31FL3236A_0_Bars (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_1_Bar(status_buf,PWM_ZERO);
    IS31FL3236A_2_Bar(status_buf,PWM_ZERO);
    IS31FL3236A_3_Bar(status_buf,PWM_ZERO);
    IS31FL3236A_4_Bar(status_buf,PWM_ZERO);
    IS31FL3236A_5_Bar(status_buf,PWM_ZERO);
    update_bit();
}


void IS31FL3236A_Digit_1_Number_0 (uint8_t status_buf, uint8_t brightness_buf){

    IS31FL3236A_Digit_1_PIN_44(status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3(status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_5(status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_30(status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31(status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32(status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33(status_buf,PWM_ZERO);
    update_bit();
}

void IS31FL3236A_Digit_1_Number_1 (uint8_t status_buf, uint8_t brightness_buf){

    IS31FL3236A_Digit_1_PIN_44( status_buf,PWM_ZERO);        // PIN_44 off
    IS31FL3236A_Digit_1_PIN_3( status_buf,PWM_ZERO);         // PIN_3 off
    IS31FL3236A_Digit_1_PIN_5( status_buf,brightness_buf);   // PIN_5 on
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);  // PIN_30 on
    IS31FL3236A_Digit_1_PIN_31( status_buf,PWM_ZERO);        // PIN_31 off
    IS31FL3236A_Digit_1_PIN_32( status_buf,PWM_ZERO);        // PIN_32 off
    IS31FL3236A_Digit_1_PIN_33( status_buf,PWM_ZERO);        // PIN_33 off
    update_bit();
}

void IS31FL3236A_Digit_1_Number_2 (uint8_t status_buf, uint8_t brightness_buf){

    IS31FL3236A_Digit_1_PIN_44( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_3( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_5( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_30( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}

void IS31FL3236A_Digit_1_Number_3 (uint8_t status_buf, uint8_t brightness_buf){

    IS31FL3236A_Digit_1_PIN_44( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_3( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_5( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}

void IS31FL3236A_Digit_1_Number_4 (uint8_t status_buf, uint8_t brightness_buf){

    IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_5( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_32( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}

void IS31FL3236A_Digit_1_Number_5 (uint8_t status_buf, uint8_t brightness_buf){

    IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_5( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}

void IS31FL3236A_Digit_1_Number_6 (uint8_t status_buf, uint8_t brightness_buf){

    IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_5( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}

void IS31FL3236A_Digit_1_Number_7 (uint8_t status_buf, uint8_t brightness_buf){

    IS31FL3236A_Digit_1_PIN_44( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_3( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_5( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_32( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_33( status_buf,PWM_ZERO);
    update_bit();
}

void IS31FL3236A_Digit_1_Number_8 (uint8_t status_buf, uint8_t brightness_buf){

    IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_5( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}

void IS31FL3236A_Digit_1_Number_9 (uint8_t status_buf, uint8_t brightness_buf){

    IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_5( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}

void IS31FL3236A_Digit_1_off (uint8_t status_buf){

    IS31FL3236A_Digit_1_PIN_44( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_3( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_5( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_30( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_31( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_32( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_33( status_buf,PWM_ZERO);
    update_bit();
}

void IS31FL3236A_Digit_2_Number_0 (uint8_t status_buf, uint8_t brightness_buf){

        IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_9( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_13( status_buf,PWM_ZERO);
        update_bit();
}

void IS31FL3236A_Digit_2_Number_1 (uint8_t status_buf, uint8_t brightness_buf){

        IS31FL3236A_Digit_2_PIN_8( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_9( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_28( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_29( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_13( status_buf,PWM_ZERO);
        update_bit();
}

void IS31FL3236A_Digit_2_Number_2 (uint8_t status_buf, uint8_t brightness_buf){

        IS31FL3236A_Digit_2_PIN_8( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_9( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_27( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
        update_bit();
}

void IS31FL3236A_Digit_2_Number_3 (uint8_t status_buf, uint8_t brightness_buf){

        IS31FL3236A_Digit_2_PIN_8( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_9( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_29( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
        update_bit();
}

void IS31FL3236A_Digit_2_Number_4 (uint8_t status_buf, uint8_t brightness_buf){

        IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_9( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_28( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_29( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
        update_bit();
}

void IS31FL3236A_Digit_2_Number_5 (uint8_t status_buf, uint8_t brightness_buf){

        IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_9( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_12( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_29( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
        update_bit();
}

void IS31FL3236A_Digit_2_Number_6 (uint8_t status_buf, uint8_t brightness_buf){

        IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_9( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_12( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
        update_bit();
}

void IS31FL3236A_Digit_2_Number_7 (uint8_t status_buf, uint8_t brightness_buf){

        IS31FL3236A_Digit_2_PIN_8( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_9( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_28( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_29( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_13( status_buf,PWM_ZERO);
        update_bit();
}

void IS31FL3236A_Digit_2_Number_8 (uint8_t status_buf, uint8_t brightness_buf){

        IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_9( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
        update_bit();
}

void IS31FL3236A_Digit_2_Number_9 (uint8_t status_buf, uint8_t brightness_buf){

        IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_9( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_29( status_buf,PWM_ZERO);
        IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
        update_bit();
}

void Brightness (uint8_t status_buf, uint8_t brightness_buf)
{

        IS31FL3236A_Sports_Mode_pin(status_buf,brightness_buf);
        IS31FL3236A_Leisure_Mode_pin(status_buf,brightness_buf);
        IS31FL3236A_Amble_Mode_pin(status_buf,brightness_buf);
        IS31FL3236A_Bluetooth_LED(status_buf,brightness_buf);
        IS31FL3236A_Light_On(status_buf,brightness_buf);
        IS31FL3236A_Auto_Mode_pin(status_buf,brightness_buf);
        IS31FL3236A_Attention(status_buf,brightness_buf);
        IS31FL3236A_1_Bar(status_buf,brightness_buf);
        IS31FL3236A_2_Bar(status_buf,brightness_buf);
        IS31FL3236A_3_Bar(status_buf,brightness_buf);
        IS31FL3236A_4_Bar(status_buf,brightness_buf);
        IS31FL3236A_5_Bar(status_buf,brightness_buf);
        IS31FL3236A_kmph_pin(status_buf,brightness_buf);
        IS31FL3236A_mph_pin(status_buf,brightness_buf);
        IS31FL3236A_Digit_1_PIN_44(status_buf,brightness_buf);
        IS31FL3236A_Digit_1_PIN_3(status_buf,brightness_buf);
        IS31FL3236A_Digit_1_PIN_5(status_buf,brightness_buf);
        IS31FL3236A_Digit_1_PIN_30(status_buf,brightness_buf);
        IS31FL3236A_Digit_1_PIN_31(status_buf,brightness_buf);
        IS31FL3236A_Digit_1_PIN_32(status_buf,brightness_buf);
        IS31FL3236A_Digit_1_PIN_33(status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_8(status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_9(status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_12(status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_27(status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_28(status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_29(status_buf,brightness_buf);
        IS31FL3236A_Digit_2_PIN_13(status_buf,brightness_buf);
}

void IS31FL3236A_Digit_1_Number_A (uint8_t status_buf, uint8_t brightness_buf){
       IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
       IS31FL3236A_Digit_1_PIN_3( status_buf,brightness_buf);
       IS31FL3236A_Digit_1_PIN_5( status_buf,brightness_buf);
       IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
       IS31FL3236A_Digit_1_PIN_31( status_buf,PWM_ZERO);
       IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
       IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
       update_bit();
}
void IS31FL3236A_Digit_2_Number_A (uint8_t status_buf, uint8_t brightness_buf){
       IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
       IS31FL3236A_Digit_2_PIN_9( status_buf,brightness_buf);
       IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
       IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
       IS31FL3236A_Digit_2_PIN_28( status_buf,PWM_ZERO);
       IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
       IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
       update_bit();
}
void IS31FL3236A_Digit_1_Number_C (uint8_t status_buf, uint8_t brightness_buf){
       IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
       IS31FL3236A_Digit_1_PIN_3( status_buf,brightness_buf);
       IS31FL3236A_Digit_1_PIN_5( status_buf,PWM_ZERO);
       IS31FL3236A_Digit_1_PIN_30( status_buf,PWM_ZERO);
       IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
       IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
       IS31FL3236A_Digit_1_PIN_33( status_buf,PWM_ZERO);
       update_bit();
}
void IS31FL3236A_Digit_2_Number_C (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_9( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_12( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_27( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_13( status_buf,PWM_ZERO);
}
void IS31FL3236A_Digit_1_Number_E (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_5( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_30( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_E (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_9( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_12( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_27( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
    update_bit();

}
void IS31FL3236A_Digit_1_Number_F (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_5( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_30( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_31( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_F (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_9( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_12( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_27( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_28( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_1_Number_H (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_5( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_H (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_9( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_28( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_1_Number_J (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_3( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_5( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,PWM_ZERO);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_J (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_9( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_13( status_buf,PWM_ZERO);
    update_bit();
}
void IS31FL3236A_Digit_1_Number_L (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_5( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_30( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,PWM_ZERO);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_L (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_9( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_12( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_27( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_13( status_buf,PWM_ZERO);
    update_bit();
}
void IS31FL3236A_Digit_1_Number_P (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_5( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_30( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_31( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_P (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_9( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_27( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_28( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_1_Number_U (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_5( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,PWM_ZERO);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_U (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_9( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_13( status_buf,PWM_ZERO);
    update_bit();
}
void IS31FL3236A_Digit_1_Number_u (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_3( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_5( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,PWM_ZERO);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_u (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_9( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_12( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_13( status_buf,PWM_ZERO);
    update_bit();
}

void IS31FL3236A_Digit_1_Number_n (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_3( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_5( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_n (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_9( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_12( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_28( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_1_Number_b (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_5( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_b (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_9( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_12( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_1_Number_c (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_3( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_5( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_30( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_c (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_9( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_12( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_27( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_1_Number_h (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_5( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_h (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_9( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_12( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_28( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_1_Number_d (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_3( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_5( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_32( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_d (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_9( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_28( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_29( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_1_Number_q (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_3( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_5( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_30( status_buf,brightness_buf);
    IS31FL3236A_Digit_1_PIN_31( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_32( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_q (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_9( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_12( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_27( status_buf,brightness_buf);
    IS31FL3236A_Digit_2_PIN_28( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_29( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_1_Number_ (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_1_PIN_44( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_3( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_5( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_30( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_31( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_32( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_1_PIN_33( status_buf,brightness_buf);
    update_bit();
}
void IS31FL3236A_Digit_2_Number_ (uint8_t status_buf, uint8_t brightness_buf){
    IS31FL3236A_Digit_2_PIN_8( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_9( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_12( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_27( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_28( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_29( status_buf,PWM_ZERO);
    IS31FL3236A_Digit_2_PIN_13( status_buf,brightness_buf);
    update_bit();
}

void LED_Turn_OFF_ALL(){
       IS31FL3236A_Sports_Mode_pin(I_OUT,PWM_ZERO);
       IS31FL3236A_Leisure_Mode_pin(I_OUT,PWM_ZERO);
       IS31FL3236A_Amble_Mode_pin(I_OUT,PWM_ZERO);
       IS31FL3236A_Bluetooth_LED(I_OUT,PWM_ZERO);
       IS31FL3236A_Light_On(I_OUT,PWM_ZERO);
       IS31FL3236A_Auto_Mode_pin(I_OUT,PWM_ZERO);
       IS31FL3236A_Attention(I_OUT,PWM_ZERO);
       IS31FL3236A_1_Bar(I_OUT,PWM_ZERO);
       IS31FL3236A_2_Bar(I_OUT,PWM_ZERO);
       IS31FL3236A_3_Bar(I_OUT,PWM_ZERO);
       IS31FL3236A_4_Bar(I_OUT,PWM_ZERO);
       IS31FL3236A_5_Bar(I_OUT,PWM_ZERO);
       IS31FL3236A_kmph_pin(I_OUT,PWM_ZERO);
       IS31FL3236A_mph_pin(I_OUT,PWM_ZERO);
       IS31FL3236A_Digit_1_PIN_44(I_OUT,PWM_ZERO);
       IS31FL3236A_Digit_1_PIN_3(I_OUT,PWM_ZERO);
       IS31FL3236A_Digit_1_PIN_5(I_OUT,PWM_ZERO);
       IS31FL3236A_Digit_1_PIN_30(I_OUT,PWM_ZERO);
       IS31FL3236A_Digit_1_PIN_31(I_OUT,PWM_ZERO);
       IS31FL3236A_Digit_1_PIN_32(I_OUT,PWM_ZERO);
       IS31FL3236A_Digit_1_PIN_33(I_OUT,PWM_ZERO);
       IS31FL3236A_Digit_2_PIN_8(I_OUT,PWM_ZERO);
       IS31FL3236A_Digit_2_PIN_9(I_OUT,PWM_ZERO);
       IS31FL3236A_Digit_2_PIN_12(I_OUT,PWM_ZERO);
       IS31FL3236A_Digit_2_PIN_27(I_OUT,PWM_ZERO);
       IS31FL3236A_Digit_2_PIN_28(I_OUT,PWM_ZERO);
       IS31FL3236A_Digit_2_PIN_29(I_OUT,PWM_ZERO);
       IS31FL3236A_Digit_2_PIN_13(I_OUT,PWM_ZERO);
       update_bit();
}

void LED_Turn_ON_ALL(){
       IS31FL3236A_Sports_Mode_pin(I_OUT,ledBrightness);
       IS31FL3236A_Leisure_Mode_pin(I_OUT,ledBrightness);
       IS31FL3236A_Amble_Mode_pin(I_OUT,ledBrightness);
       IS31FL3236A_Bluetooth_LED(I_OUT,ledBrightness);
       IS31FL3236A_Light_On(I_OUT,ledBrightness);
       IS31FL3236A_Auto_Mode_pin(I_OUT,ledBrightness);
       IS31FL3236A_Attention(I_OUT,ledBrightness);
       IS31FL3236A_1_Bar(I_OUT,ledBrightness);
       IS31FL3236A_2_Bar(I_OUT,ledBrightness);
       IS31FL3236A_3_Bar(I_OUT,ledBrightness);
       IS31FL3236A_4_Bar(I_OUT,ledBrightness);
       IS31FL3236A_5_Bar(I_OUT,ledBrightness);
       IS31FL3236A_kmph_pin(I_OUT,ledBrightness);
       IS31FL3236A_mph_pin(I_OUT,ledBrightness);
       IS31FL3236A_Digit_1_PIN_44(I_OUT,ledBrightness);
       IS31FL3236A_Digit_1_PIN_3(I_OUT,ledBrightness);
       IS31FL3236A_Digit_1_PIN_5(I_OUT,ledBrightness);
       IS31FL3236A_Digit_1_PIN_30(I_OUT,ledBrightness);
       IS31FL3236A_Digit_1_PIN_31(I_OUT,ledBrightness);
       IS31FL3236A_Digit_1_PIN_32(I_OUT,ledBrightness);
       IS31FL3236A_Digit_1_PIN_33(I_OUT,ledBrightness);
       IS31FL3236A_Digit_2_PIN_8(I_OUT,ledBrightness);
       IS31FL3236A_Digit_2_PIN_9(I_OUT,ledBrightness);
       IS31FL3236A_Digit_2_PIN_12(I_OUT,ledBrightness);
       IS31FL3236A_Digit_2_PIN_27(I_OUT,ledBrightness);
       IS31FL3236A_Digit_2_PIN_28(I_OUT,ledBrightness);
       IS31FL3236A_Digit_2_PIN_29(I_OUT,ledBrightness);
       IS31FL3236A_Digit_2_PIN_13(I_OUT,ledBrightness);
       update_bit();
}
// Function tables

IS31FL3236A_Function functionTable[FUNCTION_COUNT] = {
    Brightness,                         //0
    IS31FL3236A_Sports_Mode,
    IS31FL3236A_Leisure_Mode,
    IS31FL3236A_Amble_Mode,
    IS31FL3236A_Bluetooth_LED,
    IS31FL3236A_Light_On,               //5
    IS31FL3236A_Auto_Mode_pin,
    IS31FL3236A_Attention,
    IS31FL3236A_1_Bar,
    IS31FL3236A_2_Bar,
    IS31FL3236A_3_Bar,                  //10
    IS31FL3236A_4_Bar,
    IS31FL3236A_5_Bar,
    IS31FL3236A_kmph,
    IS31FL3236A_mph,
    IS31FL3236A_Digit_1_PIN_44,         //15
    IS31FL3236A_Digit_1_PIN_3,          //16
    IS31FL3236A_Digit_1_PIN_5,
    IS31FL3236A_Digit_1_PIN_30,
    IS31FL3236A_Digit_1_PIN_31,
    IS31FL3236A_Digit_1_PIN_32,         //20
    IS31FL3236A_Digit_1_PIN_33,
    IS31FL3236A_Digit_2_PIN_8,
    IS31FL3236A_Digit_2_PIN_9,
    IS31FL3236A_Digit_2_PIN_12,
    IS31FL3236A_Digit_2_PIN_27,         //25
    IS31FL3236A_Digit_2_PIN_28,
    IS31FL3236A_Digit_2_PIN_29,
    IS31FL3236A_Digit_2_PIN_13,
    IS31FL3236A_All_Bars,               //29
    IS31FL3236A_4_Bars,                 //30
    IS31FL3236A_3_Bars,                 //31
    IS31FL3236A_2_Bars,                 //32
    IS31FL3236A_1_Bars,                 //33
    IS31FL3236A_0_Bars,                 //34
    IS31FL3236A_Digit_1_Number_0,       //35
    IS31FL3236A_Digit_1_Number_1,       //36
    IS31FL3236A_Digit_1_Number_2,       //37
    IS31FL3236A_Digit_1_Number_3,       //38
    IS31FL3236A_Digit_1_Number_4,       //39
    IS31FL3236A_Digit_1_Number_5,       //40
    IS31FL3236A_Digit_1_Number_6,       //41
    IS31FL3236A_Digit_1_Number_7,       //42
    IS31FL3236A_Digit_1_Number_8,       //43
    IS31FL3236A_Digit_1_Number_9,       //44
    IS31FL3236A_Digit_2_Number_0,       //45
    IS31FL3236A_Digit_2_Number_1,
    IS31FL3236A_Digit_2_Number_2,
    IS31FL3236A_Digit_2_Number_3,
    IS31FL3236A_Digit_2_Number_4,
    IS31FL3236A_Digit_2_Number_5,       //50
    IS31FL3236A_Digit_2_Number_6,
    IS31FL3236A_Digit_2_Number_7,
    IS31FL3236A_Digit_2_Number_8,
    IS31FL3236A_Digit_2_Number_9,
    IS31FL3236A_Digit_1_Number_A,       //55
    IS31FL3236A_Digit_2_Number_A,
    IS31FL3236A_Digit_1_Number_C,
    IS31FL3236A_Digit_2_Number_C,
    IS31FL3236A_Digit_1_Number_E,
    IS31FL3236A_Digit_2_Number_E,       //60
    IS31FL3236A_Digit_1_Number_F,
    IS31FL3236A_Digit_2_Number_F,
    IS31FL3236A_Digit_1_Number_H,
    IS31FL3236A_Digit_2_Number_H,
    IS31FL3236A_Digit_1_Number_J,       //65
    IS31FL3236A_Digit_2_Number_J,
    IS31FL3236A_Digit_1_Number_L,
    IS31FL3236A_Digit_2_Number_L,
    IS31FL3236A_Digit_1_Number_P,
    IS31FL3236A_Digit_2_Number_P,       //70
    IS31FL3236A_Digit_1_Number_U,
    IS31FL3236A_Digit_2_Number_U,
    IS31FL3236A_Digit_1_Number_u,
    IS31FL3236A_Digit_2_Number_u,
    IS31FL3236A_Digit_1_Number_n,       //75
    IS31FL3236A_Digit_2_Number_n,
    IS31FL3236A_Digit_1_Number_b,
    IS31FL3236A_Digit_2_Number_b,
    IS31FL3236A_Digit_1_Number_c,
    IS31FL3236A_Digit_2_Number_c,       //80
    IS31FL3236A_Digit_1_Number_h,
    IS31FL3236A_Digit_2_Number_h,
    IS31FL3236A_Digit_1_Number_d,
    IS31FL3236A_Digit_2_Number_d,
    IS31FL3236A_Digit_1_Number_q,       //85
    IS31FL3236A_Digit_2_Number_q,
    IS31FL3236A_Digit_1_Number_,
    IS31FL3236A_Digit_2_Number_,        //88
};

//Steps to call
//IS31FL3236A_FunctionIndex index = LIGHT_ON;
//functionTable[index]( status_buf, brightness_buf);


//Brightness 32 Gamma Steps
const uint8_t brightness_32_ori[]={
                               0x00, 0x01, 0x02, 0x04, 0x06, 0x0A, 0x0D, 0x12,
                               0x16, 0x1C, 0x21, 0x27, 0x2E, 0x35, 0x3D, 0x45,
                               0x4E, 0x56, 0x60, 0x6A, 0x74, 0x7E, 0x8A, 0x95,
                               0xA1, 0xAD, 0xBA, 0xC7, 0xD4, 0xE2, 0xF0, 0xFF
                                };
const uint8_t brightness_32[]={
                               0x00, 0x01, 0x02, 0x04, 0x06, 0x0A, 0x0D, 0x12,
                               0x16, 0x1C, 0x21, 0x27, 0x2E, 0x35, 0x3D, 0x45,
                               0x4E, 0x56, 0x60, 0x6A, 0x74, 0x7E, 0x8A, 0x95,
                               0xA1, 0xAD, 0xBA, 0xC7, 0xD4, 0xE2, 0xF0, 0xFF
                                };

//Brightness 64 Gamma Steps
const uint8_t brightness_64_ori[]={
                               0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                               0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x14, 0x16,
                               0x18, 0x1A, 0x1D, 0x20, 0x23, 0x26, 0x29, 0x2C,
                               0x2F, 0x32, 0x35, 0x39, 0x3D, 0x41, 0x45, 0x49,
                               0x4D, 0x51, 0x55, 0x59, 0x5E, 0x63, 0x68, 0x6D,
                               0x72, 0x77, 0x7C, 0x81, 0x86, 0x8C, 0x92, 0x98,
                               0x9E, 0xA4, 0xAA, 0xB0, 0xB6, 0xBC, 0xC3, 0xCA,
                               0xD1, 0xD8, 0xDF, 0xE6, 0xED, 0xF4, 0xFB, 0xFF
                                };
const uint8_t brightness_64[]={
                               0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                               0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x14, 0x16,
                               0x18, 0x1A, 0x1D, 0x20, 0x23, 0x26, 0x29, 0x2C,
                               0x2F, 0x32, 0x35, 0x39, 0x3D, 0x41, 0x45, 0x49,
                               0x4D, 0x51, 0x55, 0x59, 0x5E, 0x63, 0x68, 0x6D,
                               0x72, 0x77, 0x7C, 0x81, 0x86, 0x8C, 0x92, 0x98,
                               0x9E, 0xA4, 0xAA, 0xB0, 0xB6, 0xBC, 0xC3, 0xCA,
                               0xD1, 0xD8, 0xDF, 0xE6, 0xED, 0xF4, 0xFB, 0xFF
                                };
// LED pin array
const uint8_t led_array[][2] = {
    {IS31FL3236A_LED43_ADDR, IS31FL3236A_LED43_PWM_ADDR},
    {IS31FL3236A_LED44_ADDR, IS31FL3236A_LED44_PWM_ADDR},
    {IS31FL3236A_LED1_ADDR,  IS31FL3236A_LED1_PWM_ADDR},
    {IS31FL3236A_LED2_ADDR,  IS31FL3236A_LED2_PWM_ADDR},
    {IS31FL3236A_LED3_ADDR,  IS31FL3236A_LED3_PWM_ADDR},
    {IS31FL3236A_LED4_ADDR,  IS31FL3236A_LED4_PWM_ADDR},
    {IS31FL3236A_LED5_ADDR,  IS31FL3236A_LED5_PWM_ADDR},
    {IS31FL3236A_LED6_ADDR,  IS31FL3236A_LED6_PWM_ADDR},
    {IS31FL3236A_LED7_ADDR,  IS31FL3236A_LED7_PWM_ADDR},
    {IS31FL3236A_LED8_ADDR,  IS31FL3236A_LED8_PWM_ADDR},
    {IS31FL3236A_LED9_ADDR,  IS31FL3236A_LED9_PWM_ADDR},
    {IS31FL3236A_LED10_ADDR, IS31FL3236A_LED10_PWM_ADDR},
    {IS31FL3236A_LED11_ADDR, IS31FL3236A_LED11_PWM_ADDR},
    {IS31FL3236A_LED12_ADDR, IS31FL3236A_LED12_PWM_ADDR},
    {IS31FL3236A_LED13_ADDR, IS31FL3236A_LED13_PWM_ADDR},
    {IS31FL3236A_LED19_ADDR, IS31FL3236A_LED19_PWM_ADDR},
    {IS31FL3236A_LED20_ADDR, IS31FL3236A_LED20_PWM_ADDR},
    {IS31FL3236A_LED21_ADDR, IS31FL3236A_LED21_PWM_ADDR},
    {IS31FL3236A_LED22_ADDR, IS31FL3236A_LED22_PWM_ADDR},
    {IS31FL3236A_LED27_ADDR, IS31FL3236A_LED27_PWM_ADDR},
    {IS31FL3236A_LED28_ADDR, IS31FL3236A_LED28_PWM_ADDR},
    {IS31FL3236A_LED29_ADDR, IS31FL3236A_LED29_PWM_ADDR},
    {IS31FL3236A_LED30_ADDR, IS31FL3236A_LED30_PWM_ADDR},
    {IS31FL3236A_LED31_ADDR, IS31FL3236A_LED31_PWM_ADDR},
    {IS31FL3236A_LED32_ADDR, IS31FL3236A_LED32_PWM_ADDR},
    {IS31FL3236A_LED33_ADDR, IS31FL3236A_LED33_PWM_ADDR},
    {IS31FL3236A_LED34_ADDR, IS31FL3236A_LED34_PWM_ADDR},
    {IS31FL3236A_LED35_ADDR, IS31FL3236A_LED35_PWM_ADDR},
};

/*********************************************************************
 * @fn      led_display_setLEDPower
 *
 * @brief   call this function to set brightness (power) of the LED display during light On and light Off status
 *
 * @param   light_status
 *
 * @return  None
 *********************************************************************/
void led_display_setLEDPower(uint8_t ledPower)
{
    // I2C command to set LED Power
    ledSetpower = ledPower;
}

/*********************************************************************
 * @fn      led_display_changeLEDPower
 *
 * @brief   call this function to change brightness (power) of the LED display during light On and light Off status
 *
 * @param   None
 *
 * @return  None
 *********************************************************************/
void led_display_changeLEDPower()
{
    if(ledPower_old != ledSetpower){
        if(ledSetpower){
            ledBrightness = PWM_LOW;
        }
        else{
            ledBrightness = PWM_CUSTOM;
        }
        ledPower_old = ledSetpower;
    }
}

/*********************************************************************
 * @fn      led_display_Init
 *
 * @brief   It is used to initialize the library
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
void led_display_init()
{
    // At Start-Up/initiation, lit all LED on Dashboard. Then, turn off ones that are not required to be on.
    // initialize ledPower
    // set all led lights on at Power On ->
    reset_led_driver();
    turn_on_led_driver();
    enable_channels();

//    code = 0;
    ledLightMode_old = 0xFF;
    ledBLESelect_old = 0xFF;
    ledUnitSelect_old = 0xFF;
    ledSpeedModeSelect_old = 0xFF;
    ledSpeed_old = 0xFF;
    led_controlLaw_ledStatus = 0xFF;
    led_controlLaw_old = 0xFF;
    ledPower_old = 0xFF;
    ledBrightness = PWM_CUSTOM;
    led_error_code_old = 0xFF;
    led_error_priority = SYSTEM_NORMAL_PRIORITY;  // 0 is the most critical error priority - 254 is the least critical error priority - 255 is when everything is normal.
    ledBatteryStatus_old = 0xFF;
    battery_bar1_status = 1;
    BLE_flash_status = 1;

//    led_display_setAllOn();   // for testing without General Purpose Timer task active
}

/*********************************************************************
 * @fn      led_display_deinit
 *
 * @brief   It is used to initialize the library
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
void led_display_deinit()
{
    //reset_led_driver();
    //disable_channels();
    turn_off_led_driver();
}

/*******************      Basic Operation       **********************
 *
 *
 *********************************************************************/
//void led_display_i2c2Display( uint8_t Param, uint8_t *ParamValue )
//{
//    uint8_t led_display_readBuffer = NULL;                       // local variables are not stored in memory. This is equivalent to reset each time this function is called.
//    uint8_t led_display_writeBuffer[2] = {0, 0};
//    size_t  readBufferSize  = 0;
//    size_t  writeBufferSize = 2;
//
//    led_display_i2cTransferStatus = led_display_ledDisplayManager -> led_display_transfer(IS31FL3236A_ADDR, &led_display_writeBuffer, writeBufferSize, &led_display_readBuffer, readBufferSize);
//
//}

/*********************************************************************
 * @fn      led_display_setAllOn
 *
 * @brief   call this function to lit all LED on display
 *
 * @param   Nil
 *
 * @return  Nil
 *********************************************************************/
void led_display_setAllOn()
{
    /* I2C to command lit all LED on Dashboard */
//    led_display_i2c2Display( 0, NULL );
    if (led_allOn == 0)
    {
        LED_Turn_ON_ALL();
        led_allOn = 1;
    }
}
/*********************************************************************
 * @fn      led_display_setAllOff
 *
 * @brief   call this function to turn off all LED on display
 *
 * @param   Nil
 *
 * @return  Nil
 *********************************************************************/
void led_display_setAllOff(){
    /* I2C to command turn off all LED on Dashboard */
    LED_Turn_OFF_ALL();

}

/*********************************************************************
 * @fn      led_display_setDashSpeed
 *
 * @brief   call this function to set Speed indicator on LED display
 *
 * @param   dashSpeed
 *
 * @return  none
 *********************************************************************/
void led_display_setDashSpeed(uint8_t dashSpeed){
    /* I2C command to set Speed Indicator */
    ledSpeed = dashSpeed;
}

/*********************************************************************
 * @fn      led_display_changeDashSpeed
 *
 * @brief   Updates the dashboard Speed on LED display
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
uint8_t ledSpeed_counter = 0;  // added to alternate between speed and Brake error code
uint8_t brake_error_on = 0; // added
uint8_t ledSpeed_maxCount = 12; // added
uint8_t led_LC_on = 0; // added
void led_display_changeDashSpeed()
{
    if (led_error_code_old >= BRAKE_ERROR_PRIORITY) {
        if ((led_error_code_old > BRAKE_ERROR_PRIORITY) || (!brake_error_on)) {// added to alternate between speed and Brake error code
            if (!speedmodeIsLocked){  //Chee added 20250110
                if(ledSpeed_old != ledSpeed || ledBrightness != ledBrightness_old || led_LC_on) {
                    int dashspeed_ones;
                    int dashspeed_tens;
                    dashspeed_ones = ledSpeed % 10;
                    dashspeed_tens = ledSpeed / 10;
                    if(dashspeed_tens == 0){
                        IS31FL3236A_Digit_1_off(I_OUT);
                    }
                    else {
                        functionTable[35 + dashspeed_tens](I_OUT,ledBrightness);
                    }
                    functionTable[45 + dashspeed_ones](I_OUT,ledBrightness);
                    ledSpeed_old = ledSpeed;
                    led_LC_on = 0; // added
                }
            }  //Chee added 20250110
            else {  //Chee added 20250110
                led_display_speedModeLockStatus();  //Chee added 20250110
                speedmodeIsLocked = 0;  //Chee added 20250110
                led_LC_on = 1;          // Chee added 20250110
            }  //Chee added 20250110
        }
        else if ((led_error_code_old == BRAKE_ERROR_PRIORITY) && (brake_error_on)) {// added to alternate between speed and Brake error code
            functionTable[35](I_OUT,ledBrightness);//0 // added
            functionTable[60](I_OUT,ledBrightness);//E // added
        } // added

        // controls the timing when the led displays the speed or the error code when minor error is present
        if ((led_error_code_old >= BRAKE_ERROR_PRIORITY) && (led_error_code_old < BATTERY_CRITICALLY_LOW_PRIORITY)) {
            ledSpeed_counter++;  // added to alternate between speed and Brake error code
            if (ledSpeed_counter >= ledSpeed_maxCount) { // added to alternate between speed and Brake error code
                ledSpeed_counter = 0; // added
                if (brake_error_on == 0) {// added
                    brake_error_on = 1; // added
                    ledSpeed_maxCount = LEDSPEED_LOWCOUNT; // added
                } // added
                else {// added
                    brake_error_on = 0; // added
                    ledSpeed_maxCount = LEDSPEED_HIGHCOUNT; // added
                } // added
            } // added
        }
    }
}

/*********************************************************************
 * @fn      led_display_setBatteryStatus
 *
 * @brief   call this function to set battery bar on LED display
 *
 * @param   batteryStatus
 *
 * @return  none
 *********************************************************************/
void led_display_setBatteryStatus(uint8_t batteryStatus)
{
    /* I2C command to set Battery Status */
    ledBatteryStatus = batteryStatus;
}

/*********************************************************************
 * @fn      led_display_changeBatteryStatus
 *
 * @brief   Updates the  battery status on LED display
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
void led_display_changeBatteryStatus(uint32_t eventcounter)
{
    /*  Enables flashing when battery status is critically low (batteryStatus = 0) */
    if ((ledBatteryStatus == 0) && (eventcounter % 3 == 0))
    {
        if(battery_bar1_status == 1)
        {
            functionTable[33](I_OUT,ledBrightness);     //turn on battery bar 1
            battery_bar1_status = 0;                    //flash control bit
        }
        else if(battery_bar1_status == 0)
        {
            functionTable[33](I_OUT,PWM_ZERO);          //turn off battery bar 1
            battery_bar1_status = 1;                    //flash control bit
        }
    }
    else if((ledBatteryStatus_old != ledBatteryStatus) || (ledBrightness != ledBrightness_old))
    {
        if(ledBatteryStatus == 1)
        {
            functionTable[33](I_OUT,ledBrightness);
        }
        else if(ledBatteryStatus == 2)
        {
            functionTable[32](I_OUT,ledBrightness);
        }
        else if(ledBatteryStatus == 3)
        {
            functionTable[31](I_OUT,ledBrightness);
        }
        else if(ledBatteryStatus == 4)
        {
            functionTable[30](I_OUT,ledBrightness);
        }
        else if(ledBatteryStatus == 5)
        {
            functionTable[29](I_OUT,ledBrightness);
        }

    }

    ledBatteryStatus_old = ledBatteryStatus;

}

/*********************************************************************
 * @fn      led_display_setSpeedMode
 *
 * @brief   call this function to set speed mode on LED display
 *
 * @param   speedMode
 *
 * @return  none
 *********************************************************************/
void led_display_setSpeedMode(uint8_t speedMode)
{
    /*  I2C command to set Speed Mode Status */
    ledSpeedMode = speedMode;
}

/*********************************************************************
 * @fn      led_display_changeSpeedMode
 *
 * @brief   Updates the Speed Mode status on LED display
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
void led_display_changeSpeedMode(uint32_t eventcounter)
{
    if ((!led_controlLaw) && (eventcounter % 3 == 0)) // gpt_counter % 5 == 0   // if not normal law -> flashing speed mode indicator
    {
        if (led_controlLaw_ledStatus)   // turn off speed mode led indicator
        {
            /*  change speed mode    */
            if(ledSpeedMode == 0){
                functionTable[3](I_OUT,PWM_ZERO);
            }
            else if(ledSpeedMode == 1){
                functionTable[2](I_OUT,PWM_ZERO);
            }
            else{
                functionTable[1](I_OUT,PWM_ZERO);
            }

            led_controlLaw_ledStatus = 0;

        }
        else    // turn on speed mode led indicator
        {
            /*  change speed mode    */
            if(ledSpeedMode == 0){
                functionTable[3](I_OUT,ledBrightness);
            }
            else if(ledSpeedMode == 1){
                functionTable[2](I_OUT,ledBrightness);
            }
            else{
                functionTable[1](I_OUT,ledBrightness);
            }

            led_controlLaw_ledStatus = 1;

        }
        led_controlLaw_old = led_controlLaw;
    }


    if (led_controlLaw)
    {
        if ( ledSpeedModeSelect_old != ledSpeedMode || ledBrightness != ledBrightness_old || led_controlLaw_old != led_controlLaw)
        {
            /*  change speed mode    */
            if(ledSpeedMode == 0){
                functionTable[3](I_OUT,ledBrightness);
            }
            else if(ledSpeedMode == 1){
                functionTable[2](I_OUT,ledBrightness);
            }
            else{
                functionTable[1](I_OUT,ledBrightness);
            }

            ledSpeedModeSelect_old = ledSpeedMode;
            led_controlLaw_old = led_controlLaw;
        }
    }
}
/*********************************************************************
 * @fn      led_display_setUnitSelectDash
 *
 * @brief   call this function to set Dashboard Display Unit on LED display
 *
 * @param   UnitSelectDash
 *
 * @return  none
 *********************************************************************/
void led_display_setUnitSelectDash(uint8_t UnitSelectDash)
{
    // I2C command to set Unit Light
    ledUnitSelectDash = UnitSelectDash;
}

/*********************************************************************
 * @fn      led_display_changeUnit
 *
 * @brief   Updates the Unit selection on LED display
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
void led_display_changeUnit()
{
    if ( ledUnitSelect_old != ledUnitSelectDash || ledBrightness!=ledBrightness_old )
    {
        /*  change unit  */
        if(ledUnitSelectDash==0){
            functionTable[14](I_OUT,ledBrightness);
        }
        else{
            functionTable[13](I_OUT,ledBrightness);
        }

        ledUnitSelect_old = ledUnitSelectDash;

    }
}

/*********************************************************************
 * @fn      led_display_changeBLE
 *
 * @brief   Updates the BLE status on LED display
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
uint8_t *ptr_led_opcode;
uint8_t *ptr_led_advertiseFlag;
//uint32_t *ptr_led_gpt_counter;

void led_display_changeBLE(uint32_t eventcounter)
{
    /******  if advertiseFlag == 1 -> flash BLE indicator.  BLE indicator light is flashing   ****/

    /******  if opcode == connected or link update -> BLE indicator light is solid (ledBLEStatus = 1  ****/
    if((*ptr_led_opcode == GAP_LINK_ESTABLISHED_EVENT) || ( *ptr_led_opcode == GAP_LINK_PARAM_UPDATE_EVENT))
    {
        ledBLEStatus = 1;
    }
    else if(*ptr_led_advertiseFlag)
    {
        ledBLEStatus = 2;
    }
    else
    {
        ledBLEStatus = 0;
    }

    if ((ledBLEStatus == 2) && (eventcounter % 3 == 0))  // (eventcounter % 3 == 0) determines the frequency of flashing
    {
        if(BLE_flash_status == 1)
                {
                    functionTable[4](I_OUT,ledBrightness);     //turn on BLE
                    BLE_flash_status = 0;                    //flash control bit
                }
        else if(BLE_flash_status == 0)
                {
                    functionTable[4](I_OUT,PWM_ZERO);          //turn off BLE
                    BLE_flash_status = 1;                    //flash control bit
                }
    }
    else if ( ledBLESelect_old != ledBLEStatus || ledBrightness != ledBrightness_old )
    {
        // change BLE
        if(ledBLEStatus == 1)
        {
            functionTable[4](I_OUT,ledBrightness);
        }
        else if(ledBLEStatus == 0)
        {
            functionTable[4](I_OUT,PWM_ZERO);
        }

        ledBLESelect_old = ledBLEStatus;

    }

}

/*********************************************************************
 * @fn      led_display_ErrorPriority
 *
 * @brief   this function is called by other functions to sort error priority
 *
 * @param   error_priority
 *
 * @return  none
 *********************************************************************/
uint8_t led_display_ErrorPriority(uint8_t error_priority)
{
    /*  stores the most critical error priority */
    if(error_priority < led_error_code_old){
        led_error_priority = error_priority;

    }
    return (led_error_priority);
}

/*********************************************************************
 * @fn      led_display_ErrorDisplay
 *
 * @brief   call this function to display error codes on Digit 1 and Digit 2 of LED display and warning icon of LED display
 *
 * @param   None
 *
 * @return  None
 *********************************************************************/
uint8_t led_display_ErrorDisplay()
{
    if(ledBrightness != ledBrightness_old || led_error_priority != led_error_code_old){
        /******************* Led display 2 digit error code **********************/
        if((led_error_priority == UART_OPEN_NULL) || (led_error_priority == ADC1_OPEN_NULL)||
                (led_error_priority == ADC2_OPEN_NULL) || (led_error_priority == PWM1_OPEN_NULL) ||
                (led_error_priority == PWM2_OPEN_NULL) || (led_error_priority == I2C_OPEN_NULL))            // UART/ADC/PWM/I2C error code = 0F
        {
            functionTable[35](I_OUT,ledBrightness);//0
            functionTable[62](I_OUT,ledBrightness);//F
        }
        else if((led_error_priority == BATTERY_VOLTAGE_ERROR_PRIORITY) || (led_error_priority == BATTERY_TEMP_ERROR_PRIORITY)) // Battery over-voltage and over-temperature error code = 1A
        {
            functionTable[36](I_OUT,ledBrightness);//1
            functionTable[56](I_OUT,ledBrightness);//A
        }
        else if(led_error_priority == BMS_COMM_ERROR_PRIORITY)   // BMS Communication error code = 1C
        {
            functionTable[36](I_OUT,ledBrightness);//1
            functionTable[58](I_OUT,ledBrightness);//C
        }
        else if(led_error_priority == GATE_DRIVER_ERROR_PRIORITY) // MCU Gate Driver error code = 2C
        {
            functionTable[37](I_OUT,ledBrightness);//2
            functionTable[58](I_OUT,ledBrightness);//C
        }
        else if(led_error_priority == MOSFET_ERROR_PRIORITY)     // MOSFET error code = 2E
        {
            functionTable[37](I_OUT,ledBrightness);//2
            functionTable[60](I_OUT,ledBrightness);//E
        }
        else if(led_error_priority == PHASE_I_ERROR_PRIORITY)    // MCU abnormal phase current error code = 2A
        {
            functionTable[37](I_OUT,ledBrightness);//2
            functionTable[56](I_OUT,ledBrightness);//A
        }
        else if(led_error_priority == CONTROLLER_TEMP_ERROR_PRIORITY) // MCU over-temperature error code = 2F
        {
            functionTable[37](I_OUT,ledBrightness);//2
            functionTable[62](I_OUT,ledBrightness);//F
        }
        else if(led_error_priority == HALL_SENSOR_ERROR_PRIORITY) // Motor Hall Sensor error code = 3A
        {
            functionTable[38](I_OUT,ledBrightness);//3
            functionTable[56](I_OUT,ledBrightness);//A
        }
        else if(led_error_priority == MOTOR_TEMP_ERROR_PRIORITY)// Motor over-temperature error code = 3C
        {
            functionTable[38](I_OUT,ledBrightness);//3
            functionTable[58](I_OUT,ledBrightness);//C
        }
        else if(led_error_priority == DASH_COMM_ERROR_PRIORITY) // Dashboard and MCU communication error code = 0A
        {
            functionTable[35](I_OUT,ledBrightness);//0
            functionTable[56](I_OUT,ledBrightness);//A
        }
        else if(led_error_priority == THROTTLE_ERROR_PRIORITY)  // Throttle error code = 0C
        {
            functionTable[35](I_OUT,ledBrightness);//0
            functionTable[58](I_OUT,ledBrightness);//C
        }
//        else if(led_error_priority == BRAKE_ERROR_PRIORITY)     // Brake throttle error code = 0E
//        {
//            functionTable[35](I_OUT,ledBrightness);//0
//            functionTable[60](I_OUT,ledBrightness);//E
//        }
//        else if(led_error_priority == SOFTWARE_ERROR_PRIORITY)  // software error code = 4A
//        {
//            functionTable[39](I_OUT,ledBrightness);//4
//            functionTable[56](I_OUT,ledBrightness);//A
//        }

        /*********** led display warning light icon ******************/
        if (led_error_priority < BATTERY_CRITICALLY_LOW_PRIORITY) // if led_error_priority is less than 0x20 (32), warning light on
        {
            functionTable[7](I_OUT,ledBrightness); // attention light
        }

        led_error_code_old = led_error_priority;

    }
    return (led_error_priority);
}

/*********************************************************************
 * @fn      led_display_setLightMode
 *
 * @brief   call this function to set light mode on LED display
 *
 * @param   light_mode
 *
 * @return  None
 *********************************************************************/
void led_display_setLightMode(uint8_t LED_light_mode)
{
    /*  I2C command to set Light Mode    */
    ledLightMode = LED_light_mode;
}

/*********************************************************************
 * @fn      led_display_changeLightMode
 *
 * @brief   Updates the light mode status on LED display
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
void led_display_changeLightMode()
{
    if ((ledLightMode_old != ledLightMode) || (ledBrightness != ledBrightness_old))
    {
        // change Light mode
        if(ledLightMode==0)
        {
            functionTable[5](I_OUT,PWM_ZERO);       // light icon off
            functionTable[6](I_OUT,PWM_ZERO);       // auto icon off
        }
        else if(ledLightMode==1)
        {
            functionTable[5](I_OUT,ledBrightness);  // light icon on
//            functionTable[6](I_OUT,PWM_ZERO);
        }
        else
        {
            functionTable[5](I_OUT,ledBrightness);  // light icon on
            functionTable[6](I_OUT,ledBrightness);  // auto icon on
        }

        ledLightMode_old = ledLightMode;
    }
}
/*********************************************************************
 * @fn      led_display_setLightStatus
 *
 * @brief   call this function to set light status on LED display
 *
 * @param   light_status
 *
 * @return  None
 *********************************************************************/
void led_display_setLightStatus(uint8_t light_status)
{
    /*  I2C command to set Light Status  */
    ledLightStatus = light_status;
}

/*********************************************************************
 * @fn      led_display_changeLightStatus
 *
 * @brief   Updates the light status on LED display
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
void led_display_changeLightStatus()
{
    if ( ledLightStatus_old != ledLightStatus  || ledBrightness!=ledBrightness_old)
    {
        /*  change Light status  */
        if(ledLightStatus==0)
        {
            functionTable[5](I_OUT,PWM_ZERO);
        }
        else
        {
            functionTable[5](I_OUT,ledBrightness);
        }

        ledLightStatus_old = ledLightStatus;
        ledBrightness_old = ledBrightness;
    }
}

/*********************************************************************
 * @fn      led_display_speedModeLockStatus
 *
 * @brief   Display "LC" on LED display
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
static void led_display_speedModeLockStatus()  //Chee added 20250110
{  //Chee added 20250110
    functionTable[67](I_OUT,ledBrightness);//L //   //Chee added 20250110
    functionTable[58](I_OUT,ledBrightness);//C //   //Chee added 20250110
}

/*********************************************************************
 * @fn      led_display_registerLedDisplay
 *
 * @brief   Register for LED display
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
void led_display_registerLedDisplay( led_display_ledDisplayManager_t *ledDisplayI2C )
{
    led_display_ledDisplayManager = ledDisplayI2C;
}

/*********************************************************************
 * @fn      led_control_setControlLaw
 *
 * @brief   this function receives the updated control law value
 *
 * @param   controlLaw
 *
 * @return  none
 *********************************************************************/
extern void led_control_setControlLaw(uint8_t controlLaw)
{
    led_controlLaw = controlLaw;
}


extern void* led_display_errorPriorityRegister()
{
    return (&led_error_priority);
}
/*********************************************************************
 * @fn      led_display_registerOpcode
 *
 * @brief   Register for opcode
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
extern void led_display_opcodeRegister( uint8_t *ptr_opcode )
{
    ptr_led_opcode = ptr_opcode;
}

/*********************************************************************
 * @fn      led_display_advertiseFlagRegister
 *
 * @brief   Register for sp_advertiseFlag
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
extern void led_display_advertiseFlagRegister(uint8_t *ptr_advertiseFlag)
{
    ptr_led_advertiseFlag = ptr_advertiseFlag;
}

/*********************************************************************
 * @fn      led_display_speedmodeIsLockRegister
 *
 * @brief   Register for speedmodeIsLock
 *
 * @param   none
 *
 * @return  speedmodeIsLocked
 *********************************************************************/
extern void* led_display_speedmodeIsLockRegister()  //Chee added 20250110
{  //Chee added 20250110
    return (&speedmodeIsLocked);  //Chee added 20250110
}  //Chee added 20250110
