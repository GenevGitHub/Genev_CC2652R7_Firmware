/*
 *  ======== ti_drivers_config.h ========
 *  Configured TI-Drivers module declarations
 *
 *  The macros defines herein are intended for use by applications which
 *  directly include this header. These macros should NOT be hard coded or
 *  copied into library source code.
 *
 *  Symbols declared as const are intended for use with libraries.
 *  Library source code must extern the correct symbol--which is resolved
 *  when the application is linked.
 *
 *  DO NOT EDIT - This file is generated for the CC2652R7RGZ
 *  by the SysConfig tool.
 */
#ifndef ti_drivers_config_h
#define ti_drivers_config_h

#define CONFIG_SYSCONFIG_PREVIEW

#define CONFIG_CC2652R7RGZ
#ifndef DeviceFamily_CC26X2X7
#define DeviceFamily_CC26X2X7
#endif

#include <ti/devices/DeviceFamily.h>

#include <stdint.h>

/* support C++ sources */
#ifdef __cplusplus
extern "C" {
#endif


/*
 *  ======== CCFG ========
 */


/*
 *  ======== ADC ========
 */

/* DIO24 */
extern const uint_least8_t              CONFIG_ADC_THR_CONST;
#define CONFIG_ADC_THR                  0
/* DIO23 */
extern const uint_least8_t              CONFIG_ADC_BRK_CONST;
#define CONFIG_ADC_BRK                  1
#define CONFIG_TI_DRIVERS_ADC_COUNT     2


/*
 *  ======== AESCCM ========
 */

extern const uint_least8_t                  CONFIG_AESCCM0_CONST;
#define CONFIG_AESCCM0                      0
#define CONFIG_TI_DRIVERS_AESCCM_COUNT      1


/*
 *  ======== AESCTRDRBG ========
 */

extern const uint_least8_t                      CONFIG_AESCTRDRBG_0_CONST;
#define CONFIG_AESCTRDRBG_0                     0
#define CONFIG_TI_DRIVERS_AESCTRDRBG_COUNT      1


/*
 *  ======== AESECB ========
 */

extern const uint_least8_t                  CONFIG_AESECB0_CONST;
#define CONFIG_AESECB0                      0
#define CONFIG_TI_DRIVERS_AESECB_COUNT      1


/*
 *  ======== ECDH ========
 */

extern const uint_least8_t              CONFIG_ECDH0_CONST;
#define CONFIG_ECDH0                    0
#define CONFIG_TI_DRIVERS_ECDH_COUNT    1


/*
 *  ======== GPIO ========
 */
/* Owned by CONFIG_ADC_THR as  */
extern const uint_least8_t CONFIG_GPIO_ADC_THR_AIN_CONST;
#define CONFIG_GPIO_ADC_THR_AIN 24

/* Owned by CONFIG_ADC_BRK as  */
extern const uint_least8_t CONFIG_GPIO_ADC_BRK_AIN_CONST;
#define CONFIG_GPIO_ADC_BRK_AIN 23

extern const uint_least8_t CONFIG_GPIO_MPB_CONST;
#define CONFIG_GPIO_MPB 10

/* Owned by CONFIG_I2C as  */
extern const uint_least8_t CONFIG_GPIO_I2C_SDA_CONST;
#define CONFIG_GPIO_I2C_SDA 19

/* Owned by CONFIG_I2C as  */
extern const uint_least8_t CONFIG_GPIO_I2C_SCL_CONST;
#define CONFIG_GPIO_I2C_SCL 18

/* Owned by CONFIG_GPTIMER_0 as  */
extern const uint_least8_t CONFIG_GPIO_PWM_0_CONST;
#define CONFIG_GPIO_PWM_0 11

/* Owned by CONFIG_GPTIMER_1 as  */
extern const uint_least8_t CONFIG_GPIO_PWM_1_CONST;
#define CONFIG_GPIO_PWM_1 13

/* Owned by CONFIG_GPTIMER_2 as  */
extern const uint_least8_t CONFIG_GPIO_PWM_2_CONST;
#define CONFIG_GPIO_PWM_2 12

/* Owned by CONFIG_UART2_1 as  */
extern const uint_least8_t CONFIG_GPIO_UART2_1_TX_CONST;
#define CONFIG_GPIO_UART2_1_TX 8

/* Owned by CONFIG_UART2_1 as  */
extern const uint_least8_t CONFIG_GPIO_UART2_1_RX_CONST;
#define CONFIG_GPIO_UART2_1_RX 9

/* The range of pins available on this device */
extern const uint_least8_t GPIO_pinLowerBound;
extern const uint_least8_t GPIO_pinUpperBound;

/* LEDs are active high */
#define CONFIG_GPIO_LED_ON  (1)
#define CONFIG_GPIO_LED_OFF (0)

#define CONFIG_LED_ON  (CONFIG_GPIO_LED_ON)
#define CONFIG_LED_OFF (CONFIG_GPIO_LED_OFF)


/*
 *  ======== I2C ========
 */

/*
 *  SCL: DIO18
 *  SDA: DIO19
 */
extern const uint_least8_t              CONFIG_I2C_CONST;
#define CONFIG_I2C                      0
#define CONFIG_TI_DRIVERS_I2C_COUNT     1

/* ======== I2C Addresses and Speeds ======== */
#include <ti/drivers/I2C.h>

/* ---- CONFIG_I2C I2C bus components ---- */

/* no components connected to CONFIG_I2C */

/* max speed unspecified, defaulting to 100 kbps */
#define CONFIG_I2C_MAXSPEED   (100U) /* kbps */
#define CONFIG_I2C_MAXBITRATE ((I2C_BitRate)I2C_100kHz)


/*
 *  ======== NVS ========
 */

extern const uint_least8_t              CONFIG_NVSINTERNAL_CONST;
#define CONFIG_NVSINTERNAL              0
#define CONFIG_TI_DRIVERS_NVS_COUNT     1


/*
 *  ======== PWM ========
 */

/* DIO11 */
extern const uint_least8_t              CONFIG_PWM_HEADLIGHT_CONST;
#define CONFIG_PWM_HEADLIGHT            0
/* DIO13 */
extern const uint_least8_t              CONFIG_PWM_BUZZER_CONST;
#define CONFIG_PWM_BUZZER               1
/* DIO12 */
extern const uint_least8_t              CONFIG_PWM_LIGHT1_CONST;
#define CONFIG_PWM_LIGHT1               2
#define CONFIG_TI_DRIVERS_PWM_COUNT     3




/*
 *  ======== TRNG ========
 */

extern const uint_least8_t              CONFIG_TRNG_0_CONST;
#define CONFIG_TRNG_0                   0
#define CONFIG_TI_DRIVERS_TRNG_COUNT    1


/*
 *  ======== UART2 ========
 */

/*
 *  TX: DIO8
 *  RX: DIO9
 */
extern const uint_least8_t                  CONFIG_UART2_1_CONST;
#define CONFIG_UART2_1                      0
#define CONFIG_TI_DRIVERS_UART2_COUNT       1


/*
 *  ======== GPTimer ========
 */

extern const uint_least8_t                  CONFIG_GPTIMER_0_CONST;
#define CONFIG_GPTIMER_0                    0
extern const uint_least8_t                  CONFIG_GPTIMER_1_CONST;
#define CONFIG_GPTIMER_1                    1
extern const uint_least8_t                  CONFIG_GPTIMER_2_CONST;
#define CONFIG_GPTIMER_2                    2
#define CONFIG_TI_DRIVERS_GPTIMER_COUNT     3


/*
 *  ======== Board_init ========
 *  Perform all required TI-Drivers initialization
 *
 *  This function should be called once at a point before any use of
 *  TI-Drivers.
 */
extern void Board_init(void);

/*
 *  ======== Board_initGeneral ========
 *  (deprecated)
 *
 *  Board_initGeneral() is defined purely for backward compatibility.
 *
 *  All new code should use Board_init() to do any required TI-Drivers
 *  initialization _and_ use <Driver>_init() for only where specific drivers
 *  are explicitly referenced by the application.  <Driver>_init() functions
 *  are idempotent.
 */
#define Board_initGeneral Board_init

#ifdef __cplusplus
}
#endif

#endif /* include guard */
