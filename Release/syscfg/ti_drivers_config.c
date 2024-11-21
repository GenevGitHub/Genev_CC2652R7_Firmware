/*
 *  ======== ti_drivers_config.c ========
 *  Configured TI-Drivers module definitions
 *
 *  DO NOT EDIT - This file is generated for the CC2652R7RGZ
 *  by the SysConfig tool.
 */

#include <stddef.h>
#include <stdint.h>

#ifndef DeviceFamily_CC26X2X7
#define DeviceFamily_CC26X2X7
#endif

#include <ti/devices/DeviceFamily.h>

#include "ti_drivers_config.h"

/*
 *  =============================== ADC ===============================
 */

#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCCC26XX.h>
#include <ti/drivers/GPIO.h>

#define CONFIG_ADC_COUNT 2

/*
 *  ======== adcCC26xxObjects ========
 */
ADCCC26XX_Object adcCC26xxObjects[CONFIG_ADC_COUNT];

/*
 *  ======== adcCC26xxHWAttrs ========
 */
const ADCCC26XX_HWAttrs adcCC26xxHWAttrs[CONFIG_ADC_COUNT] = {
    /* CONFIG_ADC_THR */
    {
        .adcDIO              = CONFIG_GPIO_ADC_THR_AIN,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO6,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .refVoltage          = 3300000,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    /* CONFIG_ADC_BRK */
    {
        .adcDIO              = CONFIG_GPIO_ADC_BRK_AIN,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO7,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .refVoltage          = 3300000,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
};

/*
 *  ======== ADC_config ========
 */
const ADC_Config ADC_config[CONFIG_ADC_COUNT] = {
    /* CONFIG_ADC_THR */
    {
        .fxnTablePtr = &ADCCC26XX_fxnTable,
        .object = &adcCC26xxObjects[CONFIG_ADC_THR],
        .hwAttrs = &adcCC26xxHWAttrs[CONFIG_ADC_THR]
    },
    /* CONFIG_ADC_BRK */
    {
        .fxnTablePtr = &ADCCC26XX_fxnTable,
        .object = &adcCC26xxObjects[CONFIG_ADC_BRK],
        .hwAttrs = &adcCC26xxHWAttrs[CONFIG_ADC_BRK]
    },
};

const uint_least8_t CONFIG_ADC_THR_CONST = CONFIG_ADC_THR;
const uint_least8_t CONFIG_ADC_BRK_CONST = CONFIG_ADC_BRK;
const uint_least8_t ADC_count = CONFIG_ADC_COUNT;

/*
 *  =============================== AESCCM ===============================
 */

#include <ti/drivers/AESCCM.h>
#include <ti/drivers/aesccm/AESCCMCC26XX.h>

#define CONFIG_AESCCM_COUNT 1
AESCCMCC26XX_Object aesccmCC26XXObjects[CONFIG_AESCCM_COUNT];

/*
 *  ======== aesccmCC26XXHWAttrs ========
 */
const AESCCMCC26XX_HWAttrs aesccmCC26XXHWAttrs[CONFIG_AESCCM_COUNT] = {
    {
        .intPriority = (~0),
    },
};

const AESCCM_Config AESCCM_config[CONFIG_AESCCM_COUNT] = {
    {   /* CONFIG_AESCCM0 */
        .object  = &aesccmCC26XXObjects[CONFIG_AESCCM0],
        .hwAttrs = &aesccmCC26XXHWAttrs[CONFIG_AESCCM0]
    },
};

const uint_least8_t CONFIG_AESCCM0_CONST = CONFIG_AESCCM0;
const uint_least8_t AESCCM_count = CONFIG_AESCCM_COUNT;

/*
 *  =============================== AESCTRDRBG ===============================
 */

#include <ti/drivers/AESCTRDRBG.h>
#include <ti/drivers/aesctrdrbg/AESCTRDRBGXX.h>

#define CONFIG_AESCTRDRBG_COUNT 1

/*
 *  ======== aesctrdrbgXXObjects ========
 */
AESCTRDRBGXX_Object aesctrdrbgXXObjects[CONFIG_AESCTRDRBG_COUNT];

/*
 *  ======== aesctrdrbgXXHWAttrs ========
 */
const AESCTRDRBGXX_HWAttrs aesctrdrbgXXHWAttrs[CONFIG_AESCTRDRBG_COUNT] = {
    /* CONFIG_AESCTRDRBG_0 */
    {
        .aesctrHWAttrs.intPriority = (~0)
    },
};

/*
 *  ======== AESCTRDRBG_config ========
 */
const AESCTRDRBG_Config AESCTRDRBG_config[CONFIG_AESCTRDRBG_COUNT] = {
    /* CONFIG_AESCTRDRBG_0 */
    {
        .object = &aesctrdrbgXXObjects[CONFIG_AESCTRDRBG_0],
        .hwAttrs = &aesctrdrbgXXHWAttrs[CONFIG_AESCTRDRBG_0]
    },
};

const uint_least8_t CONFIG_AESCTRDRBG_0_CONST = CONFIG_AESCTRDRBG_0;
const uint_least8_t AESCTRDRBG_count = CONFIG_AESCTRDRBG_COUNT;

/*
 *  =============================== AESECB ===============================
 */

#include <ti/drivers/AESECB.h>
#include <ti/drivers/aesecb/AESECBCC26XX.h>

#define CONFIG_AESECB_COUNT 1


AESECBCC26XX_Object aesecbCC26XXObjects[CONFIG_AESECB_COUNT];

/*
 *  ======== aesecbCC26XXHWAttrs ========
 */
const AESECBCC26XX_HWAttrs aesecbCC26XXHWAttrs[CONFIG_AESECB_COUNT] = {
    {
        .intPriority = (~0),
    },
};

const AESECB_Config AESECB_config[CONFIG_AESECB_COUNT] = {
    {   /* CONFIG_AESECB0 */
        .object  = &aesecbCC26XXObjects[CONFIG_AESECB0],
        .hwAttrs = &aesecbCC26XXHWAttrs[CONFIG_AESECB0]
    },
};


const uint_least8_t CONFIG_AESECB0_CONST = CONFIG_AESECB0;
const uint_least8_t AESECB_count = CONFIG_AESECB_COUNT;

/*
 *  =============================== DMA ===============================
 */

#include <ti/drivers/dma/UDMACC26XX.h>
#include <ti/devices/cc13x2x7_cc26x2x7/driverlib/udma.h>
#include <ti/devices/cc13x2x7_cc26x2x7/inc/hw_memmap.h>

UDMACC26XX_Object udmaCC26XXObject;

const UDMACC26XX_HWAttrs udmaCC26XXHWAttrs = {
    .baseAddr        = UDMA0_BASE,
    .powerMngrId     = PowerCC26XX_PERIPH_UDMA,
    .intNum          = INT_DMA_ERR,
    .intPriority     = (~0)
};

const UDMACC26XX_Config UDMACC26XX_config[1] = {
    {
        .object         = &udmaCC26XXObject,
        .hwAttrs        = &udmaCC26XXHWAttrs,
    },
};

/*
 *  =============================== ECDH ===============================
 */

#include <ti/drivers/ECDH.h>
#include <ti/drivers/ecdh/ECDHCC26X2.h>

#define CONFIG_ECDH_COUNT 1


ECDHCC26X2_Object ecdhCC26X2Objects[CONFIG_ECDH_COUNT];

/*
 *  ======== ecdhCC26X2HWAttrs ========
 */
const ECDHCC26X2_HWAttrs ecdhCC26X2HWAttrs[CONFIG_ECDH_COUNT] = {
    {
        .intPriority = (~0),
    },
};

const ECDH_Config ECDH_config[CONFIG_ECDH_COUNT] = {
    {   /* CONFIG_ECDH0 */
        .object         = &ecdhCC26X2Objects[CONFIG_ECDH0],
        .hwAttrs        = &ecdhCC26X2HWAttrs[CONFIG_ECDH0]
    },
};

const uint_least8_t CONFIG_ECDH0_CONST = CONFIG_ECDH0;
const uint_least8_t ECDH_count = CONFIG_ECDH_COUNT;

/*
 *  =============================== GPIO ===============================
 */

#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOCC26XX.h>

/* The range of pins available on this device */
const uint_least8_t GPIO_pinLowerBound = 0;
const uint_least8_t GPIO_pinUpperBound = 30;

/*
 *  ======== gpioPinConfigs ========
 *  Array of Pin configurations
 */
GPIO_PinConfig gpioPinConfigs[31] = {
    GPIO_CFG_NO_DIR, /* DIO_0 */
    GPIO_CFG_NO_DIR, /* DIO_1 */
    GPIO_CFG_NO_DIR, /* DIO_2 */
    GPIO_CFG_NO_DIR, /* DIO_3 */
    GPIO_CFG_NO_DIR, /* DIO_4 */
    GPIO_CFG_NO_DIR, /* DIO_5 */
    GPIO_CFG_NO_DIR, /* DIO_6 */
    GPIO_CFG_NO_DIR, /* DIO_7 */
    /* Owned by CONFIG_UART2_1 as TX */
    GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH, /* CONFIG_GPIO_UART2_1_TX */
    /* Owned by CONFIG_UART2_1 as RX */
    GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_NONE | GPIO_CFG_PULL_DOWN_INTERNAL, /* CONFIG_GPIO_UART2_1_RX */
    GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_BOTH_EDGES | GPIO_CFG_PULL_NONE_INTERNAL, /* CONFIG_GPIO_MPB */
    /* Owned by CONFIG_GPTIMER_0 as PWM Pin */
    GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_LOW, /* CONFIG_GPIO_PWM_0 */
    /* Owned by CONFIG_GPTIMER_2 as PWM Pin */
    GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_LOW, /* CONFIG_GPIO_PWM_2 */
    /* Owned by CONFIG_GPTIMER_1 as PWM Pin */
    GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_LOW, /* CONFIG_GPIO_PWM_1 */
    GPIO_CFG_NO_DIR, /* DIO_14 */
    GPIO_CFG_NO_DIR, /* DIO_15 */
    GPIO_CFG_NO_DIR, /* DIO_16 */
    GPIO_CFG_NO_DIR, /* DIO_17 */
    /* Owned by CONFIG_I2C as SCL */
    GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_NONE | GPIO_CFG_PULL_UP_INTERNAL, /* CONFIG_GPIO_I2C_SCL */
    /* Owned by CONFIG_I2C as SDA */
    GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_NONE | GPIO_CFG_PULL_UP_INTERNAL, /* CONFIG_GPIO_I2C_SDA */
    GPIO_CFG_NO_DIR, /* DIO_20 */
    GPIO_CFG_NO_DIR, /* DIO_21 */
    GPIO_CFG_NO_DIR, /* DIO_22 */
    /* Owned by CONFIG_ADC_BRK as ADC Pin */
    GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_NONE | GPIO_CFG_PULL_NONE_INTERNAL, /* CONFIG_GPIO_ADC_BRK_AIN */
    /* Owned by CONFIG_ADC_THR as ADC Pin */
    GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_NONE | GPIO_CFG_PULL_NONE_INTERNAL, /* CONFIG_GPIO_ADC_THR_AIN */
    GPIO_CFG_NO_DIR, /* DIO_25 */
    GPIO_CFG_NO_DIR, /* DIO_26 */
    GPIO_CFG_NO_DIR, /* DIO_27 */
    GPIO_CFG_NO_DIR, /* DIO_28 */
    GPIO_CFG_NO_DIR, /* DIO_29 */
    GPIO_CFG_NO_DIR, /* DIO_30 */
};

/*
 *  ======== gpioCallbackFunctions ========
 *  Array of callback function pointers
 *  Change at runtime with GPIO_setCallback()
 */
GPIO_CallbackFxn gpioCallbackFunctions[31];

/*
 *  ======== gpioUserArgs ========
 *  Array of user argument pointers
 *  Change at runtime with GPIO_setUserArg()
 *  Get values with GPIO_getUserArg()
 */
void* gpioUserArgs[31];

const uint_least8_t CONFIG_GPIO_ADC_THR_AIN_CONST = CONFIG_GPIO_ADC_THR_AIN;
const uint_least8_t CONFIG_GPIO_ADC_BRK_AIN_CONST = CONFIG_GPIO_ADC_BRK_AIN;
const uint_least8_t CONFIG_GPIO_MPB_CONST = CONFIG_GPIO_MPB;
const uint_least8_t CONFIG_GPIO_I2C_SDA_CONST = CONFIG_GPIO_I2C_SDA;
const uint_least8_t CONFIG_GPIO_I2C_SCL_CONST = CONFIG_GPIO_I2C_SCL;
const uint_least8_t CONFIG_GPIO_PWM_0_CONST = CONFIG_GPIO_PWM_0;
const uint_least8_t CONFIG_GPIO_PWM_1_CONST = CONFIG_GPIO_PWM_1;
const uint_least8_t CONFIG_GPIO_PWM_2_CONST = CONFIG_GPIO_PWM_2;
const uint_least8_t CONFIG_GPIO_UART2_1_TX_CONST = CONFIG_GPIO_UART2_1_TX;
const uint_least8_t CONFIG_GPIO_UART2_1_RX_CONST = CONFIG_GPIO_UART2_1_RX;

/*
 *  ======== GPIO_config ========
 */
const GPIO_Config GPIO_config = {
    .configs = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .userArgs = gpioUserArgs,
    .intPriority = (~0)
};

/*
 *  =============================== I2C ===============================
 */

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/power/PowerCC26XX.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(driverlib/ioc.h)

#define CONFIG_I2C_COUNT 1

/*
 *  ======== i2cCC26xxObjects ========
 */
I2CCC26XX_Object i2cCC26xxObjects[CONFIG_I2C_COUNT];

/*
 *  ======== i2cCC26xxHWAttrs ========
 */
const I2CCC26XX_HWAttrsV1 i2cCC26xxHWAttrs[CONFIG_I2C_COUNT] = {
    /* CONFIG_I2C */
    {
        .baseAddr    = I2C0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_I2C0,
        .intNum      = INT_I2C_IRQ,
        .intPriority = (~0),
        .swiPriority = 0,
        .sclPin      = CONFIG_GPIO_I2C_SCL,
        .sclPinMux   = IOC_PORT_MCU_I2C_MSSCL,
        .sdaPin      = CONFIG_GPIO_I2C_SDA,
        .sdaPinMux   = IOC_PORT_MCU_I2C_MSSDA
    },
};

/*
 *  ======== I2C_config ========
 */
const I2C_Config I2C_config[CONFIG_I2C_COUNT] = {
    /* CONFIG_I2C */
    {
        .object      = &i2cCC26xxObjects[CONFIG_I2C],
        .hwAttrs     = &i2cCC26xxHWAttrs[CONFIG_I2C]
    },
};

const uint_least8_t CONFIG_I2C_CONST = CONFIG_I2C;
const uint_least8_t I2C_count = CONFIG_I2C_COUNT;

/*
 *  =============================== NVS ===============================
 */

#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSCC26XX.h>

/*
 *  NVSCC26XX Internal NVS flash region definitions
 *
 * Place uninitialized char arrays at addresses
 * corresponding to the 'regionBase' addresses defined in
 * the configured NVS regions. These arrays are used as
 * place holders so that the linker will not place other
 * content there.
 *
 * For GCC targets, the char arrays are each placed into
 * the shared ".nvs" section. The user must add content to
 * their GCC linker command file to place the .nvs section
 * at the lowest 'regionBase' address specified in their NVS
 * regions.
 */

#if defined(__TI_COMPILER_VERSION__) || defined(__clang__)

static char flashBuf0[0x4000] __attribute__ ((retain, noinit, location(0x48000)));

#elif defined(__IAR_SYSTEMS_ICC__)

__no_init static char flashBuf0[0x4000] @ 0x48000;

#elif defined(__GNUC__)

__attribute__ ((section (".nvs")))
static char flashBuf0[0x4000];

#endif

NVSCC26XX_Object nvsCC26XXObjects[1];

static const NVSCC26XX_HWAttrs nvsCC26XXHWAttrs[1] = {
    /* CONFIG_NVSINTERNAL */
    {
        .regionBase = (void *) flashBuf0,
        .regionSize = 0x4000
    },
};

#define CONFIG_NVS_COUNT 1

const NVS_Config NVS_config[CONFIG_NVS_COUNT] = {
    /* CONFIG_NVSINTERNAL */
    {
        .fxnTablePtr = &NVSCC26XX_fxnTable,
        .object = &nvsCC26XXObjects[0],
        .hwAttrs = &nvsCC26XXHWAttrs[0],
    },
};

const uint_least8_t CONFIG_NVSINTERNAL_CONST = CONFIG_NVSINTERNAL;
const uint_least8_t NVS_count = CONFIG_NVS_COUNT;

/*
 *  =============================== PWM ===============================
 */

#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerCC26XX.h>

/* include driverlib definitions */
#include <ti/devices/cc13x2x7_cc26x2x7/driverlib/ioc.h>
#include <ti/devices/cc13x2x7_cc26x2x7/inc/hw_ints.h>
#include <ti/devices/cc13x2x7_cc26x2x7/inc/hw_memmap.h>

#define CONFIG_PWM_COUNT 3

/*
 *  ======== pwmCC26XXObjects ========
 */
PWMTimerCC26XX_Object pwmTimerCC26XXObjects[CONFIG_PWM_COUNT];

/*
 *  ======== pwmCC26XXHWAttrs ========
 */
const PWMTimerCC26XX_HwAttrs pwmTimerCC26XXHWAttrs[CONFIG_PWM_COUNT] = {
    /* CONFIG_PWM_HEADLIGHT */
    {
        .pwmPin = CONFIG_GPIO_PWM_0,
        .gpTimerUnit = CONFIG_GPTIMER_0
    },
    /* CONFIG_PWM_BUZZER */
    {
        .pwmPin = CONFIG_GPIO_PWM_1,
        .gpTimerUnit = CONFIG_GPTIMER_1
    },
    /* CONFIG_PWM_LIGHT1 */
    {
        .pwmPin = CONFIG_GPIO_PWM_2,
        .gpTimerUnit = CONFIG_GPTIMER_2
    },
};

/*
 *  ======== PWM_config ========
 */
const PWM_Config PWM_config[CONFIG_PWM_COUNT] = {
    /* CONFIG_PWM_HEADLIGHT */
    {
        .fxnTablePtr = &PWMTimerCC26XX_fxnTable,
        .object = &pwmTimerCC26XXObjects[CONFIG_PWM_HEADLIGHT],
        .hwAttrs = &pwmTimerCC26XXHWAttrs[CONFIG_PWM_HEADLIGHT]
    },
    /* CONFIG_PWM_BUZZER */
    {
        .fxnTablePtr = &PWMTimerCC26XX_fxnTable,
        .object = &pwmTimerCC26XXObjects[CONFIG_PWM_BUZZER],
        .hwAttrs = &pwmTimerCC26XXHWAttrs[CONFIG_PWM_BUZZER]
    },
    /* CONFIG_PWM_LIGHT1 */
    {
        .fxnTablePtr = &PWMTimerCC26XX_fxnTable,
        .object = &pwmTimerCC26XXObjects[CONFIG_PWM_LIGHT1],
        .hwAttrs = &pwmTimerCC26XXHWAttrs[CONFIG_PWM_LIGHT1]
    },
};

const uint_least8_t CONFIG_PWM_HEADLIGHT_CONST = CONFIG_PWM_HEADLIGHT;
const uint_least8_t CONFIG_PWM_BUZZER_CONST = CONFIG_PWM_BUZZER;
const uint_least8_t CONFIG_PWM_LIGHT1_CONST = CONFIG_PWM_LIGHT1;
const uint_least8_t PWM_count = CONFIG_PWM_COUNT;

/*
 *  =============================== Power ===============================
 */
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26X2.h>
#include "ti_drivers_config.h"

extern void PowerCC26XX_standbyPolicy(void);
extern bool PowerCC26XX_calibrate(unsigned int);

const PowerCC26X2_Config PowerCC26X2_config = {
    .enablePolicy             = true,
    .policyInitFxn            = NULL,
    .policyFxn                = PowerCC26XX_standbyPolicy,
    .calibrateFxn             = PowerCC26XX_calibrate,
    .calibrateRCOSC_LF        = true,
    .calibrateRCOSC_HF        = true,
    .enableTCXOFxn            = NULL
};



/*
 *  =============================== RF Driver ===============================
 */
#include <ti/drivers/GPIO.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/ioc.h)
#include <ti/drivers/rf/RF.h>

/*
 * Platform-specific driver configuration
 */
const RFCC26XX_HWAttrsV2 RFCC26XX_hwAttrs = {
    .hwiPriority        = (~0),
    .swiPriority        = (uint8_t)0,
    .xoscHfAlwaysNeeded = true,
    .globalCallback     = NULL,
    .globalEventMask    = 0
};


/*
 *  =============================== TRNG ===============================
 */

#include <ti/drivers/TRNG.h>
#include <ti/drivers/trng/TRNGCC26XX.h>

#define CONFIG_TRNG_COUNT 1


TRNGCC26XX_Object trngCC26XXObjects[CONFIG_TRNG_COUNT];

/*
 *  ======== trngCC26XXHWAttrs ========
 */
static const TRNGCC26XX_HWAttrs trngCC26XXHWAttrs[CONFIG_TRNG_COUNT] = {
    {
        .intPriority = (~0),
        .swiPriority = 0,
        .samplesPerCycle = 240000
    },
};

const TRNG_Config TRNG_config[CONFIG_TRNG_COUNT] = {
    {   /* CONFIG_TRNG_0 */
        .object         = &trngCC26XXObjects[CONFIG_TRNG_0],
        .hwAttrs        = &trngCC26XXHWAttrs[CONFIG_TRNG_0]
    },
};

const uint_least8_t CONFIG_TRNG_0_CONST = CONFIG_TRNG_0;
const uint_least8_t TRNG_count = CONFIG_TRNG_COUNT;

/*
 *  =============================== UART2 ===============================
 */

#include <ti/drivers/UART2.h>
#include <ti/drivers/uart2/UART2CC26X2.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/dma/UDMACC26XX.h>
#include <ti/drivers/power/PowerCC26X2.h>
#include <ti/devices/cc13x2x7_cc26x2x7/driverlib/ioc.h>
#include <ti/devices/cc13x2x7_cc26x2x7/inc/hw_memmap.h>
#include <ti/devices/cc13x2x7_cc26x2x7/inc/hw_ints.h>

#define CONFIG_UART2_COUNT 1

UART2CC26X2_Object uart2CC26X2Objects[CONFIG_UART2_COUNT];

static unsigned char uart2RxRingBuffer0[32];
/* TX ring buffer allocated to be used for nonblocking mode */
static unsigned char uart2TxRingBuffer0[32];

ALLOCATE_CONTROL_TABLE_ENTRY(dmaUart1RxControlTableEntry, UDMA_CHAN_UART1_RX);
ALLOCATE_CONTROL_TABLE_ENTRY(dmaUart1TxControlTableEntry, UDMA_CHAN_UART1_TX);

static const UART2CC26X2_HWAttrs uart2CC26X2HWAttrs[CONFIG_UART2_COUNT] = {
  {
    .baseAddr           = UART1_BASE,
    .intNum             = INT_UART1_COMB,
    .intPriority        = (~0),
    .rxPin              = CONFIG_GPIO_UART2_1_RX,
    .txPin              = CONFIG_GPIO_UART2_1_TX,
    .ctsPin             = GPIO_INVALID_INDEX,
    .rtsPin             = GPIO_INVALID_INDEX,
    .flowControl        = UART2_FLOWCTRL_NONE,
    .powerId            = PowerCC26XX_PERIPH_UART1,
    .rxBufPtr           = uart2RxRingBuffer0,
    .rxBufSize          = sizeof(uart2RxRingBuffer0),
    .txBufPtr           = uart2TxRingBuffer0,
    .txBufSize          = sizeof(uart2TxRingBuffer0),
    .txPinMux           = IOC_PORT_MCU_UART1_TX,
    .rxPinMux           = IOC_PORT_MCU_UART1_RX,
    .ctsPinMux          = IOC_PORT_MCU_UART1_CTS,
    .rtsPinMux          = IOC_PORT_MCU_UART1_RTS,
    .dmaTxTableEntryPri = &dmaUart1TxControlTableEntry,
    .dmaRxTableEntryPri = &dmaUart1RxControlTableEntry,
    .rxChannelMask      = 1 << UDMA_CHAN_UART1_RX,
    .txChannelMask      = 1 << UDMA_CHAN_UART1_TX,
    .txIntFifoThr       = UART2CC26X2_FIFO_THRESHOLD_1_8,
    .rxIntFifoThr       = UART2CC26X2_FIFO_THRESHOLD_4_8
  },
};

const UART2_Config UART2_config[CONFIG_UART2_COUNT] = {
    {   /* CONFIG_UART2_1 */
        .object      = &uart2CC26X2Objects[CONFIG_UART2_1],
        .hwAttrs     = &uart2CC26X2HWAttrs[CONFIG_UART2_1]
    },
};

const uint_least8_t CONFIG_UART2_1_CONST = CONFIG_UART2_1;
const uint_least8_t UART2_count = CONFIG_UART2_COUNT;


/*
 *  =============================== GPTimer ===============================
 */

#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/devices/cc13x2x7_cc26x2x7/inc/hw_memmap.h>
#include <ti/devices/cc13x2x7_cc26x2x7/inc/hw_ints.h>

#define CONFIG_GPTIMER_COUNT 3

/*
 *  ======== gptimerCC26XXObjects ========
 */
GPTimerCC26XX_Object gptimerCC26XXObjects[CONFIG_GPTIMER_COUNT];

/*
 *  ======== gptimerCC26XXHWAttrs ========
 */
const GPTimerCC26XX_HWAttrs gptimerCC26XXHWAttrs[CONFIG_GPTIMER_COUNT] = {
    /* CONFIG_GPTIMER_0, used by CONFIG_PWM_HEADLIGHT */
    {
        .baseAddr = GPT0_BASE,
        .intNum      = INT_GPT0A,
        .intPriority = (~0),
        .powerMngrId = PowerCC26XX_PERIPH_GPT0,
        .pinMux      = GPT_PIN_0A
    },
    /* CONFIG_GPTIMER_1, used by CONFIG_PWM_BUZZER */
    {
        .baseAddr = GPT0_BASE,
        .intNum      = INT_GPT0B,
        .intPriority = (~0),
        .powerMngrId = PowerCC26XX_PERIPH_GPT0,
        .pinMux      = GPT_PIN_0B
    },
    /* CONFIG_GPTIMER_2, used by CONFIG_PWM_LIGHT1 */
    {
        .baseAddr = GPT1_BASE,
        .intNum      = INT_GPT1A,
        .intPriority = (~0),
        .powerMngrId = PowerCC26XX_PERIPH_GPT1,
        .pinMux      = GPT_PIN_1A
    },
};

/*
 *  ======== GPTimer_config ========
 */
const GPTimerCC26XX_Config GPTimerCC26XX_config[CONFIG_GPTIMER_COUNT] = {
    /* CONFIG_GPTIMER_0 */
    {
        .object    = &gptimerCC26XXObjects[CONFIG_GPTIMER_0],
        .hwAttrs   = &gptimerCC26XXHWAttrs[CONFIG_GPTIMER_0],
        .timerPart = GPT_A
    },
    /* CONFIG_GPTIMER_1 */
    {
        .object    = &gptimerCC26XXObjects[CONFIG_GPTIMER_1],
        .hwAttrs   = &gptimerCC26XXHWAttrs[CONFIG_GPTIMER_1],
        .timerPart = GPT_B
    },
    /* CONFIG_GPTIMER_2 */
    {
        .object    = &gptimerCC26XXObjects[CONFIG_GPTIMER_2],
        .hwAttrs   = &gptimerCC26XXHWAttrs[CONFIG_GPTIMER_2],
        .timerPart = GPT_A
    },
};

const uint_least8_t CONFIG_GPTIMER_0_CONST = CONFIG_GPTIMER_0;
const uint_least8_t CONFIG_GPTIMER_1_CONST = CONFIG_GPTIMER_1;
const uint_least8_t CONFIG_GPTIMER_2_CONST = CONFIG_GPTIMER_2;
const uint_least8_t GPTimer_count = CONFIG_GPTIMER_COUNT;

#include <stdbool.h>

#include <ti/devices/cc13x2x7_cc26x2x7/driverlib/ioc.h>
#include <ti/devices/cc13x2x7_cc26x2x7/driverlib/cpu.h>

#include <ti/drivers/GPIO.h>

/* Board GPIO defines */
#define BOARD_EXT_FLASH_SPI_CS      20
#define BOARD_EXT_FLASH_SPI_CLK     10
#define BOARD_EXT_FLASH_SPI_PICO    9
#define BOARD_EXT_FLASH_SPI_POCI    8


/*
 *  ======== Board_sendExtFlashByte ========
 */
void Board_sendExtFlashByte(uint8_t byte)
{
    uint8_t i;

    /* SPI Flash CS */
    GPIO_write(BOARD_EXT_FLASH_SPI_CS, 0);

    for (i = 0; i < 8; i++) {
        GPIO_write(BOARD_EXT_FLASH_SPI_CLK, 0); /* SPI Flash CLK */

        /* SPI Flash PICO */
        GPIO_write(BOARD_EXT_FLASH_SPI_PICO, (byte >> (7 - i)) & 0x01);
        GPIO_write(BOARD_EXT_FLASH_SPI_CLK, 1);  /* SPI Flash CLK */

        /*
         * Waste a few cycles to keep the CLK high for at
         * least 45% of the period.
         * 3 cycles per loop: 8 loops @ 48 Mhz = 0.5 us.
         */
        CPUdelay(8);
    }

    GPIO_write(BOARD_EXT_FLASH_SPI_CLK, 0);  /* CLK */
    GPIO_write(BOARD_EXT_FLASH_SPI_CS, 1);  /* CS */

    /*
     * Keep CS high at least 40 us
     * 3 cycles per loop: 700 loops @ 48 Mhz ~= 44 us
     */
    CPUdelay(700);
}

/*
 *  ======== Board_wakeUpExtFlash ========
 */
void Board_wakeUpExtFlash(void)
{
    /* SPI Flash CS*/
    GPIO_setConfig(BOARD_EXT_FLASH_SPI_CS, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_HIGH | GPIO_CFG_OUT_STR_MED);

    /*
     *  To wake up we need to toggle the chip select at
     *  least 20 ns and ten wait at least 35 us.
     */

    /* Toggle chip select for ~20ns to wake ext. flash */
    GPIO_write(BOARD_EXT_FLASH_SPI_CS, 0);
    /* 3 cycles per loop: 1 loop @ 48 Mhz ~= 62 ns */
    CPUdelay(1);
    GPIO_write(BOARD_EXT_FLASH_SPI_CS, 1);
    /* 3 cycles per loop: 560 loops @ 48 Mhz ~= 35 us */
    CPUdelay(560);
}

/*
 *  ======== Board_shutDownExtFlash ========
 */
void Board_shutDownExtFlash(void)
{
    /*
     *  To be sure we are putting the flash into sleep and not waking it,
     *  we first have to make a wake up call
     */
    Board_wakeUpExtFlash();

    /* SPI Flash CS*/
    GPIO_setConfig(BOARD_EXT_FLASH_SPI_CS, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_HIGH | GPIO_CFG_OUT_STR_MED);
    /* SPI Flash CLK */
    GPIO_setConfig(BOARD_EXT_FLASH_SPI_CLK, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW | GPIO_CFG_OUT_STR_MED);
    /* SPI Flash PICO */
    GPIO_setConfig(BOARD_EXT_FLASH_SPI_PICO, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW | GPIO_CFG_OUT_STR_MED);
    /* SPI Flash POCI */
    GPIO_setConfig(BOARD_EXT_FLASH_SPI_POCI, GPIO_CFG_IN_PD);

    uint8_t extFlashShutdown = 0xB9;

    Board_sendExtFlashByte(extFlashShutdown);

    GPIO_resetConfig(BOARD_EXT_FLASH_SPI_CS);
    GPIO_resetConfig(BOARD_EXT_FLASH_SPI_CLK);
    GPIO_resetConfig(BOARD_EXT_FLASH_SPI_PICO);
    GPIO_resetConfig(BOARD_EXT_FLASH_SPI_POCI);
}


#include <ti/drivers/Board.h>

/*
 *  ======== Board_initHook ========
 *  Perform any board-specific initialization needed at startup.  This
 *  function is declared weak to allow applications to override it if needed.
 */
void __attribute__((weak)) Board_initHook(void)
{
}

/*
 *  ======== Board_init ========
 *  Perform any initialization needed before using any board APIs
 */
void Board_init(void)
{
    /* ==== /ti/drivers/Power initialization ==== */
    Power_init();

    /* ==== /ti/devices/CCFG initialization ==== */

    /* ==== /ti/drivers/GPIO initialization ==== */
    /* Setup GPIO module and default-initialise pins */
    GPIO_init();

    /* ==== /ti/drivers/RF initialization ==== */


    Board_shutDownExtFlash();

    Board_initHook();
}

