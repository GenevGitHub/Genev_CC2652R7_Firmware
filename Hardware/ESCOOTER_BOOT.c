/*
 * ESCOOTER_BOOT.c
 *
 *  Created on: 31 Jul 2024
 *      Author: TerenceLeung
 */
#include "Hardware/ESCOOTER_BOOT.h"
#include "Hardware/STM32MCP.h"

#include <ti/drivers/GPIO.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26X2.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_prcm.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)
#include DeviceFamily_constructPath(driverlib/pwr_ctrl.h)

#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/Error.h>

#include "ti_drivers_config.h"

uint8_t bootReady = 0x00;
uint8_t howToBoot = 0xFF;

static Clock_Handle bootCountClockHandle;
static Clock_Params clkParams;
static uint32_t bootClockTicks;
static Error_Block eb;

/*Power Notify Objects*/
Power_NotifyObj powerNotifyObj;

// Application Power_NotifyFxn function prototype
static int powerTransitionNotifyFxn(unsigned int eventType, uintptr_t eventArg,uintptr_t clientArg);

static int powerTransitionNotifyFxn(unsigned int eventType, uintptr_t eventArg,uintptr_t clientArg)
{
    uint_fast16_t powerTransition;
    switch(eventType)
    {
        case PowerCC26XX_ENTERING_SHUTDOWN:
            powerTransition = Power_getTransitionState();
            break;

        case PowerCC26XX_ENTERING_STANDBY:
            powerTransition = Power_getTransitionState();
            break;

        case PowerCC26XX_AWAKE_STANDBY:
            powerTransition = Power_getTransitionState();
            break;

        case PowerCC26XX_AWAKE_STANDBY_LATE:
            powerTransition = Power_getTransitionState();
            break;

        default:
            break;
    }
    return Power_NOTIFYDONE;
}

void PowerModeStatusManager()
{
    uint8_t event_List = PowerCC26XX_ENTERING_SHUTDOWN | PowerCC26XX_ENTERING_STANDBY | PowerCC26XX_AWAKE_STANDBY | PowerCC26XX_AWAKE_STANDBY_LATE;
    Power_registerNotify(&powerNotifyObj, event_List, powerTransitionNotifyFxn, 0);
}

void BootCountDownCreate()
{
    Error_init(&eb);
    bootClockTicks = BOOTREADY_TIME * (1000 / Clock_tickPeriod) - 1; // BOOTREADY_TIME to overflow
    //Create BootCountDown clock !!
    bootCountClockHandle = Clock_create(BootCountOVFxn, bootClockTicks, &clkParams, &eb);
}

void BootCountDownInit()
{
    Clock_Params_init(&clkParams);
    clkParams.period = bootClockTicks;
    clkParams.startFlag = FALSE;
    clkParams.arg = (UArg)0x0000;
    Clock_setTimeout(bootCountClockHandle, bootClockTicks);
    Clock_setPeriod(bootCountClockHandle, bootClockTicks);
}

void BootCountStart()
{
    //Set the initial timeout
    Clock_start(bootCountClockHandle);
}

void BootCountStop()
{
    Clock_stop(bootCountClockHandle);
}

void BootCountEliminate()
{
    Clock_delete(&bootCountClockHandle);
}

void BootCountOVFxn()
{
    bootReady++;
}
uint16_t shutdownCounter = 0;

void Boot_Init()
{
    PowerCC26X2_ResetReason resetReason = PowerCC26X2_getResetReason();
    PowerCC26X2_releaseLatches();    //@pre PowerCC26X2_getResetReason()

    switch (resetReason)
    {
    case (PowerCC26X2_RESET_SHUTDOWN_IO):
    {
        howToBoot = 0x00;
        BootCountDownCreate();
        BootCountDownInit();
        GPIO_setConfig(CONFIG_GPIO_MPB_CONST, GPIO_CFG_IN_PU);  // Pull-up when not pressed.  Pull-down when pressed.
        uint8_t press = GPIO_read(CONFIG_GPIO_MPB_CONST);
        BootServiceRoutine(press);
        BootCountEliminate();
        GPIO_resetConfig(CONFIG_GPIO_MPB_CONST);
        break;
    }
    case (PowerCC26X2_RESET_TCK_NOISE):
    {
        howToBoot = 0x01;
        break;
    }
    case (PowerCC26X2_RESET_SYSTEM):
    {
        howToBoot = 0x02;
        break;
    }
    case (PowerCC26X2_RESET_WARM_RESET):
    {
        howToBoot = 0x03;
        break;
    }
    case (PowerCC26X2_RESET_CLK):
    {
        howToBoot = 0x04;
        break;
    }
    case (PowerCC26X2_RESET_VDDR):
    {
        howToBoot = 0x05;
        break;
    }
    case (PowerCC26X2_RESET_VDDS):
    {
        howToBoot = 0x06;
        break;
    }
    case (PowerCC26X2_RESET_PIN):
    {
        howToBoot = 0x07;
        break;
    }
    case (PowerCC26X2_RESET_POR):
    {
        howToBoot = 0x08;
#ifdef  MOTOR_CONNECT
        BootCountDownCreate();
        BootCountDownInit();
        GPIO_setConfig(CONFIG_GPIO_MPB_CONST, GPIO_CFG_IN_PU);
        uint8_t press = GPIO_read(CONFIG_GPIO_MPB_CONST);
        BootServiceRoutine(press);
        BootCountEliminate();
        GPIO_resetConfig(CONFIG_GPIO_MPB_CONST);
#endif // MOTOR_CONNECT
        break;
    }
    default:
        howToBoot = 0x09;
        /*  We should not get here because there should not be any other possible cases  */
        break;
    }

}

uint8_t HowToBoot()
{
    return howToBoot;
}

void BootServiceRoutine(uint8_t isWakeUp)
{
   uint8_t BOOT_FINISHED = 0x00;
   if(isWakeUp == 0)    // when button is pressed...
   {
         /*If user is still pressing the button, the Count Down timer starts immediately */
         uint8_t stillPress = GPIO_read(CONFIG_GPIO_MPB_CONST);
         if(stillPress == 0x00)
         {
             /*Starts Counting Down for BOOTREADY_TIME x BOOTREADY_COUNT seconds*/
             BootCountStart();
         }
         /*We haven't done booting yet! Although you are pressing the button! Don't put down your hand!*/
         while(BOOT_FINISHED == 0x00)
         {
             /* status = 0x01 --> release the button
              * status = 0x00 --> press the button
              */
             uint8_t status = GPIO_read(CONFIG_GPIO_MPB_CONST);
             /*if Button is released prematurely --> go back to Shut down mode*/
             if(status == 0x01)
             {
                 /*Count-Down Timer Stops*/
                 BootCountStop();
                 /*Enters Shut Down Mode again*/
                 SystemShutDownRoutine();
                 /* POWER_ON = 0  */
             }
             /*If the user is still pressing the button until TIME'S UP, the count down timer stops! Boot Routine is finished!  */
             /*For each 500 ms timer overflow, bootCount increases by 1.
              *To have 1.5 seconds delay, bootReady is 3.
              *To see the blink, wait until the bootReady is 3 while you are pressing the button!
              * */
             if(bootReady >= BOOTREADY_COUNT)
             {
                 /*Count-Down finished*/
                 BootCountStop();
                 /*Congratulations! BOOT Process is Done ! Get out of the loop! Enjoy driving!  */
                 BOOT_FINISHED = 0x01;
                 /*  POWER_ON = 1  */
             }
         }
   }
   else if(isWakeUp == 1)   // when button not pressed -> system shut down
   {
       while(BOOT_FINISHED == 0x00)
       {
           /*Enters Shut Down Mode again*/
           SystemShutDownRoutine();
           /* POWER_ON = 0  */
       }
   }
}

void sendBootMessage()
{
    STM32MCP_controlEscooterBehavior(ESCOOTER_BOOT_ACK);
}

void SystemShutDownRoutine()
{
    GPIO_resetConfig(CONFIG_GPIO_MPB_CONST);
    /* How to wake me up??  */
    GPIO_setConfig(CONFIG_GPIO_MPB_CONST,GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_PULL_UP_INTERNAL | GPIO_CFG_SHUTDOWN_WAKE_LOW);
    /* Power shutdown Sleep  */
    Power_shutdown(0, 0);
    /*  Once power shutdown, anything else that follows are not executed, such as this While loop below.  */
    while(1)    /*  This While will not be Ran  */
    {
        shutdownCounter++;
    }
}




