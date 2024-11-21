/*
 * ESCOOTER_BOOT.h
 *
 *  Created on: 31 Jul 2024
 *      Author: TerenceLeung
 */

#ifndef HARDWARE_ESCOOTER_BOOT_H_
#define HARDWARE_ESCOOTER_BOOT_H_


#ifdef _cplusplus
extern "C"
{
#endif

#include<stdint.h>

/*Define Boot Process Parameters*/
#define BOOTREADY_TIME      300
#define BOOTREADY_COUNT     2

extern void PowerModeStatusManager();
extern void BootCountDownCreate();
extern void BootCountDownInit();
extern void BootCountStart();
extern void BootCountStop();
extern void BootCountEliminate();
extern void BootCountOVFxn();
extern void Boot_Init();
extern uint8_t HowToBoot();
extern void BootServiceRoutine(uint8_t logicLevel);
extern void sendBootMessage();
extern void SystemShutDownRoutine();



#ifdef _cplusplus
}
#endif



#endif /* HARDWARE_ESCOOTER_BOOT_H_ */
