/*
 * battery_params.h
 *
 *  Created on: 13 Jun 2024
 *      Author: Chee
 */

#ifndef HARDWARE_BATTERY_PARAMS_H_
#define HARDWARE_BATTERY_PARAMS_H_

#ifdef _cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

/*********************************************************************************************
 *  Battery Options
 *********************************************************************************************/
#undef BATTERY_48100mV_S13P2
#undef BATTERY_44400mV_S12P2
#undef BATTERY_40700mV_S11P3
#define BATTERY_40700mV_S11P2           1
#undef  BATTERY_37000mV_S10P2

/*********************************************************************************************
 *  Battery Status Constants
 *********************************************************************************************/
#define BATTERY_PERCENTAGE_INITIAL      100             // Remove this if calculated using start up voltage
#define BATTERY_PERCENTAGE_LL           5               // battery low percentage at which buzzer alert starts
#define BATTERY_PERCENTAGE_LH           8               // battery low percentage at which buzzer alert stops


/* Battery specification parameters     48.1V */
#ifdef BATTERY_48100mV_S13P2
#define BATTERY_CEILING_VOLTAGE         55510
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_NOMINAL_VOLTAGE         48100
#define BATTERY_MAX_VOLTAGE             54600
#define BATTERY_MIN_VOLTAGE             39000           // the battery voltage assumed at 0% charged
#define BATTERY_CRITICALLY_LOW          36400
#define BATTERY_MAX_CAPACITY            461760          // mW-hr
#define LEVEL45                         51480           // equivalent to 80%
#define LEVEL34                         48828           // equivalent to 63%
#define LEVEL23                         46020           // equivalent to 45%
#define LEVEL12                         43368           // equivalent to 28%
#define LEVEL01                         39000           // equivalent to  0%
#endif // BATTERY_S13P2

/* Battery specification parameters     44.4V */
#ifdef BATTERY_44400mV_S12P2
#define BATTERY_CEILING_VOLTAGE         51300
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_NOMINAL_VOLTAGE         44400
#define BATTERY_MAX_VOLTAGE             50400
#define BATTERY_MIN_VOLTAGE             36000           // the battery voltage assumed at 0% charged
#define BATTERY_CRITICALLY_LOW          33600
#define BATTERY_MAX_CAPACITY            426240          // mW-hr
#define LEVEL45                         47520           // equivalent to 80%
#define LEVEL34                         45072           // equivalent to 63%
#define LEVEL23                         42480           // equivalent to 45%
#define LEVEL12                         40032           // equivalent to 28%
#define LEVEL01                         36000           // equivalent to  0%
#endif // BATTERY_S12P2


/* Battery specification parameters     40.7V */
#ifdef BATTERY_40700mV_S11P3
#define BATTERY_CEILING_VOLTAGE         47000
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_NOMINAL_VOLTAGE         40700
#define BATTERY_MAX_VOLTAGE             46200
#define BATTERY_MIN_VOLTAGE             33000           // the battery voltage assumed at 0% charged
#define BATTERY_CRITICALLY_LOW          30000
#define BATTERY_MAX_CAPACITY            586080          // mW-hr
#define LEVEL45                         43560           // equivalent to 80%
#define LEVEL34                         41316           // equivalent to 63%
#define LEVEL23                         38940           // equivalent to 45%
#define LEVEL12                         36696           // equivalent to 28%
#define LEVEL01                         33000           // equivalent to  0%
#endif // BATTERY_S11P2


/* Battery specification parameters     40.7V */
#ifdef BATTERY_40700mV_S11P2
#define BATTERY_CEILING_VOLTAGE         47000
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_NOMINAL_VOLTAGE         40700
#define BATTERY_MAX_VOLTAGE             46200
#define BATTERY_MIN_VOLTAGE             33000           // the battery voltage assumed at 0% charged
#define BATTERY_CRITICALLY_LOW          30000
#define BATTERY_MAX_CAPACITY            390720          // mW-hr
#define LEVEL45                         43560           // equivalent to 80%
#define LEVEL34                         41316           // equivalent to 63%
#define LEVEL23                         38940           // equivalent to 45%
#define LEVEL12                         36696           // equivalent to 28%
#define LEVEL01                         33000           // equivalent to  0%
#endif // BATTERY_S11P2

/* Battery specification parameters     37.0V */
#ifdef BATTERY_37000mV_S10P2
#define BATTERY_CEILING_VOLTAGE         42700
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_NOMINAL_VOLTAGE         37000           // 10 x 3.7V
#define BATTERY_MAX_VOLTAGE             42000
#define BATTERY_MIN_VOLTAGE             30000           // the battery voltage assumed at 0% charged
#define BATTERY_CRITICALLY_LOW          26000
#define BATTERY_MAX_CAPACITY            355200          // mW-hr
#define LEVEL45                         39600           // equivalent to 80%
#define LEVEL34                         37560           // equivalent to 63%
#define LEVEL23                         35400           // equivalent to 45%
#define LEVEL12                         33360           // equivalent to 28%
#define LEVEL01                         30000           // equivalent to  0%
#endif // BATTERY_S10P2

#define VOLTAGE_DROP_COEFFICIENT        0.269           // for Cest Power 37V 9.6Ah Battery pack Voltage drop empirical coefficient

#define LEVEL45PERCENT                  80              // 80%
#define LEVEL34PERCENT                  63              // 63%
#define LEVEL23PERCENT                  45              // 45%
#define LEVEL12PERCENT                  28              // 28%
#define LEVEL01PERCENT                  0               // 0%


#ifdef _cplusplus
}
#endif

#endif /* HARDWARE_BATTERY_PARAMS_H_ */
