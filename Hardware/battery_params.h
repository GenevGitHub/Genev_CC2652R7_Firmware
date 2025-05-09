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
#define BATTERY_40700mV_S11P2           1
// Battery Options
// 1: BATTERY_48100mV_S13P2
// 2: BATTERY_44400mV_S12P2
// 3: BATTERY_40700mV_S11P3
// 4: BATTERY_40700mV_S11P2
// 5: BATTERY_37000mV_S10P2

/*********************************************************************************************
 *  Battery Status Constants
 *********************************************************************************************/
#define BATTERY_PERCENTAGE_INITIAL      100              // Remove this if calculated using start up voltage
#define BATTERY_PERCENTAGE_LL           5               // battery low percentage at which buzzer alert starts
#define BATTERY_PERCENTAGE_LH           8               // battery low percentage at which buzzer alert stops

#define BATTERY_OVERVOLTAGE             0x02
#define BATTERY_UNDERVOLTAGE            0x01
#define BATTERY_VOLTAGENORMAL           0xFF

/* Battery specification parameters     48.1V */
#ifdef BATTERY_48100mV_S13P2
#define BATTERY_CEILING_VOLTAGE         55510
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_NOMINAL_VOLTAGE         48100
#define BATTERY_MAX_VOLTAGE             54210
#define BATTERY_MIN_VOLTAGE             42900           // the battery voltage assumed at 0% charged
#define BATTERY_CRITICALLY_LOW          42000
#define BATTERY_MAX_CAPACITY            461760          // mW-hr
#define LEVEL45                         52513           // equivalent to 80%
#define LEVEL34                         50251           // equivalent to 63%
#define LEVEL23                         47989           // equivalent to 45%
#define LEVEL12                         45727           // equivalent to 28%
#define LEVEL01                         43465           // equivalent to  0%
#define LEVEL00                         42900
/** VOLTAGE_DROP_COEFFICIENT is determined experimentally via battery tests **/
#define VOLTAGE_DROP_COEFFICIENT        0.357
#endif // BATTERY_S13P2

/* Battery specification parameters     44.4V */
#ifdef BATTERY_44400mV_S12P2
#define BATTERY_CEILING_VOLTAGE         51300
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_NOMINAL_VOLTAGE         44400
#define BATTERY_MAX_VOLTAGE             50040
#define BATTERY_MIN_VOLTAGE             39600           // the battery voltage assumed at 0% charged
#define BATTERY_CRITICALLY_LOW          38800
#define BATTERY_MAX_CAPACITY            426240          // mW-hr
#define LEVEL45                         48474           // equivalent to 80%
#define LEVEL34                         46386           // equivalent to 63%
#define LEVEL23                         44298           // equivalent to 45%
#define LEVEL12                         42210           // equivalent to 28%
#define LEVEL01                         40122           // equivalent to  0%
#define LEVEL00                         39600
/** VOLTAGE_DROP_COEFFICIENT is determined experimentally via battery tests **/
#define VOLTAGE_DROP_COEFFICIENT        0.329
#endif // BATTERY_S12P2


/* Battery specification parameters     40.7V */
#ifdef BATTERY_40700mV_S11P3
#define BATTERY_CEILING_VOLTAGE         47000
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_NOMINAL_VOLTAGE         40700
#define BATTERY_MAX_VOLTAGE             45870
#define BATTERY_MIN_VOLTAGE             36300           // the battery voltage assumed at 0% charged
#define BATTERY_CRITICALLY_LOW          35500
#define BATTERY_MAX_CAPACITY            586080          // mW-hr
#define LEVEL45                         44434           // equivalent to 80%
#define LEVEL34                         42520           // equivalent to 63%
#define LEVEL23                         40606           // equivalent to 45%
#define LEVEL12                         38692           // equivalent to 28%
#define LEVEL01                         36778           // equivalent to  0%
#define LEVEL00                         36300
/** VOLTAGE_DROP_COEFFICIENT is determined experimentally via battery tests **/
#define VOLTAGE_DROP_COEFFICIENT        0.201
#endif // BATTERY_S11P3


/* Battery specification parameters     40.7V */
#ifdef BATTERY_40700mV_S11P2
#define BATTERY_CEILING_VOLTAGE         47000
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_NOMINAL_VOLTAGE         40700           // 11 x 3.7V
#define BATTERY_MAX_VOLTAGE             45870           // 11 x 4.17V
#define BATTERY_MIN_VOLTAGE             36300//34000           // the battery voltage assumed at 0% charged
#define BATTERY_CRITICALLY_LOW          35500//32000
#define BATTERY_MAX_CAPACITY            390720          // mW-hr
#define LEVEL45                         44434//43500           // equivalent to 80%
#define LEVEL34                         42520//41480           // equivalent to 63%
#define LEVEL23                         4066//39300           // equivalent to 45%
#define LEVEL12                         38692//37300           // equivalent to 28%
#define LEVEL01                         36778//34000           // equivalent to  0%
#define LEVEL00                         36300
/** VOLTAGE_DROP_COEFFICIENT is determined experimentally via battery tests **/
#define VOLTAGE_DROP_COEFFICIENT        0.301
#endif // BATTERY_S11P2

/* Battery specification parameters     37.0V */
#ifdef BATTERY_37000mV_S10P2
#define BATTERY_CEILING_VOLTAGE         42700
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_NOMINAL_VOLTAGE         37000           // 10 x 3.7V
#define BATTERY_MAX_VOLTAGE             41700           // 10 x 4.17V
#define BATTERY_MIN_VOLTAGE             33000           // the battery voltage assumed at 0% charged
#define BATTERY_CRITICALLY_LOW          32300
#define BATTERY_MAX_CAPACITY            355200          // mW-hr
#define LEVEL45                         40395           // equivalent to 80%
#define LEVEL34                         38655           // equivalent to 63%
#define LEVEL23                         36915           // equivalent to 45%
#define LEVEL12                         35175           // equivalent to 28%
#define LEVEL01                         33435           // equivalent to  0%
#define LEVEL00                         33000
/** VOLTAGE_DROP_COEFFICIENT is determined experimentally via battery tests **/
#define VOLTAGE_DROP_COEFFICIENT        0.274           // for Cest Power 37V 9.6Ah 10S2P Battery pack Voltage drop empirical coefficient
#endif // BATTERY_S10P2

#define LEVEL45PERCENT                  85              // 80%
#define LEVEL34PERCENT                  65              // 63%
#define LEVEL23PERCENT                  45              // 45%
#define LEVEL12PERCENT                  25              // 28%
#define LEVEL01PERCENT                  0               // 0%


#ifdef _cplusplus
}
#endif

#endif /* HARDWARE_BATTERY_PARAMS_H_ */
