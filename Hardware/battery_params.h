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
#define BATTERY_40700mV_S11P2           4
// Battery Options
// 1: BATTERY_48100mV_S13P2             1
// 2: BATTERY_44400mV_S12P2             2
// 3: BATTERY_40700mV_S11P3             3
// 4: BATTERY_40700mV_S11P2             4
// 5: BATTERY_37000mV_S10P2             5

/*********************************************************************************************
 *  Battery Status Constants
 *********************************************************************************************/
#define BATTERY_PERCENTAGE_INITIAL      100              // Remove this if calculated using start up voltage
#define BATTERY_PERCENTAGE_LL           25               // battery low percentage at which buzzer alert starts
#define BATTERY_PERCENTAGE_LH           30               // battery low percentage at which buzzer alert stops
#define SOC_INITIAL                     100
#define BATTERY_OVERVOLTAGE             0x02
#define BATTERY_UNDERVOLTAGE            0x01
#define BATTERY_VOLTAGENORMAL           0xFF

/* Battery specification parameters     48.1V */
#ifdef BATTERY_48100mV_S13P2
#define BATTERY_CEILING_VOLTAGE         54600
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_MAX_VOLTAGE             54000
#define BATTERY_NOMINAL_VOLTAGE         48100
#define BATTERY_WMAX_CAPACITY           432900        // mW-hr
#define BATTERY_IMAX_CAPACITY           9000          // mA-hr
#define DISCHARGE_CURVE_GRADIENT                  -1.2085
#define VOLTAGE_DROP_RATE               1981.6
#define VOLTAGE_DROP_0                  245.29
#endif // BATTERY_S13P2

/* Battery specification parameters     44.4V */
#ifdef BATTERY_44400mV_S12P2
#define BATTERY_CEILING_VOLTAGE         50400
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_MAX_VOLTAGE             49850
#define BATTERY_NOMINAL_VOLTAGE         44400
#define BATTERY_WMAX_CAPACITY           399600          // mW-hr
#define BATTERY_IMAX_CAPACITY           9000          // mA-hr
#define DISCHARGE_CURVE_GRADIENT        -1.1156
#define VOLTAGE_DROP_RATE               1981.6
#define VOLTAGE_DROP_0                  245.29
#endif // BATTERY_S12P2


/* Battery specification parameters     40.7V */
#ifdef BATTERY_40700mV_S11P3
#define BATTERY_CEILING_VOLTAGE         46200
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_MAX_VOLTAGE             45700           // Based on test data
#define BATTERY_NOMINAL_VOLTAGE         40700           // 11 x 3.7V
#define BATTERY_WMAX_CAPACITY           549450          // mW-hr
#define BATTERY_IMAX_CAPACITY           13500          // mA-hr
#define DISCHARGE_CURVE_GRADIENT        -0.6817
#define VOLTAGE_DROP_RATE               1981.6
#define VOLTAGE_DROP_0                  245.29
#endif // BATTERY_S11P3


/* Battery specification parameters     40.7V */
#ifdef BATTERY_40700mV_S11P2
#define BATTERY_CEILING_VOLTAGE         46200           // 4.2V x 11
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_MAX_VOLTAGE             45700           // Based on Genev test data
#define BATTERY_NOMINAL_VOLTAGE         40700           // 11 x 3.7V
#define BATTERY_WMAX_CAPACITY           366300          // mW-hr - Manufacturer data
#define BATTERY_IMAX_CAPACITY           9000            // mA-hr - Manufacturer data
#define DISCHARGE_CURVE_GRADIENT        -1.0226         // Based on Genev Test Data - gradient of mV vs mAh curve
#define VOLTAGE_DROP_RATE               1981.6         // Based on Genev Test Data - Linear Function Gradient
#define VOLTAGE_DROP_0                  245.29
#endif // BATTERY_S11P2

/* Battery specification parameters     37.0V */
#ifdef BATTERY_37000mV_S10P2
#define BATTERY_CEILING_VOLTAGE         42000
// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
#define BATTERY_MAX_VOLTAGE             41550           // 10 x 4.17V
#define BATTERY_NOMINAL_VOLTAGE         37000           // 10 x 3.7V
#define BATTERY_WMAX_CAPACITY           333000          // mW-hr
#define BATTERY_IMAX_CAPACITY           9000          // mA-hr
#define DISCHARGE_CURVE_GRADIENT        -0.9297
#define VOLTAGE_DROP_RATE               1981.6
#define VOLTAGE_DROP_0                  245.29
#endif // BATTERY_S10P2

/*  Key SoC Levels  */
#define SOC_LEVEL45                     86              // 4-5 bars
#define SOC_LEVEL_KVOLT_HIGH            80              // Kvolt high
#define SOC_LEVEL34                     72              // 3-4 bars
#define SOC_LEVEL23                     57              // 2-3 bars
#define SOC_LEVEL12                     41              // 1-2 bars
#define SOC_LEVEL01                     24              // 1 bar flashing = Battery critically low level
#define SOC_LEVEL_KVOLT_LOW             15               //

#ifdef _cplusplus
}
#endif

#endif /* HARDWARE_BATTERY_PARAMS_H_ */
