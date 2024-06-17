/*
 *  gGo_device_params.h
 *
 *  Created on: 4 May 2024
 *      Author: Chee
 */

#ifndef HARDWARE_GGO_DEVICE_PARAMS_H_
#define HARDWARE_GGO_DEVICE_PARAMS_H_


#ifdef _cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "Hardware/battery_params.h"

///*********************************************************************************************
// *  Battery Option
// *********************************************************************************************/
//#define BATTERY_40700mV_S11P2           1
//#undef  BATTERY_37000mV_S10P2

#define CC2652R7_LAUNCHXL               1

/*********************************************************************************************
 *  Regional / Regulation Options
 *********************************************************************************************/
#undef  REGION0                                     // Unregulated
#define REGION1                             1       // HK, Singapore
#undef  REGION2                                     // EU
#undef  REGION3                                     // Japan
#undef  REGION4                                     // Australia
#undef  REGION5                                     // Canada, USA

/******   CONSTANTS   *******/
#define PI_CONSTANT                         3.14159265359

/*********************************************************************************************
 *  Vehicle Information
 *********************************************************************************************/
#define WHEELRADIUS_CM                      10.16           // wheel radius in centimeter
#define COEFF01                             0.2156          // kg/km
#define COEFF02                             0.000386        // kg/W-hr
#define BCF                                 0.9             // Battery Capacity Safety Factor
#define CRIT_HEATSINKTEMPERATURE_C          90              // Critical Heatsink Temperature
#define CRIT_MOTORTEMPERATURE_C             100             // Critical Motor Temperature
/*********************************************************************************************
 *  Unit Conversion
 *********************************************************************************************/
#define KM2MILE                             0.621           // convert length from km to miles
#define CM2INCH                             0.3937           // convert length from cm to inch
#define KG2LBS                              2.205           // convert kilogram to pounds
#define SI_UNIT                             0x00
#define IMP_UNIT                            0x01

/***************************************************************************
 *          Motor Characteristics
 *************************************************************************/
#define KT_CONSTANT                         0.5895
#define KIQ_CONSTANT                        0.0008889
#define MOTOR_RESISTANCE                    0.6343
#define FRICTION_TORQUE                     0.1919
#define DAMPING_CONSTANT                    0.003443

///*********************************************************************************************
// *  Battery Status Constants
// *********************************************************************************************/
//#define BATTERY_PERCENTAGE_INITIAL      100             // Remove this if calculated using start up voltage
//#define BATTERY_PERCENTAGE_LL           5               // battery low percentage at which buzzer alert starts
//#define BATTERY_PERCENTAGE_LH           8               // battery low percentage at which buzzer alert stops
//
///* Battery specification parameters     40.7V */
//#ifdef BATTERY_40700mV_S11P2
//#define BATTERY_CEILING_VOLTAGE         47000
//// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
//#define BATTERY_NOMINAL_VOLTAGE         40700
//#define BATTERY_MAX_VOLTAGE             46200
//#define BATTERY_MIN_VOLTAGE             33000           // the battery voltage assumed at 0% charged
//#define BATTERY_CRITICALLY_LOW          30000
//#define BATTERY_MAX_CAPACITY            390720          // mW-hr
//#define LEVEL45                         43560           // equivalent to 80%
//#define LEVEL34                         41316           // equivalent to 63%
//#define LEVEL23                         38940           // equivalent to 45%
//#define LEVEL12                         36696           // equivalent to 28%
//#define LEVEL01                         33000           // equivalent to  0%
//#endif // BATTERY_S11P2
//
///* Battery specification parameters     37.0V */
//#ifdef BATTERY_37000mV_S10P2
//#define BATTERY_CEILING_VOLTAGE         42700
//// if voltage is greater than BATTERY_CEILING_VOLTAGE -> error:  incorrect battery is installed
//#define BATTERY_NOMINAL_VOLTAGE         37000           // 10 x 3.7V
//#define BATTERY_MAX_VOLTAGE             42000
//#define BATTERY_MIN_VOLTAGE             30000           // the battery voltage assumed at 0% charged
//#define BATTERY_CRITICALLY_LOW          26000
//#define BATTERY_MAX_CAPACITY            355200          // mW-hr
//#define LEVEL45                         39600           // equivalent to 80%
//#define LEVEL34                         37560           // equivalent to 63%
//#define LEVEL23                         35400           // equivalent to 45%
//#define LEVEL12                         33360           // equivalent to 28%
//#define LEVEL01                         30000           // equivalent to  0%
//#endif // BATTERY_S10P2
//
//#define VOLTAGE_DROP_COEFFICIENT        0.269           // for Cest Power 37V 9.6Ah Battery pack Voltage drop empirical coefficient
//
//#define LEVEL45PERCENT                  80              // 80%
//#define LEVEL34PERCENT                  63              // 63%
//#define LEVEL23PERCENT                  45              // 45%
//#define LEVEL12PERCENT                  28              // 28%
//#define LEVEL01PERCENT                  0               // 0%

/*********************************************************************************************
 *  Battery Status Colours
 *********************************************************************************************/
#define GLOWING_AQUA                    0x05
#define GLOWING_GREEN                   0x04
#define YELLOW                          0x03
#define ORANGE                          0x02
#define RED                             0x01
#define FLASHING_RED                    0x00
#define BATTERY_STATUS_INITIAL          0x05

/*********************************************************************************************
 *  Light Settings
 *********************************************************************************************/
#define LUXTHRESHOLD                   500  // light shall be ON when light intensity is consistently below this lux value

/*********************************************************************************************
 *  Error Codes
 *********************************************************************************************/
// Controller Error Codes
#define CONTROLLER_NORMAL                               0x00
#define PHASE_CURRENT_ABNORMAL                          0x2A
#define MOSFET_ABNORMAL                                 0x2E
//#define OPAMP_ABNORAML
#define GATE_DRIVER_ABNORMAL                            0x2C
#define HEATSINK_TEMPERATURE_ABNORMAL                   0x2F
//  Motor Error Codes
#define MOTOR_NORMAL                                    0x00
#define HALL_SENSOR_ABNORMAL                            0x3A
#define MOTOR_TEMPERATURE_ABNORMAL                      0x3C

/********************************************************************************************************************************************/
/*      error type                                error priority      Description                               error code displayed */
// system Normal
#define SYSTEM_NORMAL_PRIORITY                          0xFF    // 255: System normal
// system fatal errors
#define UART_OPEN_NULL                                  0x00    // 00:UART open failure                         (LED display error code 0F)
#define PWM1_OPEN_NULL                                  0x00    // 00:buzzer PWM failure                        (LED display error code 0F)
#define PWM2_OPEN_NULL                                  0x00    // 00:light PWM failure                         (LED display error code 0F)
#define I2C_OPEN_NULL                                   0x00    // 00:I2C open failure affects light sensor and led display (LED display error code 0F - will not display since i2c failed)
#define ADC1_OPEN_NULL                                  0x00    // 00:ADC1 open failure affects brake           (LED display error code 0F)
#define ADC2_OPEN_NULL                                  0x00    // 00:ADC2 open failure affects throttle        (LED display error code 0F)
#define SYS_FATAL_ERROR_PRIORITY                                 0x00
// Critical hardware errors
#define BATTERY_VOLTAGE_ERROR_PRIORITY                  0x01    // 1: Battery error priority                    (LED display error code 1A)
#define BATTERY_TEMP_ERROR_PRIORITY                     0x02    // 2: Battery error priority                    (LED display error code 1A)
#define BMS_COMM_ERROR_PRIORITY                         0x03    // 3: BMS communication error priority          (LED display error code 1C)
#define GATE_DRIVER_ERROR_PRIORITY                      0x04    // 4: Gate driver error priority                (LED display error code 2C)
#define MOSFET_ERROR_PRIORITY                           0x05    // 5: MOSFET error priority                     (LED display error code 2E)
#define PHASE_I_ERROR_PRIORITY                          0x06    // 6: Phase current error priority              (LED display error code 2A)
#define CONTROLLER_TEMP_ERROR_PRIORITY                  0x07    // 7: Controller temperature error priority     (LED display error code 2F)
#define HALL_SENSOR_ERROR_PRIORITY                      0x08    // 8: Hall sensor error priority                (LED display error code 3A)
#define MOTOR_TEMP_ERROR_PRIORITY                       0x09    // 9: Motor temperature error priority          (LED display error code 3C)
#define DASH_COMM_ERROR_PRIORITY                        0x0A    // 10: Dashboard communication error priority    (LED display error code 0A)
// Minor hardware/software errors
#define BRAKE_ERROR_PRIORITY                            0x10    // 16:Brake error priority                      (LED display error code 0E)
#define THROTTLE_ERROR_PRIORITY                         0x11    // 17:Throttle error priority                   (LED display error code 0C)
#define SOFTWARE_ERROR_PRIORITY                         0X12
// Warnings
#define BATTERY_CRITICALLY_LOW_WARNING                  0x20    // 32:Battery level critically low warning

/**  Error Codes Displayed **/
#define SYS_FATAL_ERROR_CODE                            0x0F
#define BATTERY_VOLTAGE_ERROR_CODE                      0x1A
#define BATTERY_TEMP_ERROR_CODE                         0x1A
#define BMS_COMM_ERROR_CODE                             0x1C
#define GATE_DRIVER_ERROR_CODE                          0x2C
#define MOSFET_ERROR_CODE                               0x2E
#define PHASE_I_ERROR_CODE                              0x2A
#define CONTROLLER_TEMP_ERROR_CODE                      0x2F
#define HALL_SENSOR_ERROR_CODE                          0x3A
#define MOTOR_TEMP_ERROR_CODE                           0x3C
#define DASH_COMM_ERROR_CODE                            0x0A
#define BRAKE_ERROR_CODE                                0x0E
#define THROTTLE_ERROR_CODE                             0x0C
#define SOFTWARE_ERROR_CODE                             0x0D

/*********************************************************************************************
 *  Regional / Regulation Settings
 *********************************************************************************************/
#ifdef REGION0
#define REG_MAXPOUT                                             65535 //Watt
#define REG_MAXP_SPEED                                          250  // km/hr
#define REG_MAXP_RPM                                            6630 // rpm
#define REG_MINP_SPEED                                          3
#define REG_MINP_RPM                                            80
//Speed mode maximum "powered" speed in RPM
#define BRAKE_AND_THROTTLE_MAXSPEED_AMBLE                       2650       // 100 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_LEISURE                     4770       // 180 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_SPORTS                      6630       // 250 Km/hr

#endif // REGION1

#ifdef REGION1
#define REG_MAXPOUT                                          300 //Watt
#define REG_MAXP_SPEED                                       25  // km/hr
#define REG_MAXP_RPM                                         663 // rpm
#define REG_MINP_SPEED                                       3
#define REG_MINP_RPM                                         80
//Speed mode maximum "powered" speed in RPM
#define BRAKE_AND_THROTTLE_MAXSPEED_AMBLE                    265       // 10 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_LEISURE                  477       // 18 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_SPORTS                   663       // 25 Km/hr

#endif // REGION1

#ifdef REGION2
#define REG_MAXPOUT                                          250 //Watt
#define REG_MAXP_SPEED                                       25  // km/hr
#define REG_MAXP_RPM                                         663 // rpm
#define REG_MINP_SPEED                                       3
#define REG_MINP_RPM                                         80
//Speed mode maximum "powered" speed in RPM
#define BRAKE_AND_THROTTLE_MAXSPEED_AMBLE                    265       //  10 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_LEISURE                  477       //  18 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_SPORTS                   663       //  25 Km/hr

#endif // REGION2

#ifdef REGION3
#define REG_MAXPOUT                                          250 //Watt
#define REG_MAXP_SPEED                                       24  // km/hr
#define REG_MAXP_RPM                                         636 // rpm
#define REG_MINP_SPEED                                       3
#define REG_MINP_RPM                                         80
//Speed mode maximum "powered" speed in RPM
#define BRAKE_AND_THROTTLE_MAXSPEED_AMBLE                    265       //  = 10 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_LEISURE                  451       //  = 17 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_SPORTS                   636       //  = 24 Km/hr

#endif // REGION3

#ifdef REGION4
#define REG_MAXPOUT                                          250 //Watt
#define REG_MAXP_SPEED                                       20  // km/hr
#define REG_MAXP_RPM                                         530 // rpm
#define REG_MINP_SPEED                                       3
#define REG_MINP_RPM                                         80
//Speed mode maximum "powered" speed in RPM
#define BRAKE_AND_THROTTLE_MAXSPEED_AMBLE                    265       // 275 RPM = 10 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_LEISURE                  398       // 488 RPM = 15 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_SPORTS                   530       // 530 RPM = 20 Km/hr

#endif // REGION4

#ifdef REGION5
#define REG_MAXPOUT                                          350 //Watt
#define REG_MAXP_SPEED                                       25  // km/hr
#define REG_MAXP_RPM                                         663 // rpm
#define REG_MINP_SPEED                                       3
#define REG_MINP_RPM                                         80
//Speed mode maximum "powered" speed in RPM
#define BRAKE_AND_THROTTLE_MAXSPEED_AMBLE                    265       // 275 RPM = 10 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_LEISURE                  477       // 488 RPM = 18 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_SPORTS                   663       // 674 RPM = 25 Km/hr

#endif // REGION5


#ifdef _cplusplus
}
#endif



#endif /* HARDWARE_GGO_DEVICE_PARAMS_H_ */
