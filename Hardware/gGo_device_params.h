/*
 *  gGo_device_params.h
 *  Note:   This header file contains gGo eScooter factory settings
 *
 *  Parameters and Constants
 *      REGION selection
 *      BRAKE_AND_THROTTLE_NORMALLAW
 *      BRAKE_AND_THROTTLE_DIRECTLAW
 *      ALS chip option. (1) veml6030 (2) veml3235
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
///*********************************************************************************************
// *  Battery Option
// *********************************************************************************************/
#include "Hardware/battery_params.h"

/*********************************************************************
 * CONSTANTS
 */
// Device Information
#define DEVICE_MODEL_NUMBER               "Genev gGo ES01"
#define DEVICE_SERIAL_NUMBER              "MSN T00001"     // MSN is unique for each production eScooter. Input the MSN here before uploading firmware.
#define FIRMWARE_REVISION                 "ES01.0.V01.R00"
#define MANUFACTURER_NAME                 "Genev"

/*********************************************************************************************
 *  Regional / Regulation Options
 *********************************************************************************************/
//#define  REGION0                                     // Unregulated
#define REGION1                             1       // HK, Singapore
//#define  REGION2                                     // EU
//#define  REGION3                                     // Japan
//#define  REGION4                                     // Melbourne Australia
//#define  REGION5                                     // Canada, USA
//#define  REGION6                                     // Germany

// Speed limit protection selection
// * Direct Law -> speed limit protection is deactivated
// * Normal law -> speed limit protection is activated
#define BRAKE_AND_THROTTLE_NORMALLAW                              1
#define BRAKE_AND_THROTTLE_DIRECTLAW                              0

///*********************************************************************************************
// *  Ambient Light Sensor Chip Option
// *********************************************************************************************/
//#define veml6030            1
#define veml3235            1

//#define AUXILIARY_LIGHT                     1

/******   CONSTANTS   *******/
#define PI_CONSTANT                         3.14159265359

/*********************************************************************************************
 *  Vehicle Information
 *********************************************************************************************/
#define WHEELRADIUS_CM                      10.16           // wheel radius in centimeter
#define COEFF01                             0.2156          // kg/km
#define COEFF02                             0.000386        // kg/W-hr
#define BCF                                 0.9             // Battery Capacity Safety Factor
#define CRIT_HEATSINKTEMPOFFSET50C          160             // Critical Heatsink Temperature + 50 degrees C
#define CRIT_MOTORTEMPOFFSET50C             200             // Critical Motor Temperature + 50 degrees C
/*********************************************************************************************
 *  Unit Conversion
 *********************************************************************************************/
#define KM2MILE                             0.621           // convert length from km to miles
#define CM2INCH                             0.3937           // convert length from cm to inch
#define KG2LBS                              2.205           // convert kilogram to pounds
#define SI_UNIT                             0x00
#define IMP_UNIT                            0x01

/***************************************************************************
 *     Motor Characteristics - BLDC 36-48V 15Amax 8-icnh hub motor
 *     with decoupling caps and NTC thermal sensor
 *************************************************************************/
#define KT_CONSTANT                         0.5895
#define KIQ_CONSTANT                        0.0008889
#define MOTOR_RESISTANCE                    0.6343
#define FRICTION_TORQUE                     0.1919
#define DAMPING_CONSTANT                    0.003443

/*********************************************************************************************
 *  Battery Status Colours
 *********************************************************************************************/
#define GLOWING_AQUA                        0x05
#define GLOWING_GREEN                       0x04
#define YELLOW                              0x03
#define ORANGE                              0x02
#define RED                                 0x01
#define FLASHING_RED                        0x00
#define BATTERY_STATUS_INITIAL              0x05

/*********************************************************************************************
 *  Light Settings
 *********************************************************************************************/
#define LUXTHRESHOLD                        500  // light shall be ON when light intensity is consistently below this lux value
#define DASHBOARD_ALS_CORR_FACTOR           1.888    // this correction factor corrects the difference between dashboard-covered-ALS and uncovered_ALS.

/*********************************************************************************************
 *  Error Priorities and Codes
 *********************************************************************************************/
/********************************************************************************************************************************************/
/*      error type                         error priority (criticality)      Description                               error code displayed */
// system Normal
#define SYSTEM_NORMAL_PRIORITY                          0xFF    // 255: System normal

// system fatal errors
#define SYS_FATAL_ERROR_PRIORITY                        0x00
#define UART_OPEN_NULL                                  SYS_FATAL_ERROR_PRIORITY    // 00:UART open failure                         (LED display error code 0F)
#define PWM1_OPEN_NULL                                  SYS_FATAL_ERROR_PRIORITY    // 00:buzzer PWM failure                        (LED display error code 0F)
#define PWM2_OPEN_NULL                                  SYS_FATAL_ERROR_PRIORITY    // 00:light PWM failure                         (LED display error code 0F)
#define I2C_OPEN_NULL                                   SYS_FATAL_ERROR_PRIORITY    // 00:I2C open failure affects light sensor and led display (LED display error code 0F - will not display since i2c failed)
#define ADC1_OPEN_NULL                                  SYS_FATAL_ERROR_PRIORITY    // 00:ADC1 open failure affects brake           (LED display error code 0F)
#define ADC2_OPEN_NULL                                  SYS_FATAL_ERROR_PRIORITY    // 00:ADC2 open failure affects throttle        (LED display error code 0F)

// Critical hardware/firmware errors
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
#define THROTTLE_ERROR_PRIORITY                         0x10    // 17:Throttle error priority                   (LED display error code 0C)

// Minor hardware/firmware/software errors
//#define SOFTWARE_ERROR_PRIORITY                         0X12
#define BRAKE_ERROR_PRIORITY                            0x1A    // 16:Brake error priority                      (LED display error code 0E)

// Warnings
#define BATTERY_CRITICALLY_LOW_PRIORITY                 0x20    // 32:Battery level critically low warning

/*********************************************************************************************/
/**  LED Display and MOBILE APP Error Codes  **/
/*********************************************************************************************/
#define SYS_NORMAL_CODE                                 0xFF

#define BATTERY_VOLTAGE_ERROR_CODE                      0x1A    // check that the e-scooter has not been connected with an incorrect battery rating. Restart (Power off, wait for 10 seconds and power on again)
#define BATTERY_TEMP_ERROR_CODE                         0x1A    // no battery temperature sensor at present.   The e-scooter's throttle response will automatically unlock when temp drop below the pre-set value.
#define BMS_COMM_ERROR_CODE                             0x1C    // Replace battery with a new genev battery. Restart, contact customer service if problem persists.
#define BATTERY_VOLTAGE_CRIT_LOW_CODE                   0x1E    // recharge battery immediately. If battery depletes rapidly, replace with a new genev battery.
#define GATE_DRIVER_ERROR_CODE                          0x2C    // Restart, contact genev technical / customer services or approved distributor for inspection and repair
#define MOSFET_ERROR_CODE                               0x2E    // Restart, contact genev technical / customer services or approved distributor for inspection and repair
#define PHASE_I_ERROR_CODE                              0x2A    // Restart, contact genev technical / customer services or approved distributor for inspection and repair
#define CONTROLLER_TEMP_ERROR_CODE                      0x2F    // Let it cool before use again.  The e-scooter's throttle response will automatically unlock when temp drop below the pre-set value.
#define HALL_SENSOR_ERROR_CODE                          0x3A    // Restart, contact genev technical / customer services or approved distributor for inspection and repair
#define MOTOR_TEMP_ERROR_CODE                           0x3C    // Let it cool before use again.   The e-scooter's throttle response will automatically unlock when temp drop below the pre-set value.
#define DASH_COMM_ERROR_CODE                            0x0A    // Restart, contact genev technical / customer services or approved distributor for inspection and repair
#define THROTTLE_ERROR_CODE                             0x0C    // check throttle connection with Dashboard.  Restart, contact genev technical / customer services or approved distributor for inspection and repair in problem persists.
#define BRAKE_ERROR_CODE                                0x0E    // check brake connection with Dashboard.  Restart, contact genev technical / customer services or approved distributor for inspection and repair in problem persists.
#define SOFTWARE_ERROR_CODE                             0x4A    // Restart, contact genev technical / customer services or approved distributor for inspection and repair in problem persists.
#define SYS_FATAL_ERROR_CODE                            0x0F

/*********************************************************************************************
 *  Regional / Regulation Settings
 *********************************************************************************************/
#ifdef REGION0  // unlimited
#define REG_MAXPOUT                                          300 //Watt
#define REG_MAXP_SPEED                                       30  // "30" km/hr
#define REG_MAXP_RPM                                         795 // 30 Km/hr
#define REG_MINP_SPEED                                       3
#define REG_MINP_RPM                                         1
//Speed mode maximum "powered" speed in RPM
#define BRAKE_AND_THROTTLE_MAXSPEED_AMBLE                    318       // 12 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_LEISURE                  557       // 21 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_SPORTS                   REG_MAXP_RPM       // 30 Km/hr

#endif // REGION0

#ifdef REGION1
#define REG_MAXPOUT                                          300 //Watt
#define REG_MAXP_SPEED                                       25  // km/hr
#define REG_MAXP_RPM                                         660 // rpm
#define REG_MINP_SPEED                                       3
#define REG_MINP_RPM                                         80
//Speed mode maximum "powered" speed in RPM
#define BRAKE_AND_THROTTLE_MAXSPEED_AMBLE                    265       // 10 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_LEISURE                  475       // 18 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_SPORTS                   REG_MAXP_RPM       // 25 Km/hr.
// We can set sports mode max speed to different value lower than the regulation maximum speed.
// The set value shall be less than or equal to regulation max speed.

#endif // REGION1

#ifdef REGION2
#define REG_MAXPOUT                                          250 //Watt
#define REG_MAXP_SPEED                                       25  // km/hr
#define REG_MAXP_RPM                                         660 // rpm
#define REG_MINP_SPEED                                       3
#define REG_MINP_RPM                                         80
//Speed mode maximum "powered" speed in RPM
#define BRAKE_AND_THROTTLE_MAXSPEED_AMBLE                    265       //  10 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_LEISURE                  475       //  18 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_SPORTS                   REG_MAXP_RPM       //  25 Km/hr

#endif // REGION2

#ifdef REGION3
#define REG_MAXPOUT                                          250 //Watt
#define REG_MAXP_SPEED                                       24  // km/hr
#define REG_MAXP_RPM                                         635 // rpm
#define REG_MINP_SPEED                                       3
#define REG_MINP_RPM                                         80
//Speed mode maximum "powered" speed in RPM
#define BRAKE_AND_THROTTLE_MAXSPEED_AMBLE                    265       //  = 10 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_LEISURE                  450       //  = 17 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_SPORTS                   REG_MAXP_RPM       //  = 24 Km/hr

#endif // REGION3

#ifdef REGION4
#define REG_MAXPOUT                                          250 //Watt
#define REG_MAXP_SPEED                                       20  // km/hr
#define REG_MAXP_RPM                                         530 // rpm
#define REG_MINP_SPEED                                       3
#define REG_MINP_RPM                                         80
//Speed mode maximum "powered" speed in RPM
#define BRAKE_AND_THROTTLE_MAXSPEED_AMBLE                    265       // 265 RPM = 10 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_LEISURE                  395       // 395 RPM = 15 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_SPORTS                   REG_MAXP_RPM       // 530 RPM = 20 Km/hr

#endif // REGION4

#ifdef REGION5 // Canada, USA
#define REG_MAXPOUT                                          300 //Watt
#define REG_MAXP_SPEED                                       28  // km/hr
#define REG_MAXP_RPM                                         742 // rpm
#define REG_MINP_SPEED                                       3
#define REG_MINP_RPM                                         80
//Speed mode maximum "powered" speed in RPM
#define BRAKE_AND_THROTTLE_MAXSPEED_AMBLE                    265       // 265 RPM = 10 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_LEISURE                  530       // 488 RPM = 20 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_SPORTS                   REG_MAXP_RPM       // 28 Km/hr

#endif // REGION5

#ifdef REGION6 // Germany
#define REG_MAXPOUT                                          300 // Watt
#define REG_MAXP_SPEED                                       20  // km/hr
#define REG_MAXP_RPM                                         530 // rpm
#define REG_MINP_SPEED                                       3
#define REG_MINP_RPM                                         80
//Speed mode maximum "powered" speed in RPM
#define BRAKE_AND_THROTTLE_MAXSPEED_AMBLE                    265       // 265 RPM = 10 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_LEISURE                  400       // 488 RPM = 15 Km/hr
#define BRAKE_AND_THROTTLE_MAXSPEED_SPORTS                   REG_MAXP_RPM       // 530 RPM = 20 Km/hr

#endif // REGION6

#ifdef _cplusplus
}
#endif


#endif /* HARDWARE_GGO_DEVICE_PARAMS_H_ */
