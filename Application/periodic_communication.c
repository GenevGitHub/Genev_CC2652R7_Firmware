/*
 * periodic_communication.c
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 *
 *  This .c file contains the functions that interacts with the
 *  mcu to retrieve usage data, including battery voltage,
 *  instantaneous battery current, phase voltage, phase current,
 *  mcu temperature, motor temperature, mcu error alerts.
 *
 *  The retrieved data are stored in MCUDArray, which is accessed
 *  via its pointer, ptr_pc_MCUDArray.
 *
 */

/*********************************************************************
* LIBRARIES
*/
#include "Hardware/gGo_device_params.h"
#include "Application/periodic_communication.h"

#include "Application/motor_control.h"
#include "Application/data_analytics.h"


/*********************************************************************
* CONSTANTS
*/
#define DUMMY_DATA_ON         1
//#undef  DUMMY_DATA_ON
//#define MCU_DATA_ON           1
#undef  MCU_DATA_ON

/*********************************************************************
* LOCAL VARIABLES
*/
MCUD_t *ptr_pc_MCUDArray;


/*********************************************************************
* FUNCTIONS
*/

/**** obtain/register the pointer to MCUDArray   ****/
extern void periodic_communication_MCUArrayRegister(MCUD_t *ptrMCUDArray)
{
    ptr_pc_MCUDArray = ptrMCUDArray;
}


/*********************************************************************
 * @fn      periodic_communication_MCUSampling
 *
 * @brief   This function will execute at an interval of "PERIODIC_COMMUNICATION_SAMPLING_TIME"
 *
 *          The roles of this file is to communicate with the mcu to retrieve the following data from the mcu
 *             - battery voltage
 *             - battery current
 *             - Heat Sink Temperature
 *             - motor rpm
 *             - motor temperature
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
uint8_t     xhf = 1;
uint16_t    xtt = 0;
uint16_t    xrpm = 0;

void periodic_communication_MCUSampling()
{
    /*************************************************
     *  Get Bus Voltage
     *  Get Bus Current
     *  Get RPM
     *  Get Heatsink Temperature
     *  Get Motor Temperature
     *************************************************/
#ifdef DUMMY_DATA_ON

    // ***** Simulate Get data directly from MCU with dummy data
    ptr_pc_MCUDArray->count_hf = xhf;

    // ***** Simulation of dummy Battery Voltage
    ptr_pc_MCUDArray->bat_voltage_mV = 44200;//7200 *sin(PI_CONSTANT * xtt * 0.0075) + 39000;   // simulate over-voltage, voltage-normal and under-voltage.  unit in mV

    // ***** Simulation of dummy Battery Current
    ptr_pc_MCUDArray->bat_current_mA = 6000; //rand()%13 * 1000;                                 // 3000; // dummy data - get battery current from MCU:  unit in mA

    /***** Simulation of dummy RPM @ periodic_communication_MCUSamplingRPM   *****/

    // ***** Simulation of dummy Heatsink Temp + 50
    ptr_pc_MCUDArray->heatSinkTempOffset50_Celcius = 65;    //30 * sin(PI_CONSTANT * xtt * 0.0075) + 65;      // +50                     // dummy data - temperature is shifted by 20 degrees for taking care of - negative temperature

    // ***** Simulation of dummy Motor Temp + 50
    ptr_pc_MCUDArray->motorTempOffset50_Celcius = 60;//30 * sin(PI_CONSTANT * xtt * 0.0075) + 70;         // +50                     //  dummy data - temperature is shifted by 20 degrees for taking care of - negative temperature

    ptr_pc_MCUDArray->phase_voltage_mV = 31000;
//    ptr_pc_MCUDArray->phaseB_voltage_mV = 31000;
//    ptr_pc_MCUDArray->phaseC_voltage_mV = 31000;

    ptr_pc_MCUDArray->phase_current_mA = 6500;  //1500 * sin(PI_CONSTANT * xtt * 0.0075) + 5000;         // +50                     //  dummy data - temperature is shifted by 20 degrees for taking care of - negative temperature

    xtt++;

#endif // DUMMY_DATA_ON

#ifdef MCU_DATA_ON
    /******* get motor temperature from MCU: unit in degrees Celsius ****/
    /******* Get data directly from MCU     */
//    periodic_communication_STM32MCP_getRegisterFrame();
    /*************************************************
     *  Get Bus Voltage
     *  Get Bus Current
     *  Get Heatsink Temperature
     *  Get Motor Temperature
     *************************************************/
#ifdef CC2652R7_GENEV_5X5_ID
    STM32MCP_getRegisterFrame(STM32MCP_MOTOR_1_ID, STM32MCP_BUS_VOLTAGE_REG_ID);
    STM32MCP_getRegisterFrame(STM32MCP_MOTOR_1_ID, STM32MCP_TORQUE_MEASURED_REG_ID);       // Need to create a getRegisterFrame for battery current
    STM32MCP_getRegisterFrame(STM32MCP_MOTOR_1_ID, STM32MCP_SPEED_MEASURED_REG_ID);         // is speed in RPM
    //STM32MCP_getRegisterFrame(STM32MCP_MOTOR_1_ID, STM32MCP_HEATSINK_TEMPERATURE_REG_ID);
    //STM32MCP_getRegisterFrame(STM32MCP_MOTOR_1_ID, STM32MCP_MOTOR_TEMPERATURE_REG_ID);
    //STM32MCP_getRegisterFrame(STM32MCP_MOTOR_1_ID, STM32MCP_BUS_CURRENT_REG_ID);
    //STM32MCP_getRegisterFrame(STM32MCP_MOTOR_1_ID, STM32MCP_PHASE_VOLTAGE_REG_ID);
    //STM32MCP_getRegisterFrame(STM32MCP_MOTOR_1_ID, STM32MCP_PHASE_CURRENT_REG_ID);
#endif  //CC2652R7_GENEV_5X5_ID

#endif // DUMMY_DATA_ON

    xhf++;
    if (xhf >= DATA_ANALYSIS_POINTS)
    {
        xhf = 1;    // Resets xhf
    }

}


/**************************************************************************
 * func:    periodic_communication_MCUSamplingRPM
 *          Get rpm data directly from MCU
 **************************************************************************/
void periodic_communication_MCUSamplingRPM()
{
#ifdef DUMMY_DATA_ON

    // ***** Simulation of dummy RPM
    ptr_pc_MCUDArray->speed_rpm = 3 * sin(PI_CONSTANT * 0.0075 * xrpm ) + 264;  // 380;  //dummy data - get RPM from MCU:  unit in rpm.  20 secs per cycle = 0.05 Hz
    ptr_pc_MCUDArray->rpm_status = 1;
    xrpm++;

#endif // DUMMY_DATA_ON

#ifdef MCU_DATA_ON

    STM32MCP_getRegisterFrame(STM32MCP_MOTOR_1_ID, STM32MCP_SPEED_MEASURED_REG_ID);         // is speed in RPM

#endif  // CC2652R7_GENEV_5X5_ID

}

/*********************************************************************
 * @fn      periodic_communication_getxhf
 *
 * @brief   Returns the value of xhf
 *
 * @param
 *
 * @return  xhf
 *********************************************************************/
uint8_t periodic_communication_getxhf()
{
    return (xhf);
}
