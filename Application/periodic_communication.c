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
#include "Hardware/STM32MCP.h"
#include "UDHAL/UDHAL.h"// added 20241110

/*********************************************************************
* CONSTANTS
*/
/*** MOTOR_CONNECT defined in gGo_debug_config.h   ***/

/*********************************************************************
* LOCAL VARIABLES
*/
MCUD_t *ptr_pc_MCUDArray;
static sysFatalError_t *ptr_systemFatalError;       // added 20241110

/*********************************************************************
* FUNCTIONS
*/

/**** obtain/register the pointer to MCUDArray   ****/
extern void periodic_communication_MCUArrayRegister(MCUD_t *ptrMCUDArray)
{
    ptr_pc_MCUDArray = ptrMCUDArray;
}
/**** get the pointer/register to sysFatalError   ****/
extern void periodic_communication_init( void ) // added 20241110
{
    ptr_systemFatalError = UDHAL_sysFatalErrorRegister();
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
     *  Get Heatsink Temperature
     *  Get Motor Temperature
     *  Get MCU phase voltage
     *  Get MCU phase current
     *************************************************/

    // ***** Simulate Get data directly from MCU with dummy data
    ptr_pc_MCUDArray->count_hf = xhf;

#ifndef MOTOR_CONNECT   // if NOT defined
    // ***** Simulation of dummy Battery Voltage
    ptr_pc_MCUDArray->bat_voltage_mV = 3500 * sin(PI_CONSTANT * xtt * 0.005) + 38000;   // simulate over-voltage, voltage-normal and under-voltage.  unit in mV

    // ***** Simulation of dummy Battery Current
    ptr_pc_MCUDArray->bat_current_mA = 3000 * sin(PI_CONSTANT * xtt * 0.005) + 3000; // 3000; // dummy data - get battery current from MCU:  unit in mA

    // ***** Simulation of dummy Heatsink Temp + 50
    ptr_pc_MCUDArray->heatSinkTempOffset50_Celcius = 70;//30 * sin(PI_CONSTANT * xtt * 0.0075) + 65; // +50  dummy data - temperature is shifted by 20 degrees for taking care of - negative temperature

    // ***** Simulation of dummy Motor Temp + 50
    ptr_pc_MCUDArray->motorTempOffset50_Celcius = 70;//30 * sin(PI_CONSTANT * xtt * 0.0075) + 70;   // +50 dummy data - temperature is shifted by 20 degrees for taking care of - negative temperature

    ptr_pc_MCUDArray->phase_voltage_mV = 31000;

    ptr_pc_MCUDArray->phase_current_mA = 4500;  //1500 * sin(PI_CONSTANT * xtt * 0.0075) + 5000;  //  dummy data - temperature is shifted by 20 degrees for taking care of - negative temperature

    xtt++;
#endif // MOTOR_CONNECT

#ifdef MOTOR_CONNECT
    if (!(ptr_systemFatalError->UARTfailure))    // added if statement 20241110
    {
        /* behaviour ID STM32MCP_BUS_VOLTAGE_REG_ID = get DC-bus voltage */
        /*Checks DC Bus Voltage in mV*/
        STM32MCP_controlEscooterBehavior(ESCOOTER_VOLTAGE_CHECKING);

        /* behaviour ID ESCOOTER_CURRENT_CHECKING = get DC-bus current */
        /*Checks current in mA */
        STM32MCP_controlEscooterBehavior(ESCOOTER_CURRENT_CHECKING);

        /* behaviour ID ESCOOTER_MOTOR_TEMP = get Motor Temperature*/
        /*Check Motor Temperature*/
        STM32MCP_controlEscooterBehavior(ESCOOTER_MOTOR_TEMP);

        /* behaviour ID ESCOOTER_MOTOR_DRIVE_TEMP = get Motor Driver Temperature */
        /*Check Motor Driver Temperature*/
//        STM32MCP_controlEscooterBehavior(ESCOOTER_MOTOR_DRIVE_TEMP);

    }
#endif // MOTOR_CONNECT

    xhf++;
    if (xhf >= DATA_ANALYSIS_POINTS) {
        xhf = 1;    // Resets xhf
    }
}

/**************************************************************************
 * func:    periodic_communication_MCUSamplingRPM
 *          Get rpm data directly from MCU
 **************************************************************************/
void periodic_communication_MCUSamplingRPM()
{
#ifndef MOTOR_CONNECT   // if NOT defined
    // ***** Simulation of dummy RPM
    uint16_t pc_rpm;   // but payload length is 0x05
    int32_t rawRPM = 200 * sin(PI_CONSTANT * 0.008 * xrpm ) + 220;  // 264;  //dummy data - get RPM from MCU:  unit in rpm.  20 secs per cycle = 0.05 Hz;
    uint8_t pc_rpmStatus;

    if(rawRPM >= 0)
    {
        pc_rpm = (uint16_t) (rawRPM & 0xFFFF);
        pc_rpmStatus = 1;  // when mc_rpm is >= 0
    }
    else  // **** if rawRPM is negative, e.g. pushing the E-scooter in reverse, No power shall be delivered to motor.
    {
        pc_rpm = (uint16_t) (-rawRPM & 0xFFFF);
        pc_rpmStatus = 0;  // when mc_rpm < 0
    }
    ptr_pc_MCUDArray->speed_rpm = pc_rpm;
    ptr_pc_MCUDArray->rpm_status = pc_rpmStatus;
    xrpm++;
#endif // MOTOR_CONNECT

#ifdef MOTOR_CONNECT
    // execute motor_control_brakeStatusChg() and motor_control_setIQvalue() only UART is normal
    if (!(ptr_systemFatalError->UARTfailure))        // added if statement 20241110
    {
        STM32MCP_getRegisterFrame(STM32MCP_MOTOR_1_ID, STM32MCP_SPEED_MEASURED_REG_ID);         // is speed in RPM
    }
#endif  // MOTOR_CONNECT

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
