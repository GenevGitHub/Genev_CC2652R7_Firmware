/*
 * motor_control.c
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

/*********************************************************************
 * INCLUDES
 */
#include "Hardware/gGo_device_params.h"
#include "Hardware/STM32MCP.h"

#include "Application/motor_control.h"
#include "Application/periodic_communication.h"
#include "Application/data_analytics.h"
#include "Application/brake_and_throttle.h"

/*********************************************************************
 * CONSTANTS
 */
#define TEMPERATUREOFFSET       50

/*********************************************************************
 * GLOBAL VARIABLES
 */
/*********************************************************************
 * LOCAL VARIABLES
 */
//static uint8_t motorcontrol_i2cOpenStatus = 0;    // obsolete - delete
static simplePeripheral_bleCBs_t *motorcontrol_bleCBs;
//static uint8_t motorControl_getGAPRole_taskCreate_flag = 0;   // no longer required - delete
//bool powerOn = 1;            // no longer required - singlebuttonCB moved to mpb

MCUD_t MCUDArray = {LEVEL45,
                    3000,
                    32000,
                    3000,
                    0,
                    70,
                    70,
                    0   };


/**********************************************************************
 *  Local functions
 */
static void motorcontrol_processGetRegisterFrameMsg(uint8_t *txPayload, uint8_t txPayloadLength, uint8_t *rxPayload, uint8_t rxPayloadLength);
static void motorcontrol_rxMsgCb(uint8_t *rxMsg, STM32MCP_txMsgNode_t *STM32MCP_txMsgNode);
static void motorcontrol_exMsgCb(uint8_t exceptionCode);
static void motorcontrol_erMsgCb(uint8_t errorCode);

static void motorcontrol_brakeAndThrottleCB(uint16_t allowableSpeed, uint16_t IQValue, uint8_t errorMsg);

/*********************************************************************
 * TYPEDEFS
 */
static STM32MCP_CBs_t motor_control_STM32MCP_CBs =
{
     motorcontrol_rxMsgCb,
     motorcontrol_exMsgCb,
     motorcontrol_erMsgCb
};


/*********************************************************************
* PUBLIC FUNCTIONS
*/

uint8_t fail = 0;
uint32_t resetSource = 0xFFFF;
uint8_t bootStatus = 0xFF;
uint8_t bootSuccess = 0x00;
uint8_t buttonPress = 0xFF;
uint8_t bootProcess = 0xFF;
uint8_t wkUpPin = 0xFF;
/*
PIN_Config WakeUp[] = {
      CC2640R2_GENEV_5X5_ID_DIO5 | PIN_INPUT_EN | PIN_PULLUP | PINCC26XX_WAKEUP_NEGEDGE,
      PIN_TERMINATE
};
*/
extern uint8_t Boot()
{
    /*Boot Process*/
//    resetSource = SysCtrlResetSourceGet();
//    if (resetSource == RSTSRC_WAKEUP_FROM_SHUTDOWN)
//    {
//         bootStatus = 0x01;
//    }
//    else if (resetSource == RSTSRC_PWR_ON)
//    {
//         bootStatus = 0x02;
//    }
//    else if (resetSource == RSTSRC_SYSRESET)
//    {
//        bootStatus = 0x03;
//    }
//    else if (resetSource == RSTSRC_PIN_RESET)
//    {
//        bootStatus = 0x04;
//        while(bootSuccess == 0x00)
//        {
//            if(bootSuccess == 0x01)
//            {
//                break;
//            }
//        }
//    }
    return bootStatus;
}

/*********************************************************************
 * @fn      motor_control_init
 *
 * @brief   Initialization function for the Motor Control Task.
 *
 * @param   none
 *
 * @return  none
 */
//uint8_t mccheck = 0;        // FOR DEBUGGING ONLY
uint8_t bootFlag = 0xFF;
void motor_control_init(void)
{
    data_analytics_MCUDArrayRegister(&MCUDArray);            // passes ptrMCUDArray to dataAnalysis.c
    periodic_communication_MCUArrayRegister(&MCUDArray);
    brake_and_throttle_MCUArrayRegister(&MCUDArray);

    STM32MCP_registerCBs(&motor_control_STM32MCP_CBs);
    STM32MCP_startCommunication();    //ACTIVATE OR DEACTIVATE

}

/*********************************************************************
 * @fn      motorcontrol_registerCB
 *
 * @brief   When the motor controller sends the get register frame message
 *
 * @param   STM32MCP_rxMsg_t - The memory message to the received message, the size of the message is the second index rxMsg[1]
 *
 * @return  None.
 */
void motorcontrol_registerCB(simplePeripheral_bleCBs_t *obj)
{
    motorcontrol_bleCBs = obj;
}


/*********************************************************************
 * @fn      motorcontrol_processGetRegisterFrameMsg
 *
 * @brief   When the motor controller sends the get register frame message
 *
 * @param   STM32MCP_rxMsg_t - The memory message to the received message, the size of the message is the second index rxMsg[1]
 *
 * @return  None.
 */
static void motorcontrol_processGetRegisterFrameMsg(uint8_t *txPayload, uint8_t txPayloadLength, uint8_t *rxPayload, uint8_t rxPayloadLength)
{
    uint8_t regID = txPayload[0];
    switch(regID)
    {
    case STM32MCP_BUS_VOLTAGE_REG_ID:
        {
            uint16_t voltage_mV = *((uint16_t*) rxPayload) * 1000; // rxPayload in V, voltage in mV
        // **** store voltage_mV in MCUArray.voltage_mV
            MCUDArray.bat_current_mA = voltage_mV;
            break;
        }
    // *********  current sensor to be added to MCU.  Reserved case for current measurement
    case STM32MCP_TORQUE_MEASURED_REG_ID:
        {
        //keep current in mV - do not convert it to A
            uint16_t current_mA = *((uint8_t*) rxPayload) * 1000;
        // **** store current_mA in MCUArray.current_mV
            MCUDArray.bat_current_mA = 3000;//current_mA;
            break;
        }
    case STM32MCP_HEATSINK_TEMPERATURE_REG_ID:
        {
            uint8_t heatSinkTemperature_Celcius = (*((uint8_t*) rxPayload) & 0xFF) + TEMPERATUREOFFSET;     // temperature can be a negative value, unless it is in Kelvin
        // **** store heatSinkTemperature_Celcius in MCUArray.heatSinkTemperature_Celcius
            MCUDArray.heatSinkTempOffset50_Celcius = 15;//heatSinkTemperature_Celcius;  // +50
            break;
        }
    case STM32MCP_SPEED_MEASURED_REG_ID:    // RPM
        {
            uint16_t rpm;
            int32_t rawRPM = *((int32_t*) rxPayload);
            if(rawRPM >= 0)
            {
                rpm = (uint16_t) (rawRPM & 0xFFFF);
            }
            else  // **** what if rawRPM is negative??? e.g. pushing the E-scooter in reverse.  Then rawRPM would be negative!!!
            {
                rpm = (uint16_t) (-rawRPM & 0xFFFF);
            }
            // **** store rpm in MCUArray.speed_rpm
            MCUDArray.speed_rpm = rpm;
            break;
        }
// ********************    Need to create new REG_IDs
    case STM32MCP_MOTOR_TEMPERATURE_REG_ID:
        {
            uint8_t motorTemperature_Celcius = (*((uint8_t*) rxPayload) & 0xFF) + TEMPERATUREOFFSET;     // temperature can be a negative value, unless it is in Kelvin
            MCUDArray.motorTempOffset50_Celcius = 20;//motorTemperature_Celcius;    // +50

            break;
        }
    case STM32MCP_CONTROLLER_ERRORCODE_REG_ID:
        {

            //
            break;
        }
    case STM32MCP_CONTROLLER_PHASEVOLTAGE_REG_ID:
        {

            //
            break;
        }
    case STM32MCP_CONTROLLER_PHASECURRENT_REG_ID:
        {

            //
            break;
        }
    default:
            break;
    }

}


/*********************************************************************
 * @fn      motorcontrol_rxMsgCb
 *
 * @brief   When the motor controller sends the feedback message back, it reaches here
 *
 * @param   rxMsg - The memory message to the received message, the size of the message is the second index rxMsg[1]
 *          motorID - The motor ID described in STM32MCP
 *          frameID - The frame ID described in STM32MCP
 *
 * @return  None.
 */
static void motorcontrol_rxMsgCb(uint8_t *rxMsg, STM32MCP_txMsgNode_t *STM32MCP_txMsgNode)
{
    uint8_t frameID = STM32MCP_txMsgNode->txMsg[0] & 0x1F;

    uint8_t rxPayloadLength = rxMsg[1];
    uint8_t *rxPayload = (uint8_t*)malloc(sizeof(uint8_t)*rxPayloadLength);
    memcpy(rxPayload, rxMsg + 2, rxPayloadLength);

    uint8_t txPayloadLength = STM32MCP_txMsgNode->size;
    uint8_t *txPayload = (uint8_t*)malloc(sizeof(uint8_t)*txPayloadLength);
    memcpy(txPayload, STM32MCP_txMsgNode->txMsg + 2, txPayloadLength);

    switch(frameID)
    {
    case STM32MCP_SET_REGISTER_FRAME_ID:
        break;
    case STM32MCP_GET_REGISTER_FRAME_ID:
        motorcontrol_processGetRegisterFrameMsg(txPayload, txPayloadLength, rxPayload, rxPayloadLength);
        break;
    case STM32MCP_EXECUTE_COMMAND_FRAME_ID:
        break;
    case STM32MCP_GET_BOARD_INFO_FRAME_ID:
        break;
    case STM32MCP_EXEC_RAMP_FRAME_ID:
        break;
    case STM32MCP_GET_REVUP_DATA_FRAME_ID:
        break;
    case STM32MCP_SET_REVUP_DATA_FRAME_ID:
        break;
    case STM32MCP_SET_CURRENT_REFERENCES_FRAME_ID:
        break;
    case STM32MCP_SET_SYSTEM_CONTROL_CONFIG_FRAME_ID:
        break;
    case STM32MCP_SET_DRIVE_MODE_CONFIG_FRAME_ID:
        break;
    case STM32MCP_SET_DYNAMIC_TORQUE_FRAME_ID:
        break;
    default:
        break;
    }
    free(rxPayload);
    free(txPayload);
}

/*********************************************************************
 * @fn      motorcontrol_exMsgCb
 *
 * @brief   When the motor controller sends the exception back, it reaches here
 *
 * @param   exceptionCode - The exception code described in STM32MCP
 *
 * @return  None.
 */
static void motorcontrol_exMsgCb(uint8_t exceptionCode)
{
    switch(exceptionCode)
        {
        case STM32MCP_QUEUE_OVERLOAD:
            break;
        case STM32MCP_EXCEED_MAXIMUM_RETRANSMISSION_ALLOWANCE:
            break;
        default:
            break;
        }
}

/*********************************************************************
 * @fn      motorcontrol_erMsgCb
 *
 * @brief   When the motor controller sends the error back, it reaches here
 *
 * @param   exceptionCode - The error code described in STM32MCP
 *
 * @return  None.
 */
static void motorcontrol_erMsgCb(uint8_t errorCode)
{
    switch(errorCode)
    {
    case STM32MCP_BAD_FRAME_ID:
        break;
    case STM32MCP_WRITE_ON_READ_ONLY:
        break;
    case STM32MCP_READ_NOT_ALLOWED:
        break;
    case STM32MCP_BAD_TARGET_DRIVE:
        break;
    case STM32MCP_OUT_OF_RANGE:
        break;
    case STM32MCP_BAD_COMMAND_ID:
        break;
    case STM32MCP_OVERRUN_ERROR:
        break;
    case STM32MCP_TIMEOUT_ERROR:
        break;
    case STM32MCP_BAD_CRC:
        break;
    default:
        break;
    }
}

