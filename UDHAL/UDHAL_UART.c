/******************************************************************************

 @file  UDHAL_UART.c

 @brief This file contain the functions about


 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <ti/drivers/UART2.h>
#include "ti_drivers_config.h"
#include "Hardware/STM32MCP.h"

#include "UDHAL/UDHAL_UART.h"

/*********************************************************************
 * LOCAL VARIABLES
 */
static UART2_Handle UART2_handle;
static UART2_Params UART2_params;
static uint8_t receivedByte;

uint8_t uartOpenStatus = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void UDHAL_UART_open();
static void UDHAL_UART_read(uint8_t *message, uint8_t size);
static void UDHAL_UART_write(uint8_t *message, uint8_t size);
static void UDHAL_UART_close();
//static UART2_Callback UDHAL_readCallback(UART2_Handle UART2_handle, void *rxBuf, size_t size,
//                                         void *userArg, int_fast16_t status);
//static UART2_Callback UDHAL_writeCallback(UART2_Handle UART2_handle, void *rxBuf, size_t size,
//                                          void *userArg, int_fast16_t status);
static void UDHAL_readCallback(UART2_Handle UART2_handle, void *rxBuf, size_t size,
                                         void *userArg, int_fast16_t status);
static void UDHAL_writeCallback(UART2_Handle UART2_handle, void *rxBuf, size_t size,
                                          void *userArg, int_fast16_t status);

/*********************************************************************
 * Marco
 */
static STM32MCP_uartManager_t STM32MCP_uartManager =
{
    UDHAL_UART_open,
    UDHAL_UART_read,
    UDHAL_UART_write,
    UDHAL_UART_close
};
/*********************************************************************
 * @fn      UDHAL_UART_init
 *
 * @brief   It is used to initialize the UART
 *
 * @param   None
 *
 * @return  None
 */
void UDHAL_UART_init()
{

    //UART2_init(); // uart init is no longer required on cc2652R
    STM32MCP_registerUart(&STM32MCP_uartManager);
}
/*********************************************************************
 * @fn      UDHAL_UART_params_init
 *
 * @brief   It is used to initialize the UART parameters
 *
 * @param   None
 *
 * @return  None
 */
uint8_t UDHAL_UART_params_init()
{
    UART2_Params_init(&UART2_params);
    UART2_params.baudRate      = 115200;
    UART2_params.writeMode     = UART2_Mode_CALLBACK;
    UART2_params.writeCallback = UDHAL_writeCallback;
    UART2_params.readMode      = UART2_Mode_CALLBACK;
    UART2_params.readCallback  = UDHAL_readCallback;

    UDHAL_UART_open();
    return (uartOpenStatus);
}
/*********************************************************************
 * @fn      UDHAL_UART_open
 *
 * @brief   It is used to open the UART communication port
 *
 * @param   None
 *
 * @return  None
 */
static void UDHAL_UART_open()
{
    // Open the UART and initiate the first read
    UART2_handle = UART2_open(0, &UART2_params);
    if (!UART2_handle)
    {
        uartOpenStatus = 0;
    }
    else
    {
        uartOpenStatus = 1;
    }
    UDHAL_UART_read(&receivedByte, 1);
}
/*********************************************************************
 * @fn      UDHAL_UART_write
 *
 * @brief   It is used to write data on UART
 *
 * @param   message: The pointer to the message
 *          size: The size of the message
 *
 * @return  none
 */
static void UDHAL_UART_write(uint8_t *message, uint8_t size)
{
     UART2_write(UART2_handle, message, size, NULL);
}
/*********************************************************************
 * @fn      UDHAL_UART_read
 *
 * @brief   It is used to read data from the UART
 *
 * @param   message: the pointer to the message you want to put
 *          size: The size of read message
 *
 * @return  true if the data is successfully received
 *          false if there is problem
 */
static void UDHAL_UART_read(uint8_t *message, uint8_t size)
{
    UART2_read(UART2_handle, message, size, NULL);
}
/*********************************************************************
 * @fn      UDHAL_UART_close
 *
 * @brief   It is used to terminate UART communication
 *
 * @param   None
 *
 * @return  None
 */
static void UDHAL_UART_close()
{
    UART2_writeCancel(UART2_handle);
    UART2_readCancel(UART2_handle);
    UART2_close(UART2_handle);
}
/*********************************************************************
 * @fn      readCallback
 *
 * @brief   UART read call back, when uart has received a single byte, it will execute this code.
 *          I repeat, readCallback must be configured as byte by byte. Not a number of bytes.
 *          You must add STM32MCP_flowControlHandler to this function
 *
 * @param   None.
 *
 * @return  None.
 */
//static UART2_Callback UDHAL_readCallback(UART2_Handle UART2_handle, void *rxBuf, size_t size, void *userArg, int_fast16_t status)
static void UDHAL_readCallback(UART2_Handle UART2_handle, void *rxBuf, size_t size, void *userArg, int_fast16_t status)
{
      //Pass the received byte to the flow control handler
      STM32MCP_flowControlHandler(((uint8_t *)rxBuf)[0]);
      //Wait the the next received byte
      UDHAL_UART_read(&receivedByte, 1);
}

/*********************************************************************
 * @fn      readCallback
 *
 * @brief   neglect this function, not used but required for TI driver initialization
 *
 * @param   None.
 *
 * @return  None.
 */
//static UART2_Callback UDHAL_writeCallback(UART2_Handle UART2_handle, void *rxBuf, size_t size, void *userArg, int_fast16_t status)
static void UDHAL_writeCallback(UART2_Handle UART2_handle, void *rxBuf, size_t size, void *userArg, int_fast16_t status)
{

}
