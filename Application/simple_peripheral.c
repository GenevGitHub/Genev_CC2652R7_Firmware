/******************************************************************************

 @file  simple_peripheral.c

 @brief This file contains the Simple Peripheral application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2013-2024, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#if (!(defined __TI_COMPILER_VERSION__) && !(defined __GNUC__))
#include <intrinsics.h>
#endif

#include <ti/drivers/GPIO.h>
#include <ti/drivers/utils/List.h>

#include <icall.h>
#include "util.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>
#include <simple_gatt_profile.h>

#include "Profiles/dashboard_profile.h"
#include "Profiles/battery_profile.h"
#include "Profiles/controller_profile.h"
#include "Profiles/profile_charVal.h"

/***  Include any applications that require publishing characteristic value to Simple Periphreal ***/
//#include "Application/......h"


#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC

#include <ti_drivers_config.h>
#include "simple_peripheral.h"
#include "ti_ble_config.h"

#ifdef PTM_MODE
#include "npi_task.h"               // To allow RX event registration
#include "npi_ble.h"                // To enable transmission of messages to UART
#include "icall_hci_tl.h"   // To allow ICall HCI Transport Layer
#endif // PTM_MODE

/*  When using and working with BLE functions, NVS must be accessed using
 *  OSAL_SNV library.   OSAL SNV operations are defined through ICALL */
#include <osal_snv.h>

#include "UDHAL/UDHAL.h"

#include "Hardware/gGo_device_params.h"
#include "Hardware/STM32MCP.h"
#include "Hardware/ESCOOTER_BOOT.h"
#include "Application/general_purpose_timer.h"
#include "Application/snv_internal.h"
#include "Application/brake_and_throttle.h"
#include "Application/data_analytics.h"
#include "Application/lights.h"
#include "Application/led_display.h"
#include "Application/multi_purpose_button.h"
#include "Application/motor_control.h"
#include "Application/power_on_time.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Application events
#define SP_STATE_CHANGE_EVT                  0
#define SP_CHAR_CHANGE_EVT                   1
#define SP_KEY_CHANGE_EVT                    2
#define SP_ADV_EVT                           3
#define SP_PAIR_STATE_EVT                    4
#define SP_PASSCODE_EVT                      5
#define SP_PERIODIC_EVT                      6
#define SP_READ_RPA_EVT                      7
#define SP_SEND_PARAM_UPDATE_EVT             8
#define SP_CONN_EVT                          9

// Internal Events for RTOS application
#define SP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define SP_ALL_EVENTS                        (SP_ICALL_EVT             | \
                                              SP_QUEUE_EVT)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define SP_ADDR_STR_SIZE     15

// For storing the active connections
#define SP_RSSI_TRACK_CHNLS        1            // Max possible channels can be GAP_BONDINGS_MAX
#define SP_MAX_RSSI_STORE_DEPTH    5
#define SP_INVALID_HANDLE          0xFFFF
#define RSSI_2M_THRSHLD           -30           
#define RSSI_1M_THRSHLD           -40           
#define RSSI_S2_THRSHLD           -50           
#define RSSI_S8_THRSHLD           -60           
#define SP_PHY_NONE                LL_PHY_NONE  // No PHY set
#define AUTO_PHY_UPDATE            0xFF

// Spin if the expression is not true
#define SIMPLEPERIPHERAL_ASSERT(expr) if (!(expr)) simple_peripheral_spin();

/*********************************************************************
 * TYPEDEFS
 */

// Auto connect available groups
enum
{
  AUTOCONNECT_DISABLE = 0,              // Disable
  AUTOCONNECT_GROUP_A = 1,              // Group A
  AUTOCONNECT_GROUP_B = 2               // Group B
};


// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct
{
  uint8_t event;                // event type
  void    *pData;               // pointer to message
} spEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} spPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} spPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} spGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;                //
  uint8_t data[];
} spClockEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
  List_Elem elem;
  uint16_t  connHandle;
} spConnHandleEntry_t;

// Connected device information
typedef struct
{
  uint16_t         	    connHandle;                        // Connection Handle
  spClockEventData_t*   pParamUpdateEventData;
  Clock_Struct*    	    pUpdateClock;                      // pointer to clock struct
  int8_t           	    rssiArr[SP_MAX_RSSI_STORE_DEPTH];
  uint8_t          	    rssiCntr;
  int8_t           	    rssiAvg;
  bool             	    phyCngRq;                          // Set to true if PHY change request is in progress
  uint8_t          	    currPhy;
  uint8_t          	    rqPhy;
  uint8_t          	    phyRqFailCnt;                      // PHY change request count
  bool             	    isAutoPHYEnable;                   // Flag to indicate auto phy change
} spConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task configuration
Task_Struct spTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(spTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t spTaskStack[SP_TASK_STACK_SIZE];

#define APP_EVT_EVENT_MAX 0x9
char *appEventStrings[] = {
  "APP_STATE_CHANGE_EVT     ",
  "APP_CHAR_CHANGE_EVT      ",
  "APP_KEY_CHANGE_EVT       ",
  "APP_ADV_EVT              ",
  "APP_PAIR_STATE_EVT       ",
  "APP_PASSCODE_EVT         ",
  "APP_READ_RPA_EVT         ",
  "APP_PERIODIC_EVT         ",
  "APP_SEND_PARAM_UPDATE_EVT",
  "APP_CONN_EVT             ",
};

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients
static Clock_Struct clkPeriodic;
// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Memory to pass periodic event ID to clock handler
spClockEventData_t argPeriodic =
{ .event = SP_PERIODIC_EVT };

// Memory to pass RPA read event ID to clock handler
spClockEventData_t argRpaRead =
{ .event = SP_READ_RPA_EVT };

// Per-handle connection info
static spConnRec_t connList[MAX_NUM_BLE_CONNS];

// Current connection handle as chosen by menu
static uint16_t menuConnHandle = LINKDB_CONNHANDLE_INVALID;

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Auto connect Disabled/Enabled {0 - Disabled, 1- Group A , 2-Group B, ...}
uint8_t autoConnect = AUTOCONNECT_DISABLE;

// Advertising handles
static uint8 advHandleLegacy;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimplePeripheral_init( void );
static void SimplePeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg);
static void SimplePeripheral_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData);
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg);

//static void SimplePeripheral_processStateChangeEvt(gaprole_States_t newState);
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramId);
static void dashboard_processCharValueChangeEvt(uint8_t paramId);

static void SimplePeripheral_performPeriodicTask(void);
static void SimplePeripheral_updateRPA(void);
static void SimplePeripheral_clockHandler(UArg arg);
static void SimplePeripheral_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison);
static void SimplePeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status);
static void SimplePeripheral_processPairState(spPairStateData_t *pPairState);
static void SimplePeripheral_processPasscode(spPasscodeData_t *pPasscodeData);

static void SimplePeripheral_charValueChangeCB(uint8_t paramId);
static void SimplePeripheral_dashboardCB(uint8_t paramID);
static void SimplePeripheral_batteryCB(uint8_t paramID);
static void SimplePeripheral_controllerCB(uint8_t paramID);

static status_t SimplePeripheral_enqueueMsg(uint8_t event, void *pData);

static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void SimplePeripheral_initPHYRSSIArray(void);
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg);
static uint8_t SimplePeripheral_addConn(uint16_t connHandle);
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle);
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle);
static void SimplePeripheral_processParamUpdate(uint16_t connHandle);
static status_t SimplePeripheral_startAutoPhyChange(uint16_t connHandle);
static status_t SimplePeripheral_stopAutoPhyChange(uint16_t connHandle);
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts);
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle);


static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport);
#ifdef PTM_MODE
void simple_peripheral_handleNPIRxInterceptEvent(uint8_t *pMsg);  // Declaration
static void simple_peripheral_sendToNPI(uint8_t *buf, uint16_t len);  // Declaration
#endif // PTM_MODE

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */
//// GAP Bond Manager Callbacks
//static gapBondCBs_t SimplePeripheral_BondMgrCBs =
//{
//  SimplePeripheral_passcodeCb,       // Passcode callback
//  SimplePeripheral_pairStateCb       // Pairing/Bonding state Callback
//};
//
//// Simple GATT Profile Callbacks
//static simpleProfileCBs_t SimplePeripheral_simpleProfileCBs =
//{
//  SimplePeripheral_charValueChangeCB // Simple GATT Characteristic value change callback
//};

/****** Dashboard Profile Callbacks    *****/
static DashboardCBs_t DashboardCBs =
{
     SimplePeripheral_dashboardCB
};
/****** Battery Profile Callbacks    *****/
static BatteryCBs_t BatteryCBs =
{
     SimplePeripheral_batteryCB
};
/****** Controller Profile Callbacks    *****/
static ControllerCBs_t ControllerCBs =
{
     SimplePeripheral_controllerCB
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      simple_peripheral_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void simple_peripheral_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

#ifdef PTM_MODE
/*********************************************************************
* @fn      simple_peripheral_handleNPIRxInterceptEvent
*
* @brief   Intercept an NPI RX serial message and queue for this application.
*
* @param   pMsg - a NPIMSG_msg_t containing the intercepted message.
*
* @return  none.
*/
void simple_peripheral_handleNPIRxInterceptEvent(uint8_t *pMsg)
{
 // Send Command via HCI TL
 HCI_TL_SendToStack(((NPIMSG_msg_t *)pMsg)->pBuf);

 // Free the NPI message
 NPITask_freeNpiMsg(pMsg);
}

/*********************************************************************
* @fn      simple_peripheral_sendToNPI
*
* @brief   Create an NPI packet and send to NPI to transmit.
*
* @param   buf - pointer HCI event or data.
*
* @param   len - length of buf in bytes.
*
* @return  none
*/
static void simple_peripheral_sendToNPI(uint8_t *buf, uint16_t len)
{
 npiPkt_t *pNpiPkt = (npiPkt_t *)ICall_allocMsg(sizeof(npiPkt_t) + len);

 if (pNpiPkt)
 {
   pNpiPkt->hdr.event = buf[0]; //Has the event status code in first byte of payload
   pNpiPkt->hdr.status = 0xFF;
   pNpiPkt->pktLen = len;
   pNpiPkt->pData  = (uint8 *)(pNpiPkt + 1);

   memcpy(pNpiPkt->pData, buf, len);

   // Send to NPI
   // Note: there is no need to free this packet.  NPI will do that itself.
   NPITask_sendToHost((uint8_t *)pNpiPkt);
 }
}
#endif // PTM_MODE

/*********************************************************************
 * @fn      SimplePeripheral_createTask
 *
 * @brief   Task creation function for the Simple Peripheral.
 */
#define SNV_NV_ID80   0x80

// ptrUDBuffer holds the address (pointer) to UDBuffer, which is read from SNV
uint32_t (*ptrUDBuffer)[SNV_BUFFER_SIZE] = {0};     // pointer register to UDBuffer array

// snv_internal will store the dynamic data of the device that will be written to SNV at shut down - replacing the data in UDBuffer
uint32_t snv_internal_80[SNV_BUFFER_SIZE] ={0};

uint8 snv_status = SUCCESS; // SUCCESS is defined as 0

void SimplePeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = spTaskStack;
  taskParams.stackSize = SP_TASK_STACK_SIZE;
  taskParams.priority = SP_TASK_PRIORITY;
  Task_construct(&spTask, SimplePeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.s
 */
uint8_t             sp_opcode = 0xFF;
uint8_t             sp_advertiseFlag = 0;

static uint8_t      *ptr_GAPflag;               // pointer register for GAPflag
profileCharVal_t    *ptr_sp_profileCharVal;     // pointer register for profileCharVal
static uint8        *ptr_charVal;               // pointer register for charVal

static void SimplePeripheral_init(void)
{
  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- init ", SP_TASK_PRIORITY);

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  // Set device's Sleep Clock Accuracy
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
  HCI_EXT_SetSCACmd(500);
#endif // (CENTRAL_CFG | PERIPHERAL_CFG)
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

  // Create one-shot clock for internal periodic events.
  Util_constructClock(&clkPeriodic, SimplePeripheral_clockHandler,
                      SP_PERIODIC_EVT_PERIOD, 0, false, (UArg)&argPeriodic);

  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

    // Pass all parameter update requests to the app for it to decide
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
  }

  // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
  // section in the User's Guide
  setBondManagerParameters();

/*********************  Added Services  ***************************************/
  // Initialize GATT attributes
  GGS_AddService(GAP_SERVICE);                  // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);    // GATT Service
  DevInfo_AddService();                         // Device Information Service

  /******* Initialize Profiles and Profile Characteristic Value pointer registers *********/
    Dashboard_profile_init();
    Battery_profile_init();
    Controller_profile_init();
    profile_charVal_init();

    ptr_sp_profileCharVal = profile_charVal_profileCharValRegister();

  /******************* end pointer registers **************/

/*********************  Added Services  ***************************************/
    Dashboard_AddService();                       // Dashboard Profile Service
    Battery_AddService();                         // Battery Profile Service
    Controller_AddService();                      // Controller Profile Service

    /********** send and register the pointer of sp_opcode to multi_purpose_button.c  *******/
    ptr_GAPflag = mpb_registeropcode(&sp_opcode, &sp_advertiseFlag);    // obtain pointer to GAPflag from mpb.c
    gpt_registeropcode(&sp_opcode, &sp_advertiseFlag);                  // register sp_code and sp_advertiseFlag at led display
    led_display_opcodeRegister(&sp_opcode);                             // register sp_code at led display

  /*****************  Setup initial Profile Characteristic Values  ************************/
  // For more information, see the GATT and GATTServApp sections in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  {
      /******* only if the initial characteristic values need changing  *********/
//      uint32_t    ADCounter_init_val = 0;
//      uint16_t    power_on_time_init_val = 60;
//      uint8_t     error_code_init_val = 0;
//      uint8_t     speed_mode_init_val = 0;
//      uint8_t     light_mode_init_val = 2;
//      uint8_t     light_status_init_val = 0;
//      uint32_t    device_uptime_init_val = 1355;

      /*********************** for demo purposes only **************************/
  //    uint8_t db_charValue2[DASHBOARD_LIGHT_STATUS_LEN] = {1};
  //    uint8_t db_charValue3[DASHBOARD_LIGHT_MODE_LEN] = {1};
  //    Dashboard_SetParameter(DASHBOARD_LIGHT_STATUS, DASHBOARD_LIGHT_STATUS_LEN,
  //                           db_charValue2);
  //    Dashboard_SetParameter(DASHBOARD_LIGHT_MODE, DASHBOARD_LIGHT_MODE_LEN,
  //                           &db_charValue3);
//      uint8 db_charValue4[DASHBOARD_POWER_ON_TIME_LEN] = {(power_on_time_init_val & 0xFF), ((power_on_time_init_val >> 8) & 0xFF)};
//      Dashboard_SetParameter(DASHBOARD_POWER_ON_TIME, DASHBOARD_POWER_ON_TIME_LEN,
//                             db_charValue4);
  }

  /************** Register callback with Dashboard profile, Battery profile, Controller profile ***************/

  Dashboard_RegisterAppCBs(&DashboardCBs);          // send pointer to DashboardCBs to dashboard profile
  Battery_RegisterAppCBs( &BatteryCBs );            // there is no characteristic that require callback
  Controller_RegisterAppCBs( &ControllerCBs );      // there is no characteristic that require callback

  // Start Bond Manager and register callback
//  VOID GAPBondMgr_Register(&SimplePeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  {
    // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    // Some brand smartphone is essentially needing 251/2120, so we set them here.
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    // This API is documented in hci.h
    // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client
  GATT_InitClient();

  // Initialize Connection List
  SimplePeripheral_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- call GAP_DeviceInit", GAP_PROFILE_PERIPHERAL);
  //Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, &pRandomAddress);

  // Initialize array to store connection handle and RSSI values
  SimplePeripheral_initPHYRSSIArray();

#ifdef PTM_MODE
  // Intercept NPI RX events.
  NPITask_registerIncomingRXEventAppCB(simple_peripheral_handleNPIRxInterceptEvent, INTERCEPT);

  // Register for Command Status information
  HCI_TL_Init(NULL, (HCI_TL_CommandStatusCB_t) simple_peripheral_sendToNPI, NULL, selfEntity);

  // Register for Events
  HCI_TL_getCmdResponderID(ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, selfEntity));

  // Inform Stack to Initialize PTM
  HCI_EXT_EnablePTMCmd();
#endif // PTM_MODE

}

/*********************************************************************
 * @fn      SimplePeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple Peripheral.
 *
 * @param   a0, a1 - not used.
 */
static uint8_t      sp_initComplete_flag = GPT_INACTIVE;
static bool         *ptr_sp_POWER_ON;       // register for POWER_ON status
static sysFatalError_t *ptr_sysFatalError;
static uint8_t      *ptr_snvWriteFlag;
static uint8_t      snv_writeComplete_flag = 0;
uint8_t             snv_reset = 0;   // if snv_reset = 0, it means the non-volatile storage was NOT reset by the Firmware.
uint8_t            sp_counter = 0;
static uint8_t     sp_i2cOpenStatus;
static bStatus_t    check_enableStatus;
static uint8_t      *ptr_sp_dashboardErrorCodePriority;

static uint8_t      speedmode_lock_status = 0;  //Chee added 20250110

uint8_t how_boot = 0xFF;

static void SimplePeripheral_taskFxn(UArg a0, UArg a1)
{
  /*******   Initialize application   ********/
      Boot_Init();                                                   //Start Boot Process

      PowerModeStatusManager();

      gpt_InitComplFlagRegister(&sp_initComplete_flag);             // send pointer to sp_initComplet_flag to gpt.c

      pot_InitComplFlagRegister(&sp_initComplete_flag);

      ptrUDBuffer = snv_internal_setReadBuffer(&snv_internal_80);   // pass the pointer to snv_internal_80 to snv_internal and get the pointer to UDArray in return

      data_analytics_setSNVBufferRegister(&snv_internal_80);        // pass the pointer to snv_internal_80 to data_analytics

      mpb_speedmodeLockStatusRegister(&speedmode_lock_status);     // pass the pointer to speedmode_lock_status to MPB   //Chee added 20250110

      ptr_sp_POWER_ON = mpb_powerOnRegister();                      // call mpb_powerOnRegister and get the pointer to power_On in return

      ptr_sysFatalError = UDHAL_sysFatalErrorRegister();            // call UDHAL to get pointer to sysFatalError

      led_display_advertiseFlagRegister(&sp_advertiseFlag);         // pass pointer to sp_advertiseFlag to led display

      ptr_snvWriteFlag = gpt_snvWriteFlageRegister();                   // call snvWriteFlagRegister to get pointer to snvWriteFlag

      pot_snvWriteCompleteFlag_register(&snv_writeComplete_flag);

      ptr_sp_dashboardErrorCodePriority = bat_dashboardErrorCodePriorityRegister(); // call bat_dashboardErrorCodePriorityRegister to get pointer to sp_dashboardErrorCodePriority

      SimplePeripheral_init();

/********************************************************
*            Non-Volatile Storage via SNV
* ******************************************************/
  /* clear and set Buffer SNV_NV_ID80 for the first time */
      /*  Firstly, read the memory in SNV_NV_ID80 sector  */
      snv_status = osal_snv_read(SNV_NV_ID80, sizeof(snv_internal_80), (uint32_t *)snv_internal_80);

      /*  Then, RESET NVS if the following conditions (RESET CODES) are not met */
      if ((snv_internal_80[SNV_BUFFER_SIZE - 6] != RESETCODE01) && (snv_internal_80[SNV_BUFFER_SIZE - 5] != RESETCODE02))
      {
#ifdef NEW_RESET_NVS        // defined in simple peripheral.h
          snv_internal_resetSNVdata();      // options available: (1) zero reset or (2) dummy reset
          snv_status = osal_snv_write(SNV_NV_ID80, sizeof(snv_internal_80), ptrUDBuffer);
          snv_reset = 1;    // if snv_reset = 1, it means the non-volatile storage was reset by the Firmware.
#endif // NEW_RESET_NVS
      }

      /* Override data if HARD_OVERRIDE_NVS is defined */
#ifdef HARD_OVERRIDE_NVS    // defined in simple peripheral.h
      snv_internal_resetSNVdata();      // option available for zero reset or dummy reset
      snv_status = osal_snv_write(SNV_NV_ID80, sizeof(snv_internal_80), ptrUDBuffer);
#endif // HARD_OVERRIDE_NVS

  /****    read memory again after reset     ****/
      snv_status = osal_snv_read(SNV_NV_ID80, sizeof(snv_internal_80), (uint32_t *)snv_internal_80);

/**************** End Non-Volatile Storage via SNV *************************/

    /**************************************************************************
     *  Initialize UDHAL (GPIO, I2C, ADC, UART)
     * return the status of I2C open
     * return 0 = I2C open failed
     * return 1 = I2c open successful
     *
     * UDHAL_init must be called before Hardware and Applications
     **************************************************************************/
      sp_i2cOpenStatus = UDHAL_init();

      /* Set up initial data values after reading from NVS memory  */
      brake_and_throttle_setSpeedMode(snv_internal_80[SNV_BUFFER_SIZE - 4]);   // set initial speedMode = snv_internal_80[28] in brake_and_throttle.c
      pot_setDeviceUpTime(snv_internal_80[SNV_BUFFER_SIZE - 1]);    // read and set initial device uptime in power_on_time.c

      /********* motor_contol_init must be called before data_analytics_init **********/
      STM32MCP_init();  // -> timer2_Start
      motor_control_init();

      /********* led_display_init() must be called before data_analytics_init(),
       * brake_and_throttle_init() and lights_init() ********************************/
      if (!(ptr_sysFatalError->I2Cfailure))
      {  /**** if i2c opened successfully, i.e. sp_sysFatalError = 0 (no error)  ****/
          /* Both ALS and led_display require successful open of i2c to function  */
          led_display_init();
      }

      data_analytics_init();

      brake_and_throttle_init();

      lights_init(sp_i2cOpenStatus, ptr_sysFatalError->UARTfailure, snv_internal_80[SNV_BUFFER_SIZE - 2]);       // set initial light Mode = snv_internal_80[30]

      /***** SYSTEM FATAL ERROR Check. Note SUCCESS = 0, FAILURE = 1 ******************/
      /*****            Must be activated after led_display_init()                   *****/
      if ((ptr_sysFatalError->UARTfailure) || (ptr_sysFatalError->I2Cfailure) ||
              (ptr_sysFatalError->PWMfailure) || (ptr_sysFatalError->ADCfailure) || (ptr_sysFatalError->Otherfailure))
      {
          led_display_ErrorPriority(SYS_FATAL_ERROR_PRIORITY);

          if (*ptr_sp_dashboardErrorCodePriority > SYS_FATAL_ERROR_PRIORITY)
          {
              *ptr_sp_dashboardErrorCodePriority = SYS_FATAL_ERROR_PRIORITY;
              ptr_charVal = (ptr_sp_profileCharVal->ptr_dash_charVal->ptr_dashErrorCode);
              profile_setCharVal(ptr_charVal, DASHBOARD_ERROR_CODE_LEN, SYS_FATAL_ERROR_CODE);
          }

          /**** system (CPU) fatal error has occurred
           *    Device should be instructed to reset / shut down
           *    Contact technical customer service if error persist
           *****/
          //      Go to error handling

      }

    /**************************************************************************************************************************
    *   Simple peripheral must finish initialization before GPT task functions initialize.
    *  sp_initComplete_flag is a flag for GPT that enables codes to execute only after SP completed initialization
    ****************************/
    sp_initComplete_flag = GPT_ACTIVE;

    if(HowToBoot() == 0x00)
    {
        mpb_bootAlert(400,0x00);
        sendBootMessage();
    }
    else
    {
        mpb_bootAlert(150,0x01);
    }

    /****** Simple Peripheral Application main loop  ******/
    for (;;)
    {
        uint32_t events;
        sp_counter++;

        // Waits for an event to be posted associated with the calling thread.
        // Note that an event associated with a thread is posted when a
        // message is queued to the message receive queue of the thread
        events = Event_pend(syncEvent, Event_Id_NONE, SP_ALL_EVENTS,
                            ICALL_TIMEOUT_FOREVER);

        /***** Handles GapAdv_enable when user turns on BLE from MPB    *****/
        if( (sp_opcode == GAP_LINK_TERMINATED_EVENT) || (sp_opcode == GAP_DEVICE_INIT_DONE_EVENT) ) //will enable BLE adv if sp_opcode = GAP_LINK_TERMINATED_EVENT or after initiation
        {
            if (*ptr_GAPflag == 1)
            {         // if BLE is not advertising, do the following
                if (!sp_advertiseFlag)             // if advertising request = 1, enable advertise
                {
                    check_enableStatus = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_DURATION , SP_ADVERTISING_TIMEOUT);
                    *ptr_GAPflag = 0;    // whenever advertising is enabled - set GAPflag to 0
                }
            }
        }
        /***** Terminates BLE when user opted to turn off BLE from MPB  -  added 2024/12/02  *****/
        else if( (sp_opcode == GAP_LINK_ESTABLISHED_EVENT) || (sp_opcode == GAP_LINK_PARAM_UPDATE_EVENT) )
        {
            if (*ptr_GAPflag == 2)
            {
                GAP_TerminateLinkReq(advHandleLegacy, HCI_DISCONNECT_REMOTE_USER_TERM);
                *ptr_GAPflag = 0;
            }
        } // End added 2024/12/02

        if (events)
        {
              ICall_EntityID dest;
              ICall_ServiceEnum src;
              ICall_HciExtEvt *pMsg = NULL;

              // Fetch any available messages that might have been sent from the stack
              if (ICall_fetchServiceMsg(&src, &dest,
                                        (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
              {
                    uint8 safeToDealloc = TRUE;

                    if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
                    {
                          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

                          // Check for BLE stack events first
                          if (pEvt->signature != 0xffff)
                          {
                            // Process inter-task message
                            safeToDealloc = SimplePeripheral_processStackMsg((ICall_Hdr *)pMsg);
                          }
                    }

                    if (pMsg && safeToDealloc)
                    {
                      ICall_freeMsg(pMsg);
                    }
              }

              // If RTOS queue is not empty, process app message.
              if (events & SP_QUEUE_EVT)
              {
                while (!Queue_empty(appMsgQueueHandle))
                {
                  spEvt_t *pMsg = (spEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
                  if (pMsg)
                  {
                    // Process message.
                    SimplePeripheral_processAppMsg(pMsg);

                    // Free the space from the message.
                    ICall_free(pMsg);
                  }
                }
              }
        }

        /***************************************************************************************************
         *  Handles POWER OFF, write to NVS and shut down protocols when user POWER OFF the device from MPB
         *  ************************************************************************************************/
        if (!(*ptr_sp_POWER_ON)) // i.e. if POWER_ON == 0 (i.e. power off)
        {
            /*** If power off, the following actions are executed before shut down  ***/
            // if BLE is connected/linked -> terminate all connections
            if( (sp_opcode == GAP_LINK_ESTABLISHED_EVENT) || (sp_opcode == GAP_LINK_PARAM_UPDATE_EVENT) )
            {
            /* See https://dev.ti.com/tirex/explore/node?node=A__AfmOXSEGNa9-PneA2DxLYw__com.ti.SIMPLELINK_ACADEMY_CC13XX_CC26XX_SDK__AfkT0vQ__LATEST
             * reason option: see hci.h
             *    HCI_DISCONNECT_REMOTE_USER_TERM = HCI_ERROR_CODE_REMOTE_USER_TERM_CONN = 0x13
             */
                GAP_TerminateLinkReq(advHandleLegacy, HCI_DISCONNECT_REMOTE_USER_TERM);
            }

               if (*ptr_snvWriteFlag)   // remain in FOR loop until snvWriteFlag = 1
                {
                    /****   write snv_internal_80 to snv    *******/
                    snv_status = osal_snv_write(SNV_NV_ID80, sizeof(snv_internal_80), &snv_internal_80);
                    snv_writeComplete_flag = 1; // signify that osal_snv_write is complete.

                    break;    // break out of main FOR loop when snv_write is completed.
                }
        }
    }   // main FOR loop

}


/*********************************************************************
 * @fn      SimplePeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  // Always dealloc pMsg unless set otherwise
  uint8_t safeToDealloc = TRUE;

  BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : Stack msg status=%d, event=0x%x\n", pMsg->status, pMsg->event);

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimplePeripheral_processGapMessage((gapEventHdr_t*) pMsg);
      break;
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimplePeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;
    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Events here
        {
          SimplePeripheral_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
          break;
        }
        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;
        // HCI Commands Events
        case HCI_COMMAND_STATUS_EVENT_CODE:
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;
          switch ( pMyMsg->cmdOpcode )
          {
            case HCI_LE_SET_PHY:
            {
              SimplePeripheral_updatePHYStat(HCI_LE_SET_PHY, (uint8_t *)pMsg);
              break;
            }

            default:
              break;
          }
          break;
        }
        // LE Events
        case HCI_LE_EVENT_CODE:
        {
          hciEvt_BLEPhyUpdateComplete_t *pPUC =
            (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

          // A Phy Update Has Completed or Failed
          if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
          {
            SimplePeripheral_updatePHYStat(HCI_BLE_PHY_UPDATE_COMPLETE_EVENT, (uint8_t *)pMsg);
          }
          break;
        }
        default:
          break;
      }
      break;
    }
    default:
      // do nothing
      break;
  }

#ifdef PTM_MODE
  // Check for NPI Messages
  hciPacket_t *pBuf = (hciPacket_t *)pMsg;

  // Serialized HCI Event
  if (pBuf->hdr.event == HCI_CTRL_TO_HOST_EVENT)
  {
    uint16_t len = 0;

    // Determine the packet length
    switch(pBuf->pData[0])
    {
      case HCI_EVENT_PACKET:
        len = HCI_EVENT_MIN_LENGTH + pBuf->pData[2];
        break;

      case HCI_ACL_DATA_PACKET:
        len = HCI_DATA_MIN_LENGTH + BUILD_UINT16(pBuf->pData[3], pBuf->pData[4]);
        break;

      default:
        break;
    }

    // Send to Remote Host.
    simple_peripheral_sendToNPI(pBuf->pData, len);

    // Free buffers if needed.
    switch (pBuf->pData[0])
    {
      case HCI_ACL_DATA_PACKET:
      case HCI_SCO_DATA_PACKET:
        BM_free(pBuf->pData);
      default:
        break;
    }
  }
#endif // PTM_MODE

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimplePeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimplePeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg)
{
  bool dealloc = TRUE;

  if (pMsg->event <= APP_EVT_EVENT_MAX)
  {
    BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=%s\n", 0, appEventStrings[pMsg->event]);
  }
  else
  {
    BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=0x%x\n", 0, pMsg->event);
  }

  switch (pMsg->event)
  {
    case SP_STATE_CHANGE_EVT:
//      SimplePeripheral_processStateChangeEvt((gaprole_States_t)pMsg->hdr.state);
    break;

    case SP_CHAR_CHANGE_EVT:
      SimplePeripheral_processCharValueChangeEvt(*(uint8_t*)(pMsg->pData));
      break;

    case SP_KEY_CHANGE_EVT:

      break;

    case SP_ADV_EVT:
      SimplePeripheral_processAdvEvent((spGapAdvEventData_t*)(pMsg->pData));
      break;

    case SP_PAIR_STATE_EVT:
      SimplePeripheral_processPairState((spPairStateData_t*)(pMsg->pData));
      break;

    case SP_PASSCODE_EVT:
      SimplePeripheral_processPasscode((spPasscodeData_t*)(pMsg->pData));
      break;

    case SP_PERIODIC_EVT:
      SimplePeripheral_performPeriodicTask();
      break;

    case SP_READ_RPA_EVT:
      SimplePeripheral_updateRPA();
      break;

    case SP_SEND_PARAM_UPDATE_EVT:
    {
      // Extract connection handle from data
      uint16_t connHandle = *(uint16_t *)(((spClockEventData_t *)pMsg->pData)->data);

      SimplePeripheral_processParamUpdate(connHandle);

      // This data is not dynamically allocated
      dealloc = FALSE;
      break;
    }

    case SP_CONN_EVT:
      SimplePeripheral_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));
      break;

    default:
      // Do nothing.
      break;
  }

  // Free message data if it exists and we are to dealloc
  if ((dealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
uint8_t sp_periodic_evt_counter = 0;
uint8_t sp_periodic_evt_counter2 = 0;

static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg)
{
  switch(pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      bStatus_t status = FAILURE;

      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;
      sp_opcode = pPkt->opcode;

      if(pPkt->hdr.status == SUCCESS)
      {
        // Store the system ID
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = pPkt->devAddr[0];
        systemId[1] = pPkt->devAddr[1];
        systemId[2] = pPkt->devAddr[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = pPkt->devAddr[5];
        systemId[6] = pPkt->devAddr[4];
        systemId[5] = pPkt->devAddr[3];

        // Set Device Info Service Parameter
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_DEVICE_INIT_DONE_EVENT", 0);
        // Setup and start Advertising
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- call GapAdv_create set=%d,%d\n", 0, 0);

        // Create Advertisement set #1 and assign handle
        status = GapAdv_create(&SimplePeripheral_advCallback, &advParams1,
                               &advHandleLegacy);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load advertising data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advData1), advData1);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load scan response data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanResData1), scanResData1);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Set event mask for set #1
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        /**** Enable legacy advertising for set #1 ****/
        /****   1: Comment out "GapAdv_enable" to NOT activate BLE on Startup   ****/
        /****   2: Un-comment "GapAdv_enable" to activate BLE on Startup        ****/
//        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_DURATION , SP_ADVERTISING_TIMEOUT); // advertise for a duration = SP_ADVERTISING_TIMEOUT
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);
        check_enableStatus = status;
        *ptr_GAPflag = 0;    // whenever Advertising is enable, set GAPflag to 0

        BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GapAdv_enable", 0);

//        *ptr_GAPflag = 0;

        if (addrMode > ADDRMODE_RANDOM)
        {
          SimplePeripheral_updateRPA();

          // Create one-shot clock for RPA check event.
          Util_constructClock(&clkRpaRead, SimplePeripheral_clockHandler,
                              READ_RPA_PERIOD, 0, true,
                              (UArg) &argRpaRead);
        }
      }

      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:    // it goes here whenever a link is established
    {
      gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

      sp_opcode = pPkt->opcode;

      BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_LINK_ESTABLISHED_EVENT", 0);
      uint8_t numActive = linkDB_NumActive();

      if (pPkt->hdr.status == SUCCESS)
      {
        // Add connection to list and start RSSI
        SimplePeripheral_addConn(pPkt->connectionHandle);

        // Start Periodic Clock.
        Util_startClock(&clkPeriodic);
      }

      if ((numActive < MAX_NUM_BLE_CONNS) && (autoConnect == AUTOCONNECT_DISABLE))
      {
          // Start advertising since there is room for more connections
          check_enableStatus = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_DURATION , SP_ADVERTISING_TIMEOUT);

          *ptr_GAPflag = 0;  // whenever advertising is enabled, set GAPflag to 0

      }
      else
      {
          /*    Stop advertising since there is no room for more connections   */

          GapAdv_disable(advHandleLegacy);
      }

      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;
      sp_opcode = pPkt->opcode;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();

      // Remove the connection from the list and disable RSSI if needed
      SimplePeripheral_removeConn(pPkt->connectionHandle);

      // If no active connections
      if (numActive == 0)
      {
        // Stop periodic clock
        Util_stopClock(&clkPeriodic);
      }

      BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : GAP msg: status=%d, opcode=%s\n", 0, "GAP_LINK_TERMINATED_EVENT");
      // Start advertising since there is room for more connections

      /***** Disabled automatic advertising after disconnection with device  ****/
//      status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
//
//      *ptr_GAPflag = 0;  // whenever advertising is enabled, set GAPflag to 0
//      check_enableStatus = status;

      sp_periodic_evt_counter = 0;
      sp_periodic_evt_counter2 = 0;
      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      gapUpdateLinkParamReqReply_t rsp;

      gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsg;
      sp_opcode = pReq->opcode;  //https://software-dl.ti.com/simplelink/esd/simplelink_cc2640r2_sdk/4.20.00.04/exports/docs/blestack/ble_user_guide/doxygen/ble/html/group___g_a_p___event___i_ds.html#ga565d5dfa9c10e56bb5b3d768dcb7027c

      rsp.connectionHandle = pReq->req.connectionHandle;
      rsp.signalIdentifier = pReq->req.signalIdentifier;

      // Only accept connection intervals with peripheral latency of 0
      // This is just an example of how the application can send a response
      if(pReq->req.connLatency == 0)
      {
        rsp.intervalMin = pReq->req.intervalMin;
        rsp.intervalMax = pReq->req.intervalMax;
        rsp.connLatency = pReq->req.connLatency;
        rsp.connTimeout = pReq->req.connTimeout;
        rsp.accepted = TRUE;
      }
      else
      {
        rsp.accepted = FALSE;
      }

      // Send Reply
      VOID GAP_UpdateLinkParamReqReply(&rsp);

      break;
    }

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;
      sp_opcode = pPkt->opcode;

      // Get the address from the connection handle
      linkDBInfo_t linkInfo;
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

      // Check if there are any queued parameter updates
      spConnHandleEntry_t *connHandleEntry = (spConnHandleEntry_t *)List_get(&paramUpdateList);
      if (connHandleEntry != NULL)
      {
        // Attempt to send queued update now
        SimplePeripheral_processParamUpdate(connHandleEntry->connHandle);

        // Free list element
        ICall_free(connHandleEntry);
      }

      break;
    }

#if defined ( NOTIFY_PARAM_UPDATE_RJCT )
    case GAP_LINK_PARAM_UPDATE_REJECT_EVENT:
    {
      linkDBInfo_t linkInfo;
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;
      sp_opcode = pPkt->opcode;  // https://software-dl.ti.com/simplelink/esd/simplelink_cc2640r2_sdk/4.20.00.04/exports/docs/blestack/ble_user_guide/doxygen/ble/html/group___g_a_p___event___i_ds.html#ga565d5dfa9c10e56bb5b3d768dcb7027c

      // Get the address from the connection handle
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);
      break;
    }
#endif

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void SimplePeripheral_charValueChangeCB(uint8_t paramId)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = paramId;

    if (SimplePeripheral_enqueueMsg(SP_CHAR_CHANGE_EVT, pValue) != SUCCESS)
    {
      ICall_free(pValue);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_dashboardCB
 *
 * @brief   When the client (The mobile app) writes the Dashboard GATT server
 *          It will post the paramID of the changed characteristic, and Set the gatt due to client Callback
 *
 *          G-Link v1 allows the following characteristics to be changed from the client (mobile app)
 *          - light mode (paramID 3)
 *
 * @param   paramID: the paramID of the characteristics
 *
 * @return  TRUE or FALSE
 */
static void SimplePeripheral_dashboardCB(uint8_t paramID)
{
    switch(paramID)
    {
    // whenever user tabs the light mode icon on the mobile app, SimplePeripheral_dashboardCB will be triggered
    // the dashboard is instructed here to toggle the light mode by calling lights_lightModeChange()
    case DASHBOARD_LIGHT_MODE: // whenever user tabs the light mode icon on the mobile app, SimplePeripheral_dashboardCB will toggle the light mode by calling lightControl_change()
        {
            led_display_setLightMode( lights_lightModeChange() );               // set and update the light mode indicator on dashboard
            break;
        }
        // whenever user tabs the speed mode icon on the mobile app, SimplePeripheral_dashboardCB will be triggered
        // the dashboard is instructed here to toggle between lock and unlock of the speed mode
    case DASHBOARD_SPEED_MODE:
        {
            if (!speedmode_lock_status){  //Chee added 20250110
                speedmode_lock_status = 1;  //Chee added 20250110
            }  //Chee added 20250110
            else{  //Chee added 20250110
                speedmode_lock_status = 0;  //Chee added 20250110
            }  //Chee added 20250110
            bat_dashboard_speedmode_service();
            break;
        }
    default:
        break;
    }
}

static void SimplePeripheral_batteryCB(uint8_t paramID)
{
    /***  Currently no call back from battery profile  ****/
}

static void SimplePeripheral_controllerCB(uint8_t paramID)
{
    /***  Currently no call back from controller profile  ****/

}
/*********************************************************************
 * @fn      SimplePeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramId)
{
  uint8_t newValue;

  /*********************************************************************
   *                 For Write Permit
   *
   *    Whenever the client receives an input from the user, a callback
   *    is executed.  GetParameter() retrieves the value/data, that
   *    allow us to do something with it
   *
   **********************************************************************/
  switch(paramId)
  {
      /* Whenever a new value is written to DASHBOARD LIGHT MODE on the APP
       * this processCharValueChangeEvt reads the new value on the App */
  case DASHBOARD_LIGHT_MODE:
  {
//      Dashboard_GetParameter(DASHBOARD_LIGHT_MODE, &newValue); // the value is not relevant
    break;
  }
  /* Whenever a new value is written to DASHBOARD SPEED MODE on the APP
   * this processCharValueChangeEvt is triggered to read the new value on the App */
  case DASHBOARD_SPEED_MODE:
  {
//      Dashboard_GetParameter(DASHBOARD_SPEED_MODE, &newValue); // the value is not relevant
    break;
    }
    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_performPeriodicTask(void)
{
    uint8_t arrayToCopy1[1];
    uint8_t arrayToCopy2[2];
    uint8_t arrayToCopy4[4];

  /****************************************************************
   *                 For Notify Permit
   ****************************************************************/
  /******** The following Characteristics refresh every SP_PERIODIC_EVT_TIME **********/
    if (Controller_GetParameter(CONTROLLER_MOTOR_RPM, arrayToCopy2) == SUCCESS)
    {
        Controller_SetParameter(CONTROLLER_MOTOR_RPM, CONTROLLER_MOTOR_RPM_LEN, arrayToCopy2);
    }

    if (Controller_GetParameter(CONTROLLER_MOTOR_SPEED, arrayToCopy2) == SUCCESS)
    {
        Controller_SetParameter(CONTROLLER_MOTOR_SPEED, CONTROLLER_MOTOR_SPEED_LEN, arrayToCopy2);
    }

    if (Dashboard_GetParameter(DASHBOARD_LIGHT_STATUS, arrayToCopy1) == SUCCESS)
    {
        Dashboard_SetParameter(DASHBOARD_LIGHT_STATUS, DASHBOARD_LIGHT_STATUS_LEN, arrayToCopy1);
    }

    if (Dashboard_GetParameter(DASHBOARD_LIGHT_MODE, arrayToCopy1) == SUCCESS)
    {
        Dashboard_SetParameter(DASHBOARD_LIGHT_MODE, DASHBOARD_LIGHT_MODE_LEN, arrayToCopy1);
    }

    if (Battery_GetParameter(BATTERY_BATTERY_CURRENT, arrayToCopy2) == SUCCESS)
    {
        Battery_SetParameter(BATTERY_BATTERY_CURRENT, BATTERY_BATTERY_CURRENT_LEN, arrayToCopy2);
    }

    if (Battery_GetParameter(BATTERY_BATTERY_VOLTAGE, arrayToCopy2) == SUCCESS)
    {
        Battery_SetParameter(BATTERY_BATTERY_VOLTAGE, BATTERY_BATTERY_VOLTAGE_LEN, arrayToCopy2);
    }

    /******** Speed and rpm Notification - Refresh every 7 x SP_PERIODIC_EVT_TIME **********/
    if (sp_periodic_evt_counter == SP_PERIODIC_EVT_COUNT1)   // Notification is performed once every 4th sp_periodic_evt_counts
    {
        /******** Dashboard Profile Notification **********/
        if (Dashboard_GetParameter(DASHBOARD_ERROR_CODE, arrayToCopy1) == SUCCESS)
        {
            Dashboard_SetParameter(DASHBOARD_ERROR_CODE, DASHBOARD_ERROR_CODE_LEN, arrayToCopy1);
        }

        if (Dashboard_GetParameter(DASHBOARD_SPEED_MODE, arrayToCopy1) == SUCCESS)
        {
            Dashboard_SetParameter(DASHBOARD_SPEED_MODE, DASHBOARD_SPEED_MODE_LEN, arrayToCopy1);
        }

        if (Dashboard_GetParameter(DASHBOARD_POWER_ON_TIME, arrayToCopy2) == SUCCESS)
        {
        /* Call to set that value of DASHBOARD_POWER_ON_TIME in the profile.
         * Note that if notifications of the DASHBOARD_POWER_ON_TIME have been
         * enabled by a GATT client device, then a notification will be sent
         * every time this function is called. */
            Dashboard_SetParameter(DASHBOARD_POWER_ON_TIME, DASHBOARD_POWER_ON_TIME_LEN, arrayToCopy2);
        }

        if (Dashboard_GetParameter(DASHBOARD_ADCOUNTER, arrayToCopy4) == SUCCESS)
        {
            Dashboard_SetParameter(DASHBOARD_ADCOUNTER, DASHBOARD_ADCOUNTER_LEN, arrayToCopy4);
        }

        if (Dashboard_GetParameter(DASHBOARD_DEVICE_UPTIME, arrayToCopy4) == SUCCESS)
        {
            Dashboard_SetParameter(DASHBOARD_DEVICE_UPTIME, DASHBOARD_DEVICE_UPTIME_LEN, arrayToCopy4);
        }

        /******** Battery Profile Notification **********/
        if (Battery_GetParameter(BATTERY_BATTERY_LEVEL, arrayToCopy1) == SUCCESS)
        {
            Battery_SetParameter(BATTERY_BATTERY_LEVEL, BATTERY_BATTERY_LEVEL_LEN, arrayToCopy1);
        }

        //  if (Battery_GetParameter(BATTERY_BATTERY_VOLTAGE, arrayToCopy2) == SUCCESS)
        //    {
        //      Battery_SetParameter(BATTERY_BATTERY_VOLTAGE, BATTERY_BATTERY_VOLTAGE_LEN, arrayToCopy2);
        //    }

        if (Battery_GetParameter(BATTERY_BATTERY_TEMPERATURE, arrayToCopy1) == SUCCESS)
        {
            Battery_SetParameter(BATTERY_BATTERY_TEMPERATURE, BATTERY_BATTERY_TEMPERATURE_LEN, arrayToCopy1);
        }

        if (Battery_GetParameter(BATTERY_BATTERY_ERROR_CODE, arrayToCopy1) == SUCCESS)
        {
            Battery_SetParameter(BATTERY_BATTERY_ERROR_CODE, BATTERY_BATTERY_ERROR_CODE_LEN, arrayToCopy1);
        }

        if (Battery_GetParameter(BATTERY_BATTERY_STATUS, arrayToCopy1) == SUCCESS)
        {
            Battery_SetParameter(BATTERY_BATTERY_STATUS, BATTERY_BATTERY_STATUS_LEN, arrayToCopy1);
        }

        /******** Controller Profile Notification **********/
        if (Controller_GetParameter(CONTROLLER_VOLTAGE, arrayToCopy2) == SUCCESS)
        {
            Controller_SetParameter(CONTROLLER_VOLTAGE, CONTROLLER_VOLTAGE_LEN, arrayToCopy2);
        }

        //  if (Controller_GetParameter(CONTROLLER_CURRENT, arrayToCopy2) == SUCCESS)
        //    {
        //      Controller_SetParameter(CONTROLLER_CURRENT, CONTROLLER_CURRENT_LEN, arrayToCopy2);
        //    }

        if (Controller_GetParameter(CONTROLLER_HEAT_SINK_TEMPERATURE, arrayToCopy1) == SUCCESS)
        {
            Controller_SetParameter(CONTROLLER_HEAT_SINK_TEMPERATURE, CONTROLLER_HEAT_SINK_TEMPERATURE_LEN, arrayToCopy1);
        }

        if (Controller_GetParameter(CONTROLLER_ERROR_CODE, arrayToCopy1) == SUCCESS)
        {
            Controller_SetParameter(CONTROLLER_ERROR_CODE, CONTROLLER_ERROR_CODE_LEN, arrayToCopy1);
        }

        if (Controller_GetParameter(CONTROLLER_TOTAL_DISTANCE_TRAVELLED, arrayToCopy4) == SUCCESS)
        {
            Controller_SetParameter(CONTROLLER_TOTAL_DISTANCE_TRAVELLED, CONTROLLER_TOTAL_DISTANCE_TRAVELLED_LEN, arrayToCopy4);
        }

        if (Controller_GetParameter(CONTROLLER_TOTAL_ENERGY_CONSUMPTION, arrayToCopy4) == SUCCESS)
        {
            Controller_SetParameter(CONTROLLER_TOTAL_ENERGY_CONSUMPTION, CONTROLLER_TOTAL_ENERGY_CONSUMPTION_LEN, arrayToCopy4);
        }

        if (Controller_GetParameter(CONTROLLER_OVERALL_EFFICIENCY, arrayToCopy4) == SUCCESS)
        {
            Controller_SetParameter(CONTROLLER_OVERALL_EFFICIENCY, CONTROLLER_OVERALL_EFFICIENCY_LEN, arrayToCopy4);
        }

        if (Controller_GetParameter(CONTROLLER_RANGE, arrayToCopy4) == SUCCESS)
        {
            Controller_SetParameter(CONTROLLER_RANGE, CONTROLLER_RANGE_LEN, arrayToCopy4);
        }

        if (Controller_GetParameter(CONTROLLER_CO2SAVED, arrayToCopy4) == SUCCESS)
        {
            Controller_SetParameter(CONTROLLER_CO2SAVED, CONTROLLER_CO2SAVED_LEN, arrayToCopy4);
        }

        if (Controller_GetParameter(CONTROLLER_INSTANT_ECONOMY, arrayToCopy2) == SUCCESS)
        {
            Controller_SetParameter(CONTROLLER_INSTANT_ECONOMY, CONTROLLER_INSTANT_ECONOMY_LEN, arrayToCopy2);
        }

        if (Controller_GetParameter(CONTROLLER_MOTOR_TEMPERATURE, arrayToCopy1) == SUCCESS)
        {
            Controller_SetParameter(CONTROLLER_MOTOR_TEMPERATURE, CONTROLLER_MOTOR_TEMPERATURE_LEN, arrayToCopy1);
        }
    }

    if (sp_periodic_evt_counter >= SP_PERIODIC_EVT_COUNT1)
    {
        sp_periodic_evt_counter = 0;
    }

    if (sp_periodic_evt_counter2 >= SP_PERIODIC_EVT_COUNT2)
    {
        sp_periodic_evt_counter2 = 0;
    }
}

/*********************************************************************
 * @fn      SimplePeripheral_updateRPA
 *
 * @brief   Read the current RPA from the stack
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_updateRPA(void)
{
    uint8_t* pRpaNew;

  // Read the current RPA.
    pRpaNew = GAP_GetDevAddress(FALSE);

    if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
    {
        memcpy(rpa, pRpaNew, B_ADDR_LEN);
    }
}

/*********************************************************************
 * @fn      SimplePeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimplePeripheral_clockHandler(UArg arg)
{
    spClockEventData_t *pData = (spClockEventData_t *)arg;

    if (pData->event == SP_PERIODIC_EVT)
    {
        // Start the next period
        Util_startClock(&clkPeriodic);

        sp_periodic_evt_counter++;
        sp_periodic_evt_counter2++;

        // Post event to wake up the application
        SimplePeripheral_enqueueMsg(SP_PERIODIC_EVT, NULL);
    }
    else if (pData->event == SP_READ_RPA_EVT)
    {
        // Start the next period
        Util_startClock(&clkRpaRead);

        // Post event to read the current RPA
        SimplePeripheral_enqueueMsg(SP_READ_RPA_EVT, NULL);
    }
    else if (pData->event == SP_SEND_PARAM_UPDATE_EVT)
    {
        // Send message to app
        SimplePeripheral_enqueueMsg(SP_SEND_PARAM_UPDATE_EVT, pData);
    }
}

/*********************************************************************
 * @fn      SimplePeripheral_doSetConnPhy
 *
 * @brief   Set PHY preference.
 *
 * @param   index - 0: 1M PHY
 *                  1: 2M PHY
 *                  2: 1M + 2M PHY
 *                  3: CODED PHY (Long range)
 *                  4: 1M + 2M + CODED PHY
 *
 * @return  always true
 */
bool SimplePeripheral_doSetConnPhy(uint8 index)
{
  bool status = TRUE;

  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS,
    HCI_PHY_CODED, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS | HCI_PHY_CODED,
    AUTO_PHY_UPDATE
  };

  uint8_t connIndex = SimplePeripheral_getConnIndex(menuConnHandle);
  if (connIndex >= MAX_NUM_BLE_CONNS)
  {
    return (FALSE);
  }

  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX.
  // If auto PHY update is not selected and if auto PHY update is enabled, then
  // stop auto PHY update
  // Note PHYs are already enabled by default in build_config.opt in stack project.
  if(phy[index] != AUTO_PHY_UPDATE)
  {
    // Cancel RSSI reading  and auto phy changing
    SimplePeripheral_stopAutoPhyChange(connList[connIndex].connHandle);

    SimplePeripheral_setPhy(menuConnHandle, 0, phy[index], phy[index], 0);

  }
  else
  {
    // Start RSSI read for auto PHY update (if it is disabled)
    SimplePeripheral_startAutoPhyChange(menuConnHandle);
  }

  return (status);
}
/*********************************************************************
 * @fn      SimplePeripheral_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
  spGapAdvEventData_t *pData = ICall_malloc(sizeof(spGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(SimplePeripheral_enqueueMsg(SP_ADV_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
//uint8_t GAPAdv_status = 0xFF;   // debugging only
//uint32_t GAPAdvEvent;           // debugging only

static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData)
{
//    GAPAdvEvent = pEventData->event;

  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
      BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GAP_EVT_ADV_START_AFTER_ENABLE", 0);
//          GAPAdv_status = 0x01;
          sp_advertiseFlag = 1;
      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
//        GAPAdv_status = 0x02;
        sp_advertiseFlag = 0;
      break;

    case GAP_EVT_ADV_START:
//        GAPAdv_status = 0x03;
        sp_advertiseFlag = 1;
      break;

    case GAP_EVT_ADV_END:
//        GAPAdv_status = 0x04;
        sp_advertiseFlag = 0;
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
//        GAPAdv_status = 0x05;
        sp_advertiseFlag = 0;
    break;

    case GAP_EVT_SCAN_REQ_RECEIVED:
//        GAPAdv_status = 0x06;
      break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
//        GAPAdv_status = 0x07;
      break;

    default:
      break;
  }

  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}


/*********************************************************************
 * @fn      SimplePeripheral_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimplePeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  spPairStateData_t *pData = ICall_malloc(sizeof(spPairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if(SimplePeripheral_enqueueMsg(SP_PAIR_STATE_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimplePeripheral_passcodeCb(uint8_t *pDeviceAddr,
                                        uint16_t connHandle,
                                        uint8_t uiInputs,
                                        uint8_t uiOutputs,
                                        uint32_t numComparison)
{
  spPasscodeData_t *pData = ICall_malloc(sizeof(spPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData )
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if(SimplePeripheral_enqueueMsg(SP_PASSCODE_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimplePeripheral_processPairState(spPairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;

  switch (state)
  {
    case GAPBOND_PAIRING_STATE_STARTED:
      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:

      break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:

      break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:

      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimplePeripheral_processPasscode(spPasscodeData_t *pPasscodeData)
{
  // Send passcode response
  GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle , SUCCESS,
                         B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      SimplePeripheral_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if(SimplePeripheral_enqueueMsg(SP_CONN_EVT, pReport) != SUCCESS)
  {
    ICall_free(pReport);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
  // Get index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(pReport->handle);

  // If auto phy change is enabled
  if (connList[connIndex].isAutoPHYEnable == TRUE)
  {
    // Read the RSSI
    HCI_ReadRssiCmd(pReport->handle);
  }
}


/*********************************************************************
 * @fn      SimplePeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static status_t SimplePeripheral_enqueueMsg(uint8_t event, void *pData)
{
  uint8_t success;
  spEvt_t *pMsg = ICall_malloc(sizeof(spEvt_t));

  // Create dynamic pointer to message.
  if(pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return(bleMemAllocError);
}

/*********************************************************************
 * @fn      SimplePeripheral_doAutoConnect
 *
 * @brief   Enable/Disable peripheral as AutoConnect node.
 *
 * @param   index - 0 : Disable AutoConnect
 *                  1 : Enable Group A
 *                  2 : Enable Group B
 *
 * @return  always true
 */
bool SimplePeripheral_doAutoConnect(uint8_t index)
{
    if (index == 1)
    {
      if (autoConnect != AUTOCONNECT_GROUP_A)
      {
        GapAdv_disable(advHandleLegacy);
        advData1[2] = 'G';
        advData1[3] = 'A';
        check_enableStatus = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_DURATION , SP_ADVERTISING_TIMEOUT);
        *ptr_GAPflag = 0;  // whenever advertising is enabled, set GAPflag to 0

        autoConnect = AUTOCONNECT_GROUP_A;
      }
    }
    else if (index == 2)
    {
      if (autoConnect != AUTOCONNECT_GROUP_B)
      {
        GapAdv_disable(advHandleLegacy);

        advData1[2] = 'G';
        advData1[3] = 'B';
        check_enableStatus = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_DURATION , SP_ADVERTISING_TIMEOUT);
        *ptr_GAPflag = 0;  // whenever advertising is enabled, set GAPflag to 0


        autoConnect = AUTOCONNECT_GROUP_B;
      }
    }
    else
    {
      if (autoConnect)
      {
        GapAdv_disable(advHandleLegacy);

        advData1[2] = 'S';
        advData1[3] = 'P';
        check_enableStatus = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_DURATION , SP_ADVERTISING_TIMEOUT);
        *ptr_GAPflag = 0;  // whenever advertising is enabled, set GAPflag to 0


        autoConnect = AUTOCONNECT_DISABLE;
      }

    }
    return (true);
}


/*********************************************************************
 * @fn      SimplePeripheral_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_addConn(uint16_t connHandle)
{
  uint8_t i;
  uint8_t status = bleNoResources;

  // Try to find an available entry
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
      // Allocate data to send through clock handler
      connList[i].pParamUpdateEventData = ICall_malloc(sizeof(spClockEventData_t) +
                                                       sizeof (uint16_t));
      if(connList[i].pParamUpdateEventData)
      {
        connList[i].pParamUpdateEventData->event = SP_SEND_PARAM_UPDATE_EVT;
        *((uint16_t *)connList[i].pParamUpdateEventData->data) = connHandle;

        // Create a clock object and start
        connList[i].pUpdateClock
          = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

        if (connList[i].pUpdateClock)
        {
          Util_constructClock(connList[i].pUpdateClock,
                              SimplePeripheral_clockHandler,
                              SEND_PARAM_UPDATE_DELAY, 0, true,
                              (UArg) (connList[i].pParamUpdateEventData));
        }
        else
        {
            ICall_free(connList[i].pParamUpdateEventData);
        }
      }
      else
      {
        status = bleMemAllocError;
      }
#endif  // DEFAULT_SEND_PARAM_UPDATE_REQ

      // Set default PHY to 1M
      connList[i].currPhy = HCI_PHY_1_MBPS;

      break;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  return(MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle)
{
  uint8_t i;
  // Set to invalid connection index initially
  uint8_t connIndex = MAX_NUM_BLE_CONNS;

  if(connHandle != LINKDB_CONNHANDLE_ALL)
  {
    // Get connection index from handle
    connIndex = SimplePeripheral_getConnIndex(connHandle);
    if(connIndex >= MAX_NUM_BLE_CONNS)
	{
	  return(bleInvalidRange);
	}
  }

  // Clear specific handle or all handles
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL))
    {
      connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
      connList[i].currPhy = 0;
      connList[i].phyCngRq = 0;
      connList[i].phyRqFailCnt = 0;
      connList[i].rqPhy = 0;
      memset(connList[i].rssiArr, 0, SP_MAX_RSSI_STORE_DEPTH);
      connList[i].rssiAvg = 0;
      connList[i].rssiCntr = 0;
      connList[i].isAutoPHYEnable = FALSE;
    }
  }

  return(SUCCESS);
}

/*********************************************************************
 * @fn      SimplePeripheral_clearPendingParamUpdate
 *
 * @brief   clean pending param update request in the paramUpdateList list
 *
 * @param   connHandle - connection handle to clean
 *
 * @return  none
 */
void SimplePeripheral_clearPendingParamUpdate(uint16_t connHandle)
{
  List_Elem *curr;

  for (curr = List_head(&paramUpdateList); curr != NULL; curr = List_next(curr)) 
  {
    if (((spConnHandleEntry_t *)curr)->connHandle == connHandle)
    {
      List_remove(&paramUpdateList, curr);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle)
{
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);

  if(connIndex != MAX_NUM_BLE_CONNS)
  {
    Clock_Struct* pUpdateClock = connList[connIndex].pUpdateClock;

    if (pUpdateClock != NULL)
    {
      // Stop and destruct the RTOS clock if it's still alive
      if (Util_isActive(pUpdateClock))
      {
        Util_stopClock(pUpdateClock);
      }

      // Destruct the clock object
      Clock_destruct(pUpdateClock);
      // Free clock struct
      ICall_free(pUpdateClock);
      // Free ParamUpdateEventData
      ICall_free(connList[connIndex].pParamUpdateEventData);
    }
    // Clear pending update requests from paramUpdateList
    SimplePeripheral_clearPendingParamUpdate(connHandle);
    // Stop Auto PHY Change
    SimplePeripheral_stopAutoPhyChange(connHandle);
    // Clear Connection List Entry
    SimplePeripheral_clearConnListEntry(connHandle);
  }

  return connIndex;
}

/*********************************************************************
 * @fn      SimplePeripheral_processParamUpdate
 *
 * @brief   Process a parameters update request
 *
 * @return  None
 */
static void SimplePeripheral_processParamUpdate(uint16_t connHandle)
{
  gapUpdateLinkParamReq_t req;
  uint8_t connIndex;

  req.connectionHandle = connHandle;
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
  req.connLatency = DEFAULT_DESIRED_PERIPHERAL_LATENCY;
  req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
  req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
  req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
#endif  // DEFAULT_SEND_PARAM_UPDATE_REQ

  connIndex = SimplePeripheral_getConnIndex(connHandle);
  if (connIndex >= MAX_NUM_BLE_CONNS)
  {
    return;
  }

  // Deconstruct the clock object
  Clock_destruct(connList[connIndex].pUpdateClock);
  // Free clock struct, only in case it is not NULL
  if (connList[connIndex].pUpdateClock != NULL)
  {
      ICall_free(connList[connIndex].pUpdateClock);
      connList[connIndex].pUpdateClock = NULL;
  }
  // Free ParamUpdateEventData, only in case it is not NULL
  if (connList[connIndex].pParamUpdateEventData != NULL)
      ICall_free(connList[connIndex].pParamUpdateEventData);

  // Send parameter update
  bStatus_t status = GAP_UpdateLinkParamReq(&req);

  // If there is an ongoing update, queue this for when the udpate completes
  if (status == bleAlreadyInRequestedMode)
  {
    spConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(spConnHandleEntry_t));
    if (connHandleEntry)
    {
      connHandleEntry->connHandle = connHandle;

      List_put(&paramUpdateList, (List_Elem *)connHandleEntry);
    }
  }
}

/*********************************************************************
 * @fn      SimpleCentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  uint8_t status = pMsg->pReturnParam[0];

  //Find which command this command complete is for
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
    {
      int8 rssi = (int8)pMsg->pReturnParam[3];

      if (status == SUCCESS)
      {
        uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1], pMsg->pReturnParam[2]);

        uint8_t index = SimplePeripheral_getConnIndex(handle);
        if (index >= MAX_NUM_BLE_CONNS)
        {
          return;
        }

        if (rssi != LL_RSSI_NOT_AVAILABLE)
        {
          connList[index].rssiArr[connList[index].rssiCntr++] = rssi;
          connList[index].rssiCntr %= SP_MAX_RSSI_STORE_DEPTH;

          int16_t sum_rssi = 0;
          for(uint8_t cnt=0; cnt<SP_MAX_RSSI_STORE_DEPTH; cnt++)
          {
            sum_rssi += connList[index].rssiArr[cnt];
          }
          connList[index].rssiAvg = (uint32_t)(sum_rssi/SP_MAX_RSSI_STORE_DEPTH);

          uint8_t phyRq = SP_PHY_NONE;
          uint8_t phyRqS = SP_PHY_NONE;
          uint8_t phyOpt = LL_PHY_OPT_NONE;

          if(connList[index].phyCngRq == FALSE)
          {
            if((connList[index].rssiAvg >= RSSI_2M_THRSHLD) &&
            (connList[index].currPhy != HCI_PHY_2_MBPS) &&
                 (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to higher data rate
              phyRqS = phyRq = HCI_PHY_2_MBPS;
            }
            else if((connList[index].rssiAvg < RSSI_2M_THRSHLD) &&
                    (connList[index].rssiAvg >= RSSI_1M_THRSHLD) &&
                    (connList[index].currPhy != HCI_PHY_1_MBPS) &&
                    (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to legacy regular data rate
              phyRqS = phyRq = HCI_PHY_1_MBPS;
            }
            else if((connList[index].rssiAvg >= RSSI_S2_THRSHLD) &&
                    (connList[index].rssiAvg < RSSI_1M_THRSHLD) &&
                    (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to lower data rate S=2(500kb/s)
              phyRqS = HCI_PHY_CODED;
              phyOpt = LL_PHY_OPT_S2;
              phyRq = BLE5_CODED_S2_PHY;
            }
            else if(connList[index].rssiAvg < RSSI_S2_THRSHLD )
            {
              // try to go to lowest data rate S=8(125kb/s)
              phyRqS = HCI_PHY_CODED;
              phyOpt = LL_PHY_OPT_S8;
              phyRq = BLE5_CODED_S8_PHY;
            }
            if((phyRq != SP_PHY_NONE) &&
               // First check if the request for this phy change is already not honored then don't request for change
               (((connList[index].rqPhy == phyRq) &&
                 (connList[index].phyRqFailCnt < 2)) ||
                 (connList[index].rqPhy != phyRq)))
            {
              //Initiate PHY change based on RSSI
              SimplePeripheral_setPhy(connList[index].connHandle, 0,
                                      phyRqS, phyRqS, phyOpt);
              connList[index].phyCngRq = TRUE;

              // If it a request for different phy than failed request, reset the count
              if(connList[index].rqPhy != phyRq)
              {
                // then reset the request phy counter and requested phy
                connList[index].phyRqFailCnt = 0;
              }

              if(phyOpt == LL_PHY_OPT_NONE)
              {
                connList[index].rqPhy = phyRq;
              }
              else if(phyOpt == LL_PHY_OPT_S2)
              {
                connList[index].rqPhy = BLE5_CODED_S2_PHY;
              }
              else
              {
                connList[index].rqPhy = BLE5_CODED_S8_PHY;
              }

            } // end of if ((phyRq != SP_PHY_NONE) && ...
          } // end of if (connList[index].phyCngRq == FALSE)
        } // end of if (rssi != LL_RSSI_NOT_AVAILABLE)

	  } // end of if (status == SUCCESS)
      break;
    }

    case HCI_LE_READ_PHY:
    {
      if (status == SUCCESS)
      break;
    }

    default:
      break;
  } // end of switch (pMsg->cmdOpcode)
}

/*********************************************************************
* @fn      SimplePeripheral_initPHYRSSIArray
*
* @brief   Initializes the array of structure/s to store data related
*          RSSI based auto PHy change
*
* @param   connHandle - the connection handle
*
* @param   addr - pointer to device address
*
* @return  index of connection handle
*/
static void SimplePeripheral_initPHYRSSIArray(void)
{
  //Initialize array to store connection handle and RSSI values
  memset(connList, 0, sizeof(connList));
  for (uint8_t index = 0; index < MAX_NUM_BLE_CONNS; index++)
  {
    connList[index].connHandle = SP_INVALID_HANDLE;
  }
}
/*********************************************************************
      // Set default PHY to 1M
 * @fn      SimplePeripheral_startAutoPhyChange
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   devAddr - device address
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
static status_t SimplePeripheral_startAutoPhyChange(uint16_t connHandle)
{
  status_t status = FAILURE;

  // Get connection index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Start Connection Event notice for RSSI calculation
  status = Gap_RegisterConnEventCb(SimplePeripheral_connEvtCB, GAP_CB_REGISTER, GAP_CB_CONN_EVENT_ALL, connHandle);

  // Flag in connection info if successful
  if (status == SUCCESS)
  {
    connList[connIndex].isAutoPHYEnable = TRUE;
  }

  return status;
}

/*********************************************************************
 * @fn      SimplePeripheral_stopAutoPhyChange
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static status_t SimplePeripheral_stopAutoPhyChange(uint16_t connHandle)
{
  // Get connection index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Stop connection event notice
  Gap_RegisterConnEventCb(NULL, GAP_CB_UNREGISTER, GAP_CB_CONN_EVENT_ALL, connHandle);

  // Also update the phychange request status for active RSSI tracking connection
  connList[connIndex].phyCngRq = FALSE;
  connList[connIndex].isAutoPHYEnable = FALSE;

  return SUCCESS;
}

/*********************************************************************
 * @fn      SimplePeripheral_setPhy
 *
 * @brief   Call the HCI set phy API and and add the handle to a
 *          list to match it to an incoming command status event
 */
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts)
{
  // Allocate list entry to store handle for command status
  spConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(spConnHandleEntry_t));

  if (connHandleEntry)
  {
    connHandleEntry->connHandle = connHandle;

    // Add entry to the phy command status list
    List_put(&setPhyCommStatList, (List_Elem *)connHandleEntry);

    // Send PHY Update
    HCI_LE_SetPhyCmd(connHandle, allPhys, txPhy, rxPhy, phyOpts);
  }

  return SUCCESS;
}

/*********************************************************************
* @fn      SimplePeripheral_updatePHYStat
*
* @brief   Update the auto phy update state machine
*
* @param   connHandle - the connection handle
*
* @return  None
*/
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg)
{
  uint8_t connIndex;

  switch (eventCode)
  {
    case HCI_LE_SET_PHY:
    {
      // Get connection handle from list
      spConnHandleEntry_t *connHandleEntry =
                           (spConnHandleEntry_t *)List_get(&setPhyCommStatList);

      if (connHandleEntry)
      {
        // Get index from connection handle
        connIndex = SimplePeripheral_getConnIndex(connHandleEntry->connHandle);

        ICall_free(connHandleEntry);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;

          if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
          {
            // Update the phychange request status for active RSSI tracking connection
            connList[connIndex].phyCngRq = FALSE;
            connList[connIndex].phyRqFailCnt++;
          }
        }
      }
      break;
    }

    // LE Event - a Phy update has completed or failed
    case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT:
    {
      hciEvt_BLEPhyUpdateComplete_t *pPUC =
                                     (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

      if(pPUC)
      {
        // Get index from connection handle
        connIndex = SimplePeripheral_getConnIndex(pPUC->connHandle);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          // Update the phychange request status for active RSSI tracking connection
          connList[connIndex].phyCngRq = FALSE;

          if (pPUC->status == SUCCESS)
          {
            connList[connIndex].currPhy = pPUC->rxPhy;
          }
          if(pPUC->rxPhy != connList[connIndex].rqPhy)
          {
            connList[connIndex].phyRqFailCnt++;
          }
          else
          {
            // Reset the request phy counter and requested phy
            connList[connIndex].phyRqFailCnt = 0;
            connList[connIndex].rqPhy = 0;
          }
        }
      }

      break;
    }

    default:
      break;
  } // end of switch (eventCode)
}
