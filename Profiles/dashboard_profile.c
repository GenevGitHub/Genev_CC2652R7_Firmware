/*
 * dashboard_profile.c
 *
 *  Created on: 16 May 2024
 *      Author: Chee
 */

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include <bcomdef.h>
//#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "Profiles/dashboard_profile.h"
#include <gatt_profile_uuid.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/
// Dashboard Service UUID: 0x1811
CONST uint8 Dashboard_ServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DASHBOARD_SERV_UUID), HI_UINT16(DASHBOARD_SERV_UUID)
};

// Dashboard_Error_Code UUID
CONST uint8 Dashboard_Error_CodeUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DASHBOARD_ERROR_CODE_UUID), HI_UINT16(DASHBOARD_ERROR_CODE_UUID)
};

// Dashboard_Speed_Mode UUID
CONST uint8 Dashboard_Speed_ModeUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DASHBOARD_SPEED_MODE_UUID), HI_UINT16(DASHBOARD_SPEED_MODE_UUID)
};

// Dashboard_Light_Status UUID
CONST uint8 Dashboard_Light_StatusUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DASHBOARD_LIGHT_STATUS_UUID), HI_UINT16(DASHBOARD_LIGHT_STATUS_UUID)
};

// Dashboard_Light_Mode UUID
CONST uint8 Dashboard_Light_ModeUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DASHBOARD_LIGHT_MODE_UUID), HI_UINT16(DASHBOARD_LIGHT_MODE_UUID)
};

// Dashboard_Power_On_Time UUID
CONST uint8 Dashboard_Power_On_TimeUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DASHBOARD_POWER_ON_TIME_UUID), HI_UINT16(DASHBOARD_POWER_ON_TIME_UUID)
};

// Dashboard_ADCounter UUID
CONST uint8 Dashboard_ADCounterUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DASHBOARD_ADCOUNTER_UUID), HI_UINT16(DASHBOARD_ADCOUNTER_UUID)
};

// Dashboard_Device_UpTime UUID
CONST uint8 Dashboard_Device_UpTimeUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DASHBOARD_DEVICE_UPTIME_UUID), HI_UINT16(DASHBOARD_DEVICE_UPTIME_UUID)
};
/*********************************************************************
 * LOCAL VARIABLES
 */

static DashboardCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

/*********** Dashboard Service Declaration ****************/
static CONST gattAttrType_t DashboardDecl = { ATT_BT_UUID_SIZE, Dashboard_ServUUID };


/**************************  Characteristic Properties ******************************/
// Characteristic "Dashboard_Error_Code" Properties (for declaration) - Client (App) side
static uint8 Dashboard_Error_CodeProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Dashboard_Speed_Mode" Properties (for declaration)
//static uint8 Dashboard_Speed_ModeProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 Dashboard_Speed_ModeProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_NOTIFY;   // Chee 20250111
// Characteristic "Dashboard_Light_Status" Properties (for declaration)
static uint8 Dashboard_Light_StatusProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Dashboard_Light_Mode" Properties (for declaration)
static uint8 Dashboard_Light_ModeProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_NOTIFY;
// Characteristic "Dashboard_Power_On_Time" Properties (for declaration)
static uint8 Dashboard_Power_On_TimeProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Dashboard_ADCounter" Properties (for declaration)
static uint8 Dashboard_ADCounterProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Dashboard_Device_UpTime" Properties (for declaration)
static uint8 Dashboard_Device_UpTimeProps = GATT_PROP_READ | GATT_PROP_NOTIFY;


/**************************  Characteristic Values ******************************/
// Characteristic "Dashboard_Error_Code" Value variable and declares initial values
uint8 Dashboard_Error_CodeVal[DASHBOARD_ERROR_CODE_LEN] = {0xFF};  // "error code" - not error priority
// Characteristic "Dashboard_Speed_Mode" Value variable and declares initial values
uint8 Dashboard_Speed_ModeVal[DASHBOARD_SPEED_MODE_LEN] = {1};
// Characteristic "Dashboard_Light_Status" Value variable and declares initial values
uint8 Dashboard_Light_StatusVal[DASHBOARD_LIGHT_STATUS_LEN] = {0};
// Characteristic "Dashboard_Light_Mode" Value variable and declares initial values
uint8 Dashboard_Light_ModeVal[DASHBOARD_LIGHT_MODE_LEN] = {2};
// Characteristic "Dashboard_Power_On_Time" Value variable  and declares initial values - little Endian {low byte, high byte}
uint8 Dashboard_Power_On_TimeVal[DASHBOARD_POWER_ON_TIME_LEN] = {0x00, 0x00};
// Characteristic "Dashboard_ADCounter" Value variable and declares initial values - little Endian {low byte, high byte}
uint8 Dashboard_ADCounterVal[DASHBOARD_ADCOUNTER_LEN] = {0, 0, 0, 0};
// Characteristic "Dashboard_Device_UpTime" Value variable  and declares initial values - little Endian {low byte, high byte}
uint8 Dashboard_Device_UpTimeVal[DASHBOARD_DEVICE_UPTIME_LEN] = {0x00, 0x00, 0x00, 0x00};


/**************************  User Descriptions ******************************/
// Simple Profile Characteristic 1 User Description
static uint8 Dashboard_Error_CodeUserDesp[17] = "Dashboard Alert";
// Simple Profile Characteristic 2 User Description
static uint8 Dashboard_Speed_ModeUserDesp[17] = "Speed Mode";
// Simple Profile Characteristic 3 User Description
static uint8 Dashboard_Light_StatusUserDesp[17] = "Light Status";
// Simple Profile Characteristic 4 User Description
static uint8 Dashboard_Light_ModeUserDesp[17] = "Light Mode";
// Simple Profile Characteristic 5 User Description
static uint8 Dashboard_Power_On_TimeUserDesp[17] = "Device On Time";
// Simple Profile Characteristic 6 User Description
static uint8 Dashboard_ADCounterUserDesp[17] = "AD Data ID";
// Simple Profile Characteristic 7 User Description
static uint8 Dashboard_Device_UpTimeUserDesp[17] = "Device Uptime";

/**************************  Client Characteristic ******************************/
// Characteristic "Dashboard_Error_Code" CCCD
static gattCharCfg_t *Dashboard_Error_CodeConfig;
// Characteristic "Dashboard_Speed_Mode" CCCD
static gattCharCfg_t *Dashboard_Speed_ModeConfig;
// Characteristic "Dashboard_Light_Status" CCCD
static gattCharCfg_t *Dashboard_Light_StatusConfig;
// Characteristic "Dashboard_Light_Mode" : Client Characteristic Configuration Description
static gattCharCfg_t *Dashboard_Light_ModeConfig;
// Client Characteristic Configuration
static gattCharCfg_t *Dashboard_Power_On_TimeConfig;
// Characteristic "Dashboard_ADCounter" CCCD
static gattCharCfg_t *Dashboard_ADCounterConfig;
// Client Characteristic Configuration
static gattCharCfg_t *Dashboard_Device_UpTimeConfig;

/********* Declare Struct of Dashboard Characteristic Values ****************/
dashboardCharVal_t DCVArray;


/*********************************************************************
*
*
* Profile Attributes - Table
*
*/
static gattAttribute_t DashboardAttrTbl[] =
{
  /********* Dashboard Service Declaration *********/
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8 *)&DashboardDecl
  },


    /************* Characteristic ERROR CODE ***********/
    // Dashboard_Error_Code Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Dashboard_Error_CodeProps
    },
        // Dashboard_Error_Code Characteristic Value
        {
        { ATT_BT_UUID_SIZE, Dashboard_Error_CodeUUID },
           GATT_PERMIT_READ,
            0,
            Dashboard_Error_CodeVal
        },
          // Dashboard_Error_Code CCCD
            {
              { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                  0,
                  (uint8 *)&Dashboard_Error_CodeConfig
            },
        // Dashboard_Error_Code user descriptor
        {
        {ATT_BT_UUID_SIZE, charUserDescUUID},
            GATT_PERMIT_READ,
            0,
            Dashboard_Error_CodeUserDesp    //"Dashboard Error Code"
        },

    /********* Characteristic: SPEED MODE ************/
    // Dashboard_Speed_Mode Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
          GATT_PERMIT_READ,
          0,
          &Dashboard_Speed_ModeProps
    },
      // Dashboard_Speed_Mode Characteristic Value
      {
        { ATT_BT_UUID_SIZE, Dashboard_Speed_ModeUUID },
//            GATT_PERMIT_READ,
            GATT_PERMIT_READ | GATT_PERMIT_WRITE,   // Client (mobile App) is given the permission to write. Chee 20250111
            0,
            Dashboard_Speed_ModeVal
      },
                          // Dashboard_Speed_Mode CCCD
                            {
                              { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                                  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                                  0,
                                  (uint8 *)&Dashboard_Speed_ModeConfig
                            },
      // Dashboard_Speed_Mode user descriptor
      {
        {ATT_BT_UUID_SIZE, charUserDescUUID},
            GATT_PERMIT_READ,
            0,
            Dashboard_Speed_ModeUserDesp    //"Speed Mode"
      },

    /************** Characteristic: LIGHT STATUS **********/
    // Dashboard_Light_Status Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
          GATT_PERMIT_READ,
          0,
          &Dashboard_Light_StatusProps
    },
      // Dashboard_Light_Status Characteristic Value
      {
        { ATT_BT_UUID_SIZE,  Dashboard_Light_StatusUUID},
            GATT_PERMIT_READ,
            0,
            Dashboard_Light_StatusVal
      },
                      // Dashboard_Light_Status CCCD
                        {
                          { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                              GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                              0,
                              (uint8 *)&Dashboard_Light_StatusConfig
                        },
      // Dashboard_Light_Status user descriptor
      {
        {ATT_BT_UUID_SIZE, charUserDescUUID},
            GATT_PERMIT_READ,
            0,
            Dashboard_Light_StatusUserDesp    //"Light Status"
      },

    /********* Characteristic: LIGHT MODE **************/
    // Dashboard_Light_Mode Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
          GATT_PERMIT_READ,
          0,
          &Dashboard_Light_ModeProps
    },
      // Dashboard_Light_Mode Characteristic Value
      {
        { ATT_BT_UUID_SIZE, Dashboard_Light_ModeUUID },
            GATT_PERMIT_READ | GATT_PERMIT_WRITE,   // Client is given the permission to write
            0,
            Dashboard_Light_ModeVal
      },
          // Dashboard_Light_Mode : Client Characteristic Configuration Description
                      // Light mode can be changed from APP (Client)
                      {
                        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                            GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                            0,
                            (uint8 *)&Dashboard_Light_ModeConfig
                      },
      // Dashboard_Light_Mode user descriptor
      {
        {ATT_BT_UUID_SIZE, charUserDescUUID},
            GATT_PERMIT_READ,
            0,
            Dashboard_Light_ModeUserDesp    //"Light Mode"
      },


    /********* Characteristic: POWER ON TIME *********/
    // Dashboard_Power_On_Time Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
          GATT_PERMIT_READ,
          0,
          &Dashboard_Power_On_TimeProps
    },
      // Dashboard_Power_On_Time Characteristic Value
      {
        { ATT_BT_UUID_SIZE, Dashboard_Power_On_TimeUUID },
            GATT_PERMIT_READ,
            0,
            Dashboard_Power_On_TimeVal
      },
          // Dashboard_Power_On_Time : Client Characteristic Configuration Description
                      {
                        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                            GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                            0,
                            (uint8 *)&Dashboard_Power_On_TimeConfig
                      },
      // Dashboard_Power_On_Time user descriptor
      {
        {ATT_BT_UUID_SIZE, charUserDescUUID},
            GATT_PERMIT_READ,
            0,
            Dashboard_Power_On_TimeUserDesp    //"Power On Time (minutes)"
      },

    /********* Characteristic: ADCOUNTER *********/
    // Dashboard_ADCounter Characteristic Declaration
      {
        { ATT_BT_UUID_SIZE, characterUUID },
            GATT_PERMIT_READ,
            0,
            &Dashboard_ADCounterProps
      },
      // Dashboard_ADCounter Characteristic Value
      {
        { ATT_BT_UUID_SIZE, Dashboard_ADCounterUUID },
            GATT_PERMIT_READ,
            0,
            Dashboard_ADCounterVal
      },
                  // Dashboard_ADCounter CCCD
                        {
                          { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                              GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                              0,
                              (uint8 *)&Dashboard_ADCounterConfig
                        },
      // Dashboard_ADCounter user descriptor
      {
        {ATT_BT_UUID_SIZE, charUserDescUUID},
            GATT_PERMIT_READ,
            0,
            Dashboard_ADCounterUserDesp    //"Data ID"
      },


  /********* Characteristic: DEVICE UPTIME *********/
  // Dashboard_Device_UpTime Characteristic Declaration
  {
    { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &Dashboard_Device_UpTimeProps
  },
    // Dashboard_Device_UpTime Characteristic Value
    {
      { ATT_BT_UUID_SIZE, Dashboard_Device_UpTimeUUID },
          GATT_PERMIT_READ,
          0,
          Dashboard_Device_UpTimeVal
    },
        // Dashboard_Device_UpTime : Client Characteristic Configuration Description
                    {
                      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                          GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                          0,
                          (uint8 *)&Dashboard_Device_UpTimeConfig
                    },
    // Dashboard_Device_UpTime user descriptor
    {
      {ATT_BT_UUID_SIZE, charUserDescUUID},
          GATT_PERMIT_READ,
          0,
          Dashboard_Device_UpTimeUserDesp    //"Device UpTime (minutes)"
    }

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t Dashboard_ReadAttrCB( uint16 connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 *pLen,
                                           uint16 offset, uint16 maxLen,
                                           uint8 method );
static bStatus_t Dashboard_WriteAttrCB( uint16 connHandle,
                                            gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len,
                                            uint16 offset, uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Dashboard Profile Service Callbacks
CONST gattServiceCBs_t DashboardCBs =
{
  Dashboard_ReadAttrCB,  // Read callback function pointer
  Dashboard_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/
/******  Pointer Register functions ******/

extern void Dashboard_profile_init(){
    DCVArray.ptr_ADdataID = Dashboard_ADCounterVal;
    DCVArray.ptr_powerOnTime = Dashboard_Power_On_TimeVal;
    DCVArray.ptr_dashErrorCode = Dashboard_Error_CodeVal;
    DCVArray.ptr_speedMode = Dashboard_Speed_ModeVal;
    DCVArray.ptr_lightMode = Dashboard_Light_ModeVal;
    DCVArray.ptr_lightStatus = Dashboard_Light_StatusVal;
    DCVArray.ptr_deviceUpTime = Dashboard_Device_UpTimeVal;
}

extern void* Dashboard_CharValRegister(void)
{
    return (&DCVArray);
}

/*
 * Dashboard_AddService- Initializes the Dashboard service by registering
 *          GATT attributes with the GATT server.
 *
 */
bStatus_t Dashboard_AddService( void )
{
    uint8_t status;

    /******* Allocate Client Characteristic Configuration table ********/
    Dashboard_Error_CodeConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                              MAX_NUM_BLE_CONNS );
    if ( Dashboard_Error_CodeConfig == NULL ) {
        return ( bleMemAllocError );
    }
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Dashboard_Error_CodeConfig );

    /******* Allocate Client Characteristic Configuration table ********/
    Dashboard_Speed_ModeConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                MAX_NUM_BLE_CONNS );
    if ( Dashboard_Speed_ModeConfig == NULL ) {
        return ( bleMemAllocError );
    }
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Dashboard_Speed_ModeConfig );


    /******* Allocate Client Characteristic Configuration table ********/
    Dashboard_Light_StatusConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                  MAX_NUM_BLE_CONNS );
    if ( Dashboard_Light_StatusConfig == NULL ) {
        return ( bleMemAllocError );
    }
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Dashboard_Light_StatusConfig );

    /******* Allocate Client Characteristic Configuration table ********/
    Dashboard_Light_ModeConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                              MAX_NUM_BLE_CONNS );
    if ( Dashboard_Light_ModeConfig == NULL ) {
        return ( bleMemAllocError );
    }
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Dashboard_Light_ModeConfig );

    /******* Allocate Client Characteristic Configuration table ********/
    Dashboard_Power_On_TimeConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                   MAX_NUM_BLE_CONNS );
    if ( Dashboard_Power_On_TimeConfig == NULL ) {
        return ( bleMemAllocError );
    }
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Dashboard_Power_On_TimeConfig );

    /******* Allocate Client Characteristic Configuration table *******/
    Dashboard_ADCounterConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                               MAX_NUM_BLE_CONNS );
    if ( Dashboard_ADCounterConfig == NULL ) {
        return ( bleMemAllocError );
    }
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Dashboard_ADCounterConfig );

    /******* Allocate Client Characteristic Configuration table ********/
    Dashboard_Device_UpTimeConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                   MAX_NUM_BLE_CONNS );
    if ( Dashboard_Device_UpTimeConfig == NULL ) {
        return ( bleMemAllocError );
    }
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Dashboard_Device_UpTimeConfig );

    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( DashboardAttrTbl,
                                        GATT_NUM_ATTRS( DashboardAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &DashboardCBs );

    return ( status );
}

/*
 * Dashboard_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t Dashboard_RegisterAppCBs( DashboardCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*
 * Dashboard_SetParameter - Set a Dashboard parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t Dashboard_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case DASHBOARD_ERROR_CODE:
        if ( len == DASHBOARD_ERROR_CODE_LEN )
        {
        memcpy(Dashboard_Error_CodeVal, value, len);
        // Try to send notification.
        GATTServApp_ProcessCharCfg( Dashboard_Error_CodeConfig, Dashboard_Error_CodeVal, FALSE,
                                    DashboardAttrTbl, GATT_NUM_ATTRS( DashboardAttrTbl ),
                                    INVALID_TASK_ID,  Dashboard_ReadAttrCB);
        }
        else
        {
        ret = bleInvalidRange;
        }
        break;
    case DASHBOARD_SPEED_MODE:
        if ( len == DASHBOARD_SPEED_MODE_LEN )
        {
          memcpy(Dashboard_Speed_ModeVal, value, len);
          // Try to send notification.
          GATTServApp_ProcessCharCfg( Dashboard_Speed_ModeConfig, Dashboard_Speed_ModeVal, FALSE,
                                      DashboardAttrTbl, GATT_NUM_ATTRS( DashboardAttrTbl ),
                                      INVALID_TASK_ID,  Dashboard_ReadAttrCB);
        }
        else
        {
          ret = bleInvalidRange;
        }
        break;
    case DASHBOARD_LIGHT_STATUS:
        if ( len == DASHBOARD_LIGHT_STATUS_LEN )
        {
          memcpy(Dashboard_Light_StatusVal, value, len);
          // Try to send notification.
          GATTServApp_ProcessCharCfg( Dashboard_Light_StatusConfig, Dashboard_Light_StatusVal, FALSE,
                                      DashboardAttrTbl, GATT_NUM_ATTRS( DashboardAttrTbl ),
                                      INVALID_TASK_ID,  Dashboard_ReadAttrCB);
        }
        else
        {
          ret = bleInvalidRange;
        }
        break;
    case DASHBOARD_LIGHT_MODE:
        if ( len == DASHBOARD_LIGHT_MODE_LEN )
        {
          memcpy(Dashboard_Light_ModeVal, value, len);
          // Try to send notification.
          GATTServApp_ProcessCharCfg( Dashboard_Light_ModeConfig, Dashboard_Light_ModeVal, FALSE,
                                      DashboardAttrTbl, GATT_NUM_ATTRS( DashboardAttrTbl ),
                                      INVALID_TASK_ID,  Dashboard_ReadAttrCB);
        }
        else
        {
          ret = bleInvalidRange;
        }
        break;
    case DASHBOARD_POWER_ON_TIME:
        if ( len == DASHBOARD_POWER_ON_TIME_LEN )
        {
          memcpy(Dashboard_Power_On_TimeVal, value, len);
          // Try to send notification.
          GATTServApp_ProcessCharCfg( Dashboard_Power_On_TimeConfig, Dashboard_Power_On_TimeVal,
                                      FALSE, DashboardAttrTbl, GATT_NUM_ATTRS( DashboardAttrTbl ),
                                      INVALID_TASK_ID,  Dashboard_ReadAttrCB);
        }
        else
        {
          ret = bleInvalidRange;
        }
        break;
    case DASHBOARD_ADCOUNTER:
        if ( len == DASHBOARD_ADCOUNTER_LEN )
        {
          memcpy(Dashboard_ADCounterVal, value, len);
          // Try to send notification.
          GATTServApp_ProcessCharCfg( Dashboard_ADCounterConfig, Dashboard_ADCounterVal,
                                      FALSE, DashboardAttrTbl, GATT_NUM_ATTRS( DashboardAttrTbl ),
                                      INVALID_TASK_ID,  Dashboard_ReadAttrCB);
        }
        else
        {
          ret = bleInvalidRange;
        }
        break;
    case DASHBOARD_DEVICE_UPTIME:
        if ( len == DASHBOARD_DEVICE_UPTIME_LEN )
        {
          memcpy(Dashboard_Device_UpTimeVal, value, len);
          // Try to send notification.
          GATTServApp_ProcessCharCfg( Dashboard_Device_UpTimeConfig, Dashboard_Device_UpTimeVal,
                                      FALSE, DashboardAttrTbl, GATT_NUM_ATTRS( DashboardAttrTbl ),
                                      INVALID_TASK_ID,  Dashboard_ReadAttrCB);
        }
        else
        {
          ret = bleInvalidRange;
        }
        break;
    default:
        ret = INVALIDPARAMETER;
        break;
  }
  return ret;
}
/*
 * Dashboard_GetParameter - Get a Dashboard parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t Dashboard_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
  case DASHBOARD_LIGHT_MODE:
          memcpy(value, Dashboard_Light_ModeVal, DASHBOARD_LIGHT_MODE_LEN);
        break;
  case DASHBOARD_LIGHT_STATUS:
          memcpy(value, Dashboard_Light_StatusVal, DASHBOARD_LIGHT_STATUS_LEN);
        break;
  case DASHBOARD_SPEED_MODE:
          memcpy(value, Dashboard_Speed_ModeVal, DASHBOARD_SPEED_MODE_LEN);
        break;
  case DASHBOARD_POWER_ON_TIME:
          memcpy(value, Dashboard_Power_On_TimeVal, DASHBOARD_POWER_ON_TIME_LEN);
        break;
  case DASHBOARD_ADCOUNTER:
          memcpy(value, Dashboard_ADCounterVal, DASHBOARD_ADCOUNTER_LEN);
        break;
  case DASHBOARD_DEVICE_UPTIME:
          memcpy(value, Dashboard_Device_UpTimeVal, DASHBOARD_DEVICE_UPTIME_LEN);
        break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}
/*********************************************************************
 * @fn          Dashboard_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t Dashboard_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                       uint8 *pValue, uint16 *pLen, uint16 offset,
                                       uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;
  // See if request is regarding the Dashboard_Error_Code Characteristic Value
  if (! memcmp(pAttr->type.uuid, Dashboard_Error_CodeUUID, pAttr->type.len) )
  {
    if ( offset > DASHBOARD_ERROR_CODE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DASHBOARD_ERROR_CODE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Dashboard_Speed_Mode Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Dashboard_Speed_ModeUUID, pAttr->type.len) )
  {
    if ( offset > DASHBOARD_SPEED_MODE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DASHBOARD_SPEED_MODE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Dashboard_Light_Status Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Dashboard_Light_StatusUUID, pAttr->type.len) )
  {
    if ( offset > DASHBOARD_LIGHT_STATUS_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DASHBOARD_LIGHT_STATUS_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Dashboard_Light_Mode Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Dashboard_Light_ModeUUID, pAttr->type.len) )
  {
    if ( offset > DASHBOARD_LIGHT_MODE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DASHBOARD_LIGHT_MODE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Dashboard_Power_On_Time Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Dashboard_Power_On_TimeUUID, pAttr->type.len) )
  {
    if ( offset > DASHBOARD_POWER_ON_TIME_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DASHBOARD_POWER_ON_TIME_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Dashboard_ADCounter Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Dashboard_ADCounterUUID, pAttr->type.len) )
  {
    if ( offset > DASHBOARD_ADCOUNTER_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DASHBOARD_ADCOUNTER_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Dashboard_Device_UpTime Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Dashboard_Device_UpTimeUUID, pAttr->type.len) )
  {
    if ( offset > DASHBOARD_DEVICE_UPTIME_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, DASHBOARD_DEVICE_UPTIME_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return (status);
}


/*********************************************************************
 * @fn      Dashbpard_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
bStatus_t Dashboard_WriteAttrCB(uint16_t connHandle,
                                     gattAttribute_t *pAttr,
                                     uint8_t *pValue, uint16_t len,
                                     uint16_t offset, uint8_t method)
  {
  bStatus_t status = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  /* Toggle light mode */
  else if(! memcmp(pAttr->type.uuid, Dashboard_Light_ModeUUID, pAttr->type.len))
  {
      if ( offset + len > DASHBOARD_LIGHT_MODE_LEN )
      {
            status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
         // Copy pValue into the variable we point to from the attribute table.
        memcpy(pAttr->pValue + offset, pValue, len);

        // Only notify application if entire expected value is written
        if ( offset + len == DASHBOARD_LIGHT_MODE_LEN)
          paramID = DASHBOARD_LIGHT_MODE;
      }
  }
  /* Toggle lock and unlock of speed mode */  //Chee added 20250110
  else if(! memcmp(pAttr->type.uuid, Dashboard_Speed_ModeUUID, pAttr->type.len))
  {
      if ( offset + len > DASHBOARD_SPEED_MODE_LEN )
      {
            status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
         // Copy pValue into the variable we point to from the attribute table.
        memcpy(pAttr->pValue + offset, pValue, len);

        // Only notify application if entire expected value is written
        if ( offset + len == DASHBOARD_SPEED_MODE_LEN)
          paramID = DASHBOARD_SPEED_MODE;
      }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }
  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF)
  {
    if ( pAppCBs && pAppCBs->pfnChangeCb )
    {
      pAppCBs->pfnChangeCb( paramID ); // Call app function from stack task context.
    }
  }

  return (status);
}

