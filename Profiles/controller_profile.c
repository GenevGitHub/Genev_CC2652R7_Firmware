/*
 * controller_profile.c
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

#include "Profiles/controller_profile.h"
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

// Controller Service UUID
static CONST uint8 ControllerUUID[ATT_BT_UUID_SIZE] =
{
     LO_UINT16(CONTROLLER_SERV_UUID), HI_UINT16(CONTROLLER_SERV_UUID)
};

// Controller_Voltage UUID
static CONST uint8 Controller_VoltageUUID[ATT_BT_UUID_SIZE] =
{
     LO_UINT16(CONTROLLER_VOLTAGE_UUID), HI_UINT16(CONTROLLER_VOLTAGE_UUID)
};

// Controller_Current UUID
//static CONST uint8 Controller_CurrentUUID[ATT_BT_UUID_SIZE] =
//{
//     LO_UINT16(CONTROLLER_CURRENT_UUID), HI_UINT16(CONTROLLER_CURRENT_UUID)
//};

// Controller_Heat_Sink_Temperature UUID
static CONST uint8 Controller_Heat_Sink_TemperatureUUID[ATT_BT_UUID_SIZE] =
{
     LO_UINT16(CONTROLLER_HEAT_SINK_TEMPERATURE_UUID), HI_UINT16(CONTROLLER_HEAT_SINK_TEMPERATURE_UUID)
};

// Controller_Error_Code UUID
static CONST uint8 Controller_Error_CodeUUID[ATT_BT_UUID_SIZE] =
{
     LO_UINT16(CONTROLLER_ERROR_CODE_UUID), HI_UINT16(CONTROLLER_ERROR_CODE_UUID)
};

// Controller_Motor_RPM UUID
static CONST uint8 Controller_Motor_RPM_UUID[ATT_BT_UUID_SIZE] =
{
     LO_UINT16(CONTROLLER_MOTOR_RPM_UUID), HI_UINT16(CONTROLLER_MOTOR_RPM_UUID)
};

// Controller_Motor_Speed UUID
static CONST uint8 Controller_Motor_SpeedUUID[ATT_BT_UUID_SIZE] =
{
     LO_UINT16(CONTROLLER_MOTOR_SPEED_UUID), HI_UINT16(CONTROLLER_MOTOR_SPEED_UUID)
};

// Controller_Total_Distance_Travelled UUID
static CONST uint8 Controller_Total_Distance_TravelledUUID[ATT_BT_UUID_SIZE] =
{
     LO_UINT16(CONTROLLER_TOTAL_DISTANCE_TRAVELLED_UUID), HI_UINT16(CONTROLLER_TOTAL_DISTANCE_TRAVELLED_UUID)
};

// Controller_Total_Energy_Consumption UUID
static CONST uint8 Controller_Total_Energy_ConsumptionUUID[ATT_BT_UUID_SIZE] =
{
     LO_UINT16(CONTROLLER_TOTAL_ENERGY_CONSUMPTION_UUID), HI_UINT16(CONTROLLER_TOTAL_ENERGY_CONSUMPTION_UUID)
};

// Controller_Overall_Efficiency UUID
static CONST uint8 Controller_Overall_EfficiencyUUID[ATT_BT_UUID_SIZE] =
{
     LO_UINT16(CONTROLLER_OVERALL_EFFICIENCY_UUID), HI_UINT16(CONTROLLER_OVERALL_EFFICIENCY_UUID)
};

// Controller_Range UUID
static CONST uint8 Controller_RangeUUID[ATT_BT_UUID_SIZE] =
{
     LO_UINT16(CONTROLLER_RANGE_UUID), HI_UINT16(CONTROLLER_RANGE_UUID)
};

// Controller_co2Saved UUID
static CONST uint8 Controller_co2SavedUUID[ATT_BT_UUID_SIZE] =
{
     LO_UINT16(CONTROLLER_CO2SAVED_UUID), HI_UINT16(CONTROLLER_CO2SAVED_UUID)
};

// Controller_Motor_Temperature UUID
static CONST uint8 Controller_Motor_TemperatureUUID[ATT_BT_UUID_SIZE] =
{
     LO_UINT16(CONTROLLER_MOTOR_TEMPERATURE_UUID), HI_UINT16(CONTROLLER_MOTOR_TEMPERATURE_UUID)
};

// Controller_Instant_Economy UUID
static CONST uint8 Controller_Instant_EconomyUUID[ATT_BT_UUID_SIZE] =
{
     LO_UINT16(CONTROLLER_INSTANT_ECONOMY_UUID), HI_UINT16(CONTROLLER_INSTANT_ECONOMY_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static ControllerCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
//static CONST gattAttrType_t ControllerDecl = { ATT_UUID_SIZE, ControllerUUID };
static CONST gattAttrType_t ControllerDecl = { ATT_BT_UUID_SIZE, ControllerUUID };
/*************** Controller Characteristics Properties ***********************/
// Characteristic "Controller_Voltage" Properties (for declaration)
static uint8_t Controller_VoltageProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
//// Characteristic "Controller_Current" Properties (for declaration)
//static uint8_t Controller_CurrentProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Controller_Heat_Sink_Temperature" Properties (for declaration)
static uint8_t Controller_Heat_Sink_TemperatureProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Controller_Error_Code" Properties (for declaration)
static uint8_t Controller_Error_CodeProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Controller_Motor_RPM" Properties (for declaration)
static uint8_t Controller_Motor_RPM_Props = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Controller_Motor_Speed" Properties (for declaration)
static uint8_t Controller_Motor_SpeedProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Controller_Total_Distance_Travelled" Properties (for declaration)
static uint8_t Controller_Total_Distance_TravelledProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Controller_Total_Energy_Consumption" Properties (for declaration)
static uint8_t Controller_Total_Energy_ConsumptionProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Controller_Overall_Efficiency" Properties (for declaration)
static uint8_t Controller_Overall_EfficiencyProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Controller_Range" Properties (for declaration)
static uint8 Controller_RangeProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Controller_co2Saved" Properties (for declaration)
static uint8 Controller_co2SavedProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Controller_Motor_Temperature" Properties (for declaration)
static uint8_t Controller_Motor_TemperatureProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "Controller_Instant_Economy" Properties (for declaration)
static uint8 Controller_Instant_EconomyProps = GATT_PROP_READ | GATT_PROP_NOTIFY;


/*************** Controller Characteristics Values ***********************/
// Characteristic "Controller_Voltage" Value variable -> need to break it up into bits of 8 bits for communication
static uint8_t Controller_VoltageVal[CONTROLLER_VOLTAGE_LEN] = {0};
//// Characteristic "Controller_Current" Value variable
//static uint8_t Controller_CurrentVal[CONTROLLER_CURRENT_LEN] = {0};
// Characteristic "Controller_Heat_Sink_Temperature" Value variable
static uint8_t Controller_Heat_Sink_TemperatureVal[CONTROLLER_HEAT_SINK_TEMPERATURE_LEN] = {65};        // temperature offset by 50 to ensure negative value is not possible
// Characteristic "Controller_Error_Code" Value variable
static uint8_t Controller_Error_CodeVal[CONTROLLER_ERROR_CODE_LEN] = {0xFF};  // "error code" - not error priority
// Characteristic "Controller_Motor_RPM" Value variable
static uint8_t Controller_Motor_RPM_Val[CONTROLLER_MOTOR_RPM_LEN] = {0};
// Characteristic "Controller_Motor_Speed" Value variable
static uint8_t Controller_Motor_SpeedVal[CONTROLLER_MOTOR_SPEED_LEN] = {0};
// Characteristic "Controller_Total_Distance_Travelled" Value variable
uint8_t Controller_Total_Distance_TravelledVal[CONTROLLER_TOTAL_DISTANCE_TRAVELLED_LEN] = {0};
// Characteristic "Controller_Total_Energy_Consumption" Value variable
static uint8_t Controller_Total_Energy_ConsumptionVal[CONTROLLER_TOTAL_ENERGY_CONSUMPTION_LEN] = {0};
// Characteristic "Controller_Overall_Efficiency" Value variable
static uint8_t Controller_Overall_EfficiencyVal[CONTROLLER_OVERALL_EFFICIENCY_LEN] = {0};
// Characteristic "Controller_Range" Value variable
static uint8 Controller_RangeVal[CONTROLLER_RANGE_LEN] = {0};
// Characteristic "Controller_co2Saved" Value variable
static uint8 Controller_co2SavedVal[CONTROLLER_CO2SAVED_LEN] = {0};
// Characteristic "Controller_Motor_Temperature" Value variable
static uint8_t Controller_Motor_TemperatureVal[CONTROLLER_MOTOR_TEMPERATURE_LEN] = {65};
// Characteristic "Controller_Instant_Economy" Value variable
static uint8 Controller_Instant_EconomyVal[CONTROLLER_INSTANT_ECONOMY_LEN] = {0};


/*************** Controller Characteristics Descriptions ***********************/
// Controller Voltage User Description
static uint8 Controller_VoltageUserDesp[17] = "Phase Voltage";
//// Controller Current User Description
//static uint8 Controller_CurrentUserDesp[17] = "Phase Current";
// Controller _Heat_Sink_Temperature User Description
static uint8 Controller_Heat_Sink_TemperatureUserDesp[17] = "Contr. Temp +50";
// Controller Error User Description
static uint8 Controller_Error_CodeUserDesp[17] = "Controller Alert";
// Controller Motor RPM User Description
static uint8 Controller_Motor_RPMUserDesp[17] = "Avg. Motor RPM";
// Controller Motor Speed User Description
static uint8 Controller_Motor_SpeedUserDesp[17] = "Avg. Motor Speed";
// Controller_Total_Distance_Travelled User Description
static uint8 Controller_Total_Distance_TravelledUserDesp[17] = "Dist. Traveled";
// Controller_Total_Energy_Consumption User Description
static uint8 Controller_Total_Energy_ConsumptionUserDesp[17] = "Energy Consumed";
// Controller_Overall_Efficiency User Description
static uint8 Controller_Overall_EfficiencyUserDesp[17] = "Overall Economy";
// Controller_Range User Description
static uint8 Controller_RangeUserDesp[17] = "Range";
// Controller_co2Saved User Description
static uint8 Controller_co2SavedUserDesp[17] = "co2 Saved";
// Controller_Motor_Temperature User Description
static uint8 Controller_Motor_TemperatureUserDesp[17] = "Motor Temp +50";
// Controller_Motor_Instant_Economy User Description
static uint8 Controller_Instant_EconomyUserDesp[17] = "Instant. Eco.";


/********* Declare Struct of Controller Characteristic Values ****************/
controllerCharVal_t CCVArray;


/**************************  Client Characteristic ******************************/
// Characteristic "Controller_Voltage" CCCD
static gattCharCfg_t *Controller_VoltageConfig;
//// Characteristic "Controller_Current" CCCD
//static gattCharCfg_t *Controller_CurrentConfig;
// Characteristic "Controller_Heat_Sink_Temperature" CCCD
static gattCharCfg_t *Controller_Heat_Sink_TemperatureConfig;
// Characteristic "Controller_Error_Code" CCCD
static gattCharCfg_t *Controller_Error_CodeConfig;
// Characteristic "Controller_Motor_RPM" CCCD
static gattCharCfg_t *Controller_Motor_RPM_Config;
// Characteristic "Controller_Motor_Speed" CCCD
static gattCharCfg_t *Controller_Motor_SpeedConfig;
// Characteristic "Controller_Total_Distance_Travelled" CCCD
static gattCharCfg_t *Controller_Total_Distance_TravelledConfig;
// Characteristic "Controller_Total_Energy_Consumption" CCCD
static gattCharCfg_t *Controller_Total_Energy_ConsumptionConfig;
// Characteristic "Controller_Overall_Efficiency" CCCD
static gattCharCfg_t *Controller_Overall_EfficiencyConfig;
// Characteristic "Controller_Range" CCCD
static gattCharCfg_t *Controller_RangeConfig;
// Characteristic "Controller_co2Saved" CCCD
static gattCharCfg_t *Controller_co2SavedConfig;
// Characteristic "Controller_Motor_Temperature" CCCD
static gattCharCfg_t *Controller_Motor_TemperatureConfig;
// Characteristic "Controller_Instant_Economy" CCCD
static gattCharCfg_t *Controller_Instant_EconomyConfig;

/*********************************************************************
*
*
* Profile Attributes - Table
*
*/
static gattAttribute_t ControllerAttrTbl[] =
{
  // Controller Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8 *)&ControllerDecl
  },
    // Controller_Voltage Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Controller_VoltageProps
    },
        // Controller_Voltage Characteristic Value
        {
          { ATT_BT_UUID_SIZE, Controller_VoltageUUID },
          GATT_PERMIT_READ,
          0,
          Controller_VoltageVal
        },
        // Controller_Voltage CCCD
                    {
                      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                      GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                      0,
                      (uint8 *)&Controller_VoltageConfig
                    },
        // Controller_Voltage user descriptor
        {
          {ATT_BT_UUID_SIZE, charUserDescUUID},
          GATT_PERMIT_READ,
          0,
          Controller_VoltageUserDesp  //"Voltage (mV)"
        },

//    // Controller_Current Characteristic Declaration
//    {
//      { ATT_BT_UUID_SIZE, characterUUID },
//      GATT_PERMIT_READ,
//      0,
//      &Controller_CurrentProps
//    },
//        // Controller_Current Characteristic Value
//        {
//          { ATT_BT_UUID_SIZE, Controller_CurrentUUID },
//          GATT_PERMIT_READ,
//          0,
//          Controller_CurrentVal
//        },
//                    // Controller_Current CCCD
//                    {
//                      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
//                      GATT_PERMIT_READ | GATT_PERMIT_WRITE,
//                      0,
//                      (uint8 *)&Controller_CurrentConfig
//                    },
//        // Controller_Current user descriptor
//        {
//          {ATT_BT_UUID_SIZE, charUserDescUUID},
//          GATT_PERMIT_READ,
//          0,
//          Controller_CurrentUserDesp  //"Current (mA)"
//        },
        /********* Characteristic: Controller_Heat_Sink_Temperature *********/
        // Controller_Heat_Sink_Temperature Characteristic Declaration
        {
          { ATT_BT_UUID_SIZE, characterUUID },
          GATT_PERMIT_READ,
          0,
          &Controller_Heat_Sink_TemperatureProps
        },
          // Controller_Heat_Sink_Temperature Characteristic Value
          {
            { ATT_BT_UUID_SIZE, Controller_Heat_Sink_TemperatureUUID },
            GATT_PERMIT_READ,
            0,
            Controller_Heat_Sink_TemperatureVal
          },
                  // Controller_Heat_Sink_Temperature CCCD
                        {
                          { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                          GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                          0,
                          (uint8 *)&Controller_Heat_Sink_TemperatureConfig
                        },
         // Controller_Heat_Sink_Temperature user descriptor
          {
            {ATT_BT_UUID_SIZE, charUserDescUUID},
            GATT_PERMIT_READ,
            0,
            Controller_Heat_Sink_TemperatureUserDesp    //"Heat Sink Temperature (Cels)"
          },

          /********* Characteristic: Controller_Error_Code *********/
            // Controller_Error_Code Characteristic Declaration
        {
          { ATT_BT_UUID_SIZE, characterUUID },
          GATT_PERMIT_READ,
          0,
          &Controller_Error_CodeProps
        },
          // Controller_Error_Code_Temperature Characteristic Value
          {
            { ATT_BT_UUID_SIZE, Controller_Error_CodeUUID },
            GATT_PERMIT_READ,
            0,
            Controller_Error_CodeVal
          },
                  // Controller_Error_Code_Temperature CCCD
                        {
                          { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                          GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                          0,
                          (uint8 *)&Controller_Error_CodeConfig
                        },
          // Controller_Error_Code user descriptor
          {
            {ATT_BT_UUID_SIZE, charUserDescUUID},
            GATT_PERMIT_READ,
            0,
            Controller_Error_CodeUserDesp    //"Controller Error Code"
          },


          /********* Characteristic: Controller_Motor_RPM *********/
            // Controller_Motor_RPM Characteristic Declaration
            {
              { ATT_BT_UUID_SIZE, characterUUID },
              GATT_PERMIT_READ,
              0,
              &Controller_Motor_RPM_Props
            },
              // Controller_Motor_RPM Characteristic Value
              {
                { ATT_BT_UUID_SIZE, Controller_Motor_RPM_UUID },
                GATT_PERMIT_READ,
                0,
                Controller_Motor_RPM_Val
              },
                          // Controller_Motor_RPM CCCD
                            {
                              { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                              GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                              0,
                              (uint8 *)&Controller_Motor_RPM_Config
                            },
              // Controller_Motor_RPM user descriptor
              {
                {ATT_BT_UUID_SIZE, charUserDescUUID},
                GATT_PERMIT_READ,
                0,
                Controller_Motor_RPMUserDesp    //"Motor RPM"
              },

          /********* Characteristic: Controller_Motor_Speed *********/
            // Controller_Motor_Speed Characteristic Declaration
            {
              { ATT_BT_UUID_SIZE, characterUUID },
              GATT_PERMIT_READ,
              0,
              &Controller_Motor_SpeedProps
            },
              // Controller_Motor_Speed Characteristic Value
              {
                { ATT_BT_UUID_SIZE, Controller_Motor_SpeedUUID },
                GATT_PERMIT_READ,
                0,
                Controller_Motor_SpeedVal
              },
                          // Controller_Motor_Speed CCCD
                          {
                            { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                            GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                            0,
                            (uint8 *)&Controller_Motor_SpeedConfig
                          },
              // Controller_Motor_Speed user descriptor
              {
                {ATT_BT_UUID_SIZE, charUserDescUUID},
                GATT_PERMIT_READ,
                0,
                Controller_Motor_SpeedUserDesp    //"Motor Speed (100kph)"   // 100 x km/hr
              },

              /********* Characteristic: Controller_Total_Distance_Travelled *********/
                // Controller_Total_Distance_Travelled Characteristic Declaration
                {
                  { ATT_BT_UUID_SIZE, characterUUID },
                  GATT_PERMIT_READ,
                  0,
                  &Controller_Total_Distance_TravelledProps
                },
                  // Controller_Total_Distance_Travelled Characteristic Value
                  {
                    { ATT_BT_UUID_SIZE, Controller_Total_Distance_TravelledUUID },
                    GATT_PERMIT_READ,
                    0,
                    Controller_Total_Distance_TravelledVal
                  },
                              // Controller_Total_Distance_Travelled CCCD
                                {
                                  { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                                  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                                  0,
                                  (uint8 *)&Controller_Total_Distance_TravelledConfig
                                },
                 // Controller_Total_Distance_Travelled user descriptor
                  {
                    {ATT_BT_UUID_SIZE, charUserDescUUID},
                    GATT_PERMIT_READ,
                    0,
                    Controller_Total_Distance_TravelledUserDesp    //"Total Distance Travelled (dm)"
                  },

                  /********* Characteristic: Controller_Total_Energy_Consumption *********/
                  // Controller_Total_Energy_Consumption Characteristic Declaration
                {
                  { ATT_BT_UUID_SIZE, characterUUID },
                  GATT_PERMIT_READ,
                  0,
                  &Controller_Total_Energy_ConsumptionProps
                },
                  // Controller_Total_Energy_Consumption Characteristic Value
                  {
                    { ATT_BT_UUID_SIZE, Controller_Total_Energy_ConsumptionUUID },
                    GATT_PERMIT_READ,
                    0,
                    Controller_Total_Energy_ConsumptionVal
                  },
                              // Controller_Total_Energy_Consumption CCCD
                                 {
                                   { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                                   GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                                   0,
                                   (uint8 *)&Controller_Total_Energy_ConsumptionConfig
                                 },
                 // Controller_Total_Energy_Consumption user descriptor
                  {
                    {ATT_BT_UUID_SIZE, charUserDescUUID},
                    GATT_PERMIT_READ,
                    0,
                    Controller_Total_Energy_ConsumptionUserDesp    //"Total Energy Consumption (mWh)"
                  },

                  /********* Characteristic: Controller_Overall_Efficiency *********/
                  // Controller_Overall_Efficiency Characteristic Declaration
                {
                  { ATT_BT_UUID_SIZE, characterUUID },
                  GATT_PERMIT_READ,
                  0,
                  &Controller_Overall_EfficiencyProps
                },
                  // Controller_Overall_Efficiency Characteristic Value
                  {
                    { ATT_BT_UUID_SIZE, Controller_Overall_EfficiencyUUID },
                    GATT_PERMIT_READ,
                    0,
                    Controller_Overall_EfficiencyVal
                  },
                              // Controller_Total_Energy_Efficiency CCCD
                              {
                                { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                                GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                                0,
                                (uint8 *)&Controller_Overall_EfficiencyConfig
                              },
                 // Controller_Overall_Efficiency user descriptor
                  {
                    {ATT_BT_UUID_SIZE, charUserDescUUID},
                    GATT_PERMIT_READ,
                    0,
                    Controller_Overall_EfficiencyUserDesp    //"Overall Economy (100Whpk)"   // unit in W-hr / km x 100
                  },

                  /********* Characteristic: Controller_Range *********/
                  // Controller_Range Characteristic Declaration
                  {
                      { ATT_BT_UUID_SIZE, characterUUID },
                      GATT_PERMIT_READ,
                      0,
                      &Controller_RangeProps
                  },
                      // Controller_Range Characteristic Value
                      {
                         { ATT_BT_UUID_SIZE, Controller_RangeUUID },
                         GATT_PERMIT_READ,
                         0,
                         Controller_RangeVal
                      },
                                  // Controller_Range CCCD
                                     {
                                        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                                        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                                        0,
                                        (uint8 *)&Controller_RangeConfig
                                     },
                      // Controller_Range user descriptor
                      {
                         {ATT_BT_UUID_SIZE, charUserDescUUID},
                         GATT_PERMIT_READ,
                         0,
                         Controller_RangeUserDesp    //"Range Available (m)"  // in meters
                      },

                  /********* Characteristic: Controller_co2Saved *********/
                  // Controller_co2Saved Characteristic Declaration
                {
                    { ATT_BT_UUID_SIZE, characterUUID },
                    GATT_PERMIT_READ,
                    0,
                    &Controller_co2SavedProps
                },
                    // Controller_co2Saved Characteristic Value
                    {
                       { ATT_BT_UUID_SIZE, Controller_co2SavedUUID },
                       GATT_PERMIT_READ,
                       0,
                       Controller_co2SavedVal
                    },
                                // Controller_co2Saved CCCD
                                  {
                                     { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                                     GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                                     0,
                                     (uint8 *)&Controller_co2SavedConfig
                                  },
                    // Controller_co2Saved user descriptor
                    {
                       {ATT_BT_UUID_SIZE, charUserDescUUID},
                       GATT_PERMIT_READ,
                       0,
                       Controller_co2SavedUserDesp    //"CO2 Saved (grams)"   // in grams
                    },

                /********* Characteristic: Controller_Motor_Temperature *********/
                // Controller_Motor_Temperature Characteristic Declaration
                {
                  { ATT_BT_UUID_SIZE, characterUUID },
                  GATT_PERMIT_READ,
                  0,
                  &Controller_Motor_TemperatureProps
                },
                  // Controller_Motor_Temperature Characteristic Value
                  {
                    { ATT_BT_UUID_SIZE, Controller_Motor_TemperatureUUID },
                    GATT_PERMIT_READ,
                    0,
                    Controller_Motor_TemperatureVal
                  },
                              // Controller_Motor_Temperature CCCD
                              {
                                { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                                GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                                0,
                                (uint8 *)&Controller_Motor_TemperatureConfig
                              },
                  // Controller_Motor_Temperature user descriptor
                  {
                    {ATT_BT_UUID_SIZE, charUserDescUUID},
                    GATT_PERMIT_READ,
                    0,
                    Controller_Motor_TemperatureUserDesp    //"Motor Temperature (Cels)"
                  },

              /********* Characteristic: Controller_Instant_Economy *********/
              // Controller_Instant_Economy Characteristic Declaration
              {
                { ATT_BT_UUID_SIZE, characterUUID },
                GATT_PERMIT_READ,
                0,
                &Controller_Instant_EconomyProps
              },
                // Controller_Instant_Economy Characteristic Value
                {
                  { ATT_BT_UUID_SIZE, Controller_Instant_EconomyUUID },
                  GATT_PERMIT_READ,
                  0,
                  Controller_Instant_EconomyVal
                },
                            // Controller_Instant_Economy CCCD
                            {
                              { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                              GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                              0,
                              (uint8 *)&Controller_Instant_EconomyConfig
                            },
                // Controller_Instant_Economy user descriptor
                {
                  {ATT_BT_UUID_SIZE, charUserDescUUID},
                  GATT_PERMIT_READ,
                  0,
                  Controller_Instant_EconomyUserDesp    //"Instantaneous Economy (100Whpk)"   // unit in W-hr / km x 100
                }
            };

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t Controller_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method );
static bStatus_t Controller_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t ControllerCBs =
{
  Controller_ReadAttrCB,  // Read callback function pointer
  Controller_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

extern void* Controller_CharValRegister(void)
{
    return (&CCVArray);
}

extern void Controller_profile_init(){
    CCVArray.ptr_co2saved = Controller_co2SavedVal;
    CCVArray.ptr_controllerErrorCode = Controller_Error_CodeVal;
    CCVArray.ptr_voltage = Controller_VoltageVal;
//    CCVArray.ptr_current = Controller_CurrentVal;
    CCVArray.ptr_heatSinkTempOffset50 = Controller_Heat_Sink_TemperatureVal;
    CCVArray.ptr_motorRPM = Controller_Motor_RPM_Val;
    CCVArray.ptr_motorSpeed = Controller_Motor_SpeedVal;
    CCVArray.ptr_totalDistance = Controller_Total_Distance_TravelledVal;
    CCVArray.ptr_totalEnergy = Controller_Total_Energy_ConsumptionVal;
    CCVArray.ptr_overallEfficiency = Controller_Overall_EfficiencyVal;
    CCVArray.ptr_range = Controller_RangeVal;
    CCVArray.ptr_motorTempOffset50 = Controller_Motor_TemperatureVal;
    CCVArray.ptr_instantEconomy = Controller_Instant_EconomyVal;
}

/*
 * Controller_AddService- Initializes the Controller service by registering
 *          GATT attributes with the GATT server.
 *
 */
bStatus_t Controller_AddService( void )
{
  uint8_t status;

  /***** Allocate Client Characteristic Configuration table *****/
      Controller_VoltageConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                MAX_NUM_BLE_CONNS );
      if ( Controller_VoltageConfig == NULL )
      {
        return ( bleMemAllocError );
      }
      // Initialize Client Characteristic Configuration attributes
      GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Controller_VoltageConfig );


//      /***** Allocate Client Characteristic Configuration table *****/
//          Controller_CurrentConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
//                                                                    MAX_NUM_BLE_CONNS );
//          if ( Controller_CurrentConfig == NULL )
//          {
//            return ( bleMemAllocError );
//          }
//          // Initialize Client Characteristic Configuration attributes
//          GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Controller_CurrentConfig );


      /***** Allocate Client Characteristic Configuration table *****/
          Controller_Heat_Sink_TemperatureConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                    MAX_NUM_BLE_CONNS );
          if ( Controller_Heat_Sink_TemperatureConfig == NULL )
          {
            return ( bleMemAllocError );
          }
          // Initialize Client Characteristic Configuration attributes
          GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Controller_Heat_Sink_TemperatureConfig );


      /***** Allocate Client Characteristic Configuration table *****/
          Controller_Error_CodeConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                    MAX_NUM_BLE_CONNS );
          if ( Controller_Error_CodeConfig == NULL )
          {
            return ( bleMemAllocError );
          }
          // Initialize Client Characteristic Configuration attributes
          GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Controller_Error_CodeConfig );


      /***** Allocate Client Characteristic Configuration table *****/
          Controller_Motor_RPM_Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                    MAX_NUM_BLE_CONNS );
          if ( Controller_Motor_RPM_Config == NULL )
          {
            return ( bleMemAllocError );
          }
          // Initialize Client Characteristic Configuration attributes
          GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Controller_Motor_RPM_Config );


      /***** Allocate Client Characteristic Configuration table *****/
          Controller_Motor_SpeedConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                    MAX_NUM_BLE_CONNS );
          if ( Controller_Motor_SpeedConfig == NULL )
          {
            return ( bleMemAllocError );
          }
          // Initialize Client Characteristic Configuration attributes
          GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Controller_Motor_SpeedConfig );


      /***** Allocate Client Characteristic Configuration table *****/
          Controller_Total_Distance_TravelledConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                    MAX_NUM_BLE_CONNS );
          if ( Controller_Total_Distance_TravelledConfig == NULL )
          {
            return ( bleMemAllocError );
          }
          // Initialize Client Characteristic Configuration attributes
          GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Controller_Total_Distance_TravelledConfig );


      /***** Allocate Client Characteristic Configuration table *****/
          Controller_Total_Energy_ConsumptionConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                    MAX_NUM_BLE_CONNS );
          if ( Controller_Total_Energy_ConsumptionConfig == NULL )
          {
            return ( bleMemAllocError );
          }

          // Initialize Client Characteristic Configuration attributes
          GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Controller_Total_Energy_ConsumptionConfig );


      /***** Allocate Client Characteristic Configuration table *****/
          Controller_Overall_EfficiencyConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                    MAX_NUM_BLE_CONNS );
          if ( Controller_Overall_EfficiencyConfig == NULL )
          {
            return ( bleMemAllocError );
          }
          // Initialize Client Characteristic Configuration attributes
          GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Controller_Overall_EfficiencyConfig );


      /***** Allocate Client Characteristic Configuration table *****/
          Controller_RangeConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                    MAX_NUM_BLE_CONNS );
          if ( Controller_RangeConfig == NULL )
          {
            return ( bleMemAllocError );
          }
          // Initialize Client Characteristic Configuration attributes
          GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Controller_RangeConfig );


      /***** Allocate Client Characteristic Configuration table *****/
          Controller_co2SavedConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                    MAX_NUM_BLE_CONNS );
          if ( Controller_co2SavedConfig == NULL )
          {
            return ( bleMemAllocError );
          }
          // Initialize Client Characteristic Configuration attributes
          GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Controller_co2SavedConfig );


      /***** Allocate Client Characteristic Configuration table *****/
          Controller_Motor_TemperatureConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                    MAX_NUM_BLE_CONNS );
          if ( Controller_Motor_TemperatureConfig == NULL )
          {
            return ( bleMemAllocError );
          }
          // Initialize Client Characteristic Configuration attributes
          GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Controller_Motor_TemperatureConfig );


      /***** Allocate Client Characteristic Configuration table *****/
          Controller_Instant_EconomyConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                                    MAX_NUM_BLE_CONNS );
          if ( Controller_Instant_EconomyConfig == NULL )
          {
            return ( bleMemAllocError );
          }
          // Initialize Client Characteristic Configuration attributes
          GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, Controller_Instant_EconomyConfig );

  // Register GATT attribute list and CBs with GATT Server App
          status = GATTServApp_RegisterService( ControllerAttrTbl,
                                        GATT_NUM_ATTRS( ControllerAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &ControllerCBs );

  return ( status );
}
/****************************************************************************
 * Controller_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 ****************************************************************************/
bStatus_t Controller_RegisterAppCBs( ControllerCBs_t *appCallbacks )
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
/****************************************************************************
 * Controller_SetParameter - Set a Battery parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 ****************************************************************************/
bStatus_t Controller_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case CONTROLLER_VOLTAGE:
    {
        if ( len == CONTROLLER_VOLTAGE_LEN )
        {
        memcpy(Controller_VoltageVal, value, len);
        // Try to send notification.
        // Call to GATTServApp_ProcessCharCfg requires numerous inputs.  In order to ensure
        // GATTServApp_ProcessCharCfg could be accessed correctly, it must be called within
        // simple peripheral
        GATTServApp_ProcessCharCfg( Controller_VoltageConfig, Controller_VoltageVal, FALSE,
                                    ControllerAttrTbl, GATT_NUM_ATTRS( ControllerAttrTbl ),
                                    INVALID_TASK_ID,  Controller_ReadAttrCB);
        }
        else
        {
        ret = bleInvalidRange;
        }
        break;
    }
//    case CONTROLLER_CURRENT:
//    {
//        if ( len == CONTROLLER_CURRENT_LEN )
//        {
//        memcpy(Controller_CurrentVal, value, len);
//        // Try to send notification.
//        // Call to GATTServApp_ProcessCharCfg requires numerous inputs.  In order to ensure
//        // GATTServApp_ProcessCharCfg could be accessed correctly, it must be called within
//        // simple peripheral
//        GATTServApp_ProcessCharCfg( Controller_CurrentConfig, Controller_CurrentVal, FALSE,
//                                    ControllerAttrTbl, GATT_NUM_ATTRS( ControllerAttrTbl ),
//                                    INVALID_TASK_ID,  Controller_ReadAttrCB);
//        }
//        else
//        {
//        ret = bleInvalidRange;
//        }
//        break;
//    }
    case CONTROLLER_HEAT_SINK_TEMPERATURE:
    {
        if ( len == CONTROLLER_HEAT_SINK_TEMPERATURE_LEN )
        {
            memcpy(Controller_Heat_Sink_TemperatureVal, value, len);
            // Try to send notification.
            // Call to GATTServApp_ProcessCharCfg requires numerous inputs.  In order to ensure
            // GATTServApp_ProcessCharCfg could be accessed correctly, it must be called within
            // simple peripheral
            GATTServApp_ProcessCharCfg( Controller_Heat_Sink_TemperatureConfig, Controller_Heat_Sink_TemperatureVal, FALSE,
                                        ControllerAttrTbl, GATT_NUM_ATTRS( ControllerAttrTbl ),
                                        INVALID_TASK_ID,  Controller_ReadAttrCB);
        }
        else
        {
            ret = bleInvalidRange;
        }
        break;
    }
    case CONTROLLER_ERROR_CODE:
    {
        if ( len == CONTROLLER_ERROR_CODE_LEN )
        {
            memcpy(Controller_Error_CodeVal, value, len);
            // Try to send notification.
            // Call to GATTServApp_ProcessCharCfg requires numerous inputs.  In order to ensure
            // GATTServApp_ProcessCharCfg could be accessed correctly, it must be called within
            // simple peripheral
            GATTServApp_ProcessCharCfg( Controller_Error_CodeConfig, Controller_Error_CodeVal, FALSE,
                                        ControllerAttrTbl, GATT_NUM_ATTRS( ControllerAttrTbl ),
                                        INVALID_TASK_ID,  Controller_ReadAttrCB);
        }
        else
        {
            ret = bleInvalidRange;
        }
        break;
    }
    case CONTROLLER_MOTOR_RPM:
    {
        if ( len == CONTROLLER_MOTOR_RPM_LEN )
        {
            memcpy(Controller_Motor_RPM_Val, value, len);
            // Try to send notification.
            // Call to GATTServApp_ProcessCharCfg requires numerous inputs.  In order to ensure
            // GATTServApp_ProcessCharCfg could be accessed correctly, it must be called within
            // simple peripheral
            GATTServApp_ProcessCharCfg( Controller_Motor_RPM_Config, Controller_Motor_RPM_Val, FALSE,
                                        ControllerAttrTbl, GATT_NUM_ATTRS( ControllerAttrTbl ),
                                        INVALID_TASK_ID,  Controller_ReadAttrCB);
         }
        else
        {
            ret = bleInvalidRange;
        }
        break;
    }
    case CONTROLLER_MOTOR_SPEED:
    {
        if ( len == CONTROLLER_MOTOR_SPEED_LEN )
        {
            memcpy(Controller_Motor_SpeedVal, value, len);
            // Try to send notification.
            // Call to GATTServApp_ProcessCharCfg requires numerous inputs.  In order to ensure
            // GATTServApp_ProcessCharCfg could be accessed correctly, it must be called within
            // simple peripheral
            GATTServApp_ProcessCharCfg( Controller_Motor_SpeedConfig, Controller_Motor_SpeedVal, FALSE,
                                        ControllerAttrTbl, GATT_NUM_ATTRS( ControllerAttrTbl ),
                                        INVALID_TASK_ID,  Controller_ReadAttrCB);
        }
        else
        {
            ret = bleInvalidRange;
        }
        break;
    }
    case CONTROLLER_TOTAL_DISTANCE_TRAVELLED:
    {
        if ( len == CONTROLLER_TOTAL_DISTANCE_TRAVELLED_LEN )
        {
            memcpy(Controller_Total_Distance_TravelledVal, value, len);
            // Try to send notification.
            // Call to GATTServApp_ProcessCharCfg requires numerous inputs.  In order to ensure
            // GATTServApp_ProcessCharCfg could be accessed correctly, it must be called within
            // simple peripheral
            GATTServApp_ProcessCharCfg( Controller_Total_Distance_TravelledConfig, Controller_Total_Distance_TravelledVal, FALSE,
                                        ControllerAttrTbl, GATT_NUM_ATTRS( ControllerAttrTbl ),
                                        INVALID_TASK_ID,  Controller_ReadAttrCB);
        }
        else
        {
            ret = bleInvalidRange;
        }
        break;
    }
    case CONTROLLER_TOTAL_ENERGY_CONSUMPTION:
    {
        if ( len == CONTROLLER_TOTAL_ENERGY_CONSUMPTION_LEN )
        {
            memcpy(Controller_Total_Energy_ConsumptionVal, value, len);
            // Try to send notification.
            // Call to GATTServApp_ProcessCharCfg requires numerous inputs.  In order to ensure
            // GATTServApp_ProcessCharCfg could be accessed correctly, it must be called within
            // simple peripheral
            GATTServApp_ProcessCharCfg( Controller_Total_Energy_ConsumptionConfig, Controller_Total_Energy_ConsumptionVal, FALSE,
                                        ControllerAttrTbl, GATT_NUM_ATTRS( ControllerAttrTbl ),
                                        INVALID_TASK_ID,  Controller_ReadAttrCB);
        }
        else
        {
            ret = bleInvalidRange;
        }
        break;
    }
    case CONTROLLER_OVERALL_EFFICIENCY:
    {
        if ( len == CONTROLLER_OVERALL_EFFICIENCY_LEN )

        {
            memcpy(Controller_Overall_EfficiencyVal, value, len);
            // Try to send notification.
            // Call to GATTServApp_ProcessCharCfg requires numerous inputs.  In order to ensure
            // GATTServApp_ProcessCharCfg could be accessed correctly, it must be called within
            // simple peripheral
            GATTServApp_ProcessCharCfg( Controller_Overall_EfficiencyConfig, Controller_Overall_EfficiencyVal, FALSE,
                                        ControllerAttrTbl, GATT_NUM_ATTRS( ControllerAttrTbl ),
                                        INVALID_TASK_ID,  Controller_ReadAttrCB);
        }
        else
        {
            ret = bleInvalidRange;
        }
        break;
    }
    case CONTROLLER_INSTANT_ECONOMY:
    {
        if ( len == CONTROLLER_INSTANT_ECONOMY_LEN )
        {
            memcpy(Controller_Instant_EconomyVal, value, len);
            // Try to send notification.
            // Call to GATTServApp_ProcessCharCfg requires numerous inputs.  In order to ensure
            // GATTServApp_ProcessCharCfg could be accessed correctly, it must be called within
            // simple peripheral
            GATTServApp_ProcessCharCfg( Controller_Instant_EconomyConfig, Controller_Instant_EconomyVal, FALSE,
                                        ControllerAttrTbl, GATT_NUM_ATTRS( ControllerAttrTbl ),
                                        INVALID_TASK_ID,  Controller_ReadAttrCB);
        }
        else
        {
            ret = bleInvalidRange;
        }
        break;
    }
    case CONTROLLER_RANGE:
    {
        if ( len == CONTROLLER_RANGE_LEN )
        {
            memcpy(Controller_RangeVal, value, len);
            // Try to send notification.
            // Call to GATTServApp_ProcessCharCfg requires numerous inputs.  In order to ensure
            // GATTServApp_ProcessCharCfg could be accessed correctly, it must be called within
            // simple peripheral
            GATTServApp_ProcessCharCfg( Controller_RangeConfig, Controller_RangeVal, FALSE,
                                        ControllerAttrTbl, GATT_NUM_ATTRS( ControllerAttrTbl ),
                                        INVALID_TASK_ID,  Controller_ReadAttrCB);
       }
        else
        {
            ret = bleInvalidRange;
        }
        break;
    }
    case CONTROLLER_CO2SAVED:
    {
        if ( len == CONTROLLER_CO2SAVED_LEN )
        {
            memcpy(Controller_co2SavedVal, value, len);
            // Try to send notification.
            // Call to GATTServApp_ProcessCharCfg requires numerous inputs.  In order to ensure
            // GATTServApp_ProcessCharCfg could be accessed correctly, it must be called within
            // simple peripheral
            GATTServApp_ProcessCharCfg( Controller_co2SavedConfig, Controller_co2SavedVal, FALSE,
                                        ControllerAttrTbl, GATT_NUM_ATTRS( ControllerAttrTbl ),
                                        INVALID_TASK_ID,  Controller_ReadAttrCB);
        }
        else
        {
            ret = bleInvalidRange;
        }
        break;
    }
    case CONTROLLER_MOTOR_TEMPERATURE:
    {
        if ( len == CONTROLLER_MOTOR_TEMPERATURE_LEN )
        {
            memcpy(Controller_Motor_TemperatureVal, value, len);
            // Try to send notification.
            // Call to GATTServApp_ProcessCharCfg requires numerous inputs.  In order to ensure
            // GATTServApp_ProcessCharCfg could be accessed correctly, it must be called within
            // simple peripheral
            GATTServApp_ProcessCharCfg( Controller_Motor_TemperatureConfig, Controller_Motor_TemperatureVal, FALSE,
                                        ControllerAttrTbl, GATT_NUM_ATTRS( ControllerAttrTbl ),
                                        INVALID_TASK_ID,  Controller_ReadAttrCB);
        }
        else
        {
            ret = bleInvalidRange;
        }
        break;
    }
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}

/*
 * Controller_GetParameter - Get a Controller parameter/characteristic.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t Controller_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case CONTROLLER_VOLTAGE:
        memcpy((uint8_t*)value, Controller_VoltageVal, CONTROLLER_VOLTAGE_LEN);
        break;
//    case CONTROLLER_CURRENT:
//        memcpy((uint8_t*)value, Controller_CurrentVal, CONTROLLER_CURRENT_LEN);
//        break;
    case CONTROLLER_HEAT_SINK_TEMPERATURE:
        memcpy((uint8_t*)value, Controller_Heat_Sink_TemperatureVal, CONTROLLER_HEAT_SINK_TEMPERATURE_LEN);
        break;
    case CONTROLLER_ERROR_CODE:
        memcpy((uint8_t*)value, Controller_Error_CodeVal, CONTROLLER_ERROR_CODE_LEN);
        break;
    case CONTROLLER_MOTOR_RPM:
        memcpy((uint8_t*)value, Controller_Motor_RPM_Val, CONTROLLER_MOTOR_RPM_LEN);
        break;
    case CONTROLLER_MOTOR_SPEED:
        memcpy((uint8_t*)value, Controller_Motor_SpeedVal, CONTROLLER_MOTOR_SPEED_LEN);
        break;
    case CONTROLLER_TOTAL_DISTANCE_TRAVELLED:
        memcpy((uint8_t*)value, Controller_Total_Distance_TravelledVal, CONTROLLER_TOTAL_DISTANCE_TRAVELLED_LEN);
        break;
    case CONTROLLER_TOTAL_ENERGY_CONSUMPTION:
        memcpy((uint8_t*)value, Controller_Total_Energy_ConsumptionVal, CONTROLLER_TOTAL_ENERGY_CONSUMPTION_LEN);
        break;
    case CONTROLLER_OVERALL_EFFICIENCY:
        memcpy((uint8_t*)value, Controller_Overall_EfficiencyVal, CONTROLLER_OVERALL_EFFICIENCY_LEN);
        break;
    case CONTROLLER_RANGE:
        memcpy((uint8_t*)value, Controller_RangeVal, CONTROLLER_RANGE_LEN);
        break;
    case CONTROLLER_CO2SAVED:
        memcpy((uint8_t*)value, Controller_co2SavedVal, CONTROLLER_CO2SAVED_LEN);
        break;
    case CONTROLLER_MOTOR_TEMPERATURE:
        memcpy((uint8_t*)value, Controller_Motor_TemperatureVal, CONTROLLER_MOTOR_TEMPERATURE_LEN);
        break;
    case CONTROLLER_INSTANT_ECONOMY:
        memcpy((uint8_t*)value, Controller_Instant_EconomyVal, CONTROLLER_INSTANT_ECONOMY_LEN);
        break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          Controller_ReadAttrCB
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
static bStatus_t Controller_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                       uint8 *pValue, uint16 *pLen, uint16 offset,
                                       uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the Voltage Characteristic Value
  if (! memcmp(pAttr->type.uuid, Controller_VoltageUUID, pAttr->type.len) )
  {
    if ( offset > CONTROLLER_VOLTAGE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CONTROLLER_VOLTAGE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
//  // See if request is regarding the Current Characteristic Value
//  else if (! memcmp(pAttr->type.uuid, Controller_CurrentUUID, pAttr->type.len) )
//  {
//    if ( offset > CONTROLLER_CURRENT_LEN )  // Prevent malicious ATT ReadBlob offsets.
//    {
//      status = ATT_ERR_INVALID_OFFSET;
//    }
//    else
//    {
//      *pLen = MIN(maxLen, CONTROLLER_CURRENT_LEN - offset);  // Transmit as much as possible
//      memcpy(pValue, pAttr->pValue + offset, *pLen);
//    }
//  }
  // See if request is regarding the Heat Sink Temperature Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Controller_Heat_Sink_TemperatureUUID, pAttr->type.len) )
  {
    if ( offset > CONTROLLER_HEAT_SINK_TEMPERATURE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CONTROLLER_HEAT_SINK_TEMPERATURE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Error Code Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Controller_Error_CodeUUID, pAttr->type.len) )
  {
    if ( offset > CONTROLLER_ERROR_CODE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CONTROLLER_ERROR_CODE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Motor RPM Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Controller_Motor_RPM_UUID, pAttr->type.len) )
  {
    if ( offset > CONTROLLER_MOTOR_RPM_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CONTROLLER_MOTOR_RPM_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Motor Speed Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Controller_Motor_SpeedUUID, pAttr->type.len) )
  {
    if ( offset > CONTROLLER_MOTOR_SPEED_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CONTROLLER_MOTOR_SPEED_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Total Distance Travelled Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Controller_Total_Distance_TravelledUUID, pAttr->type.len) )
  {
    if ( offset > CONTROLLER_TOTAL_DISTANCE_TRAVELLED_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CONTROLLER_TOTAL_DISTANCE_TRAVELLED_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Total Energy Consumption Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Controller_Total_Energy_ConsumptionUUID, pAttr->type.len) )
  {
    if ( offset > CONTROLLER_TOTAL_ENERGY_CONSUMPTION_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CONTROLLER_TOTAL_ENERGY_CONSUMPTION_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Total Energy Efficiency Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Controller_Overall_EfficiencyUUID, pAttr->type.len) )
  {
    if ( offset > CONTROLLER_OVERALL_EFFICIENCY_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CONTROLLER_TOTAL_ENERGY_CONSUMPTION_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Range Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Controller_RangeUUID, pAttr->type.len) )
  {
    if ( offset > CONTROLLER_RANGE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CONTROLLER_RANGE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the CO2SAVED Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Controller_co2SavedUUID, pAttr->type.len) )
  {
    if ( offset > CONTROLLER_CO2SAVED_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CONTROLLER_CO2SAVED_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Motor Temperature Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Controller_Motor_TemperatureUUID, pAttr->type.len) )
  {
    if ( offset > CONTROLLER_MOTOR_TEMPERATURE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CONTROLLER_MOTOR_TEMPERATURE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Instant economy Characteristic Value
  else if (! memcmp(pAttr->type.uuid, Controller_Instant_EconomyUUID, pAttr->type.len) )
  {
    if ( offset > CONTROLLER_INSTANT_ECONOMY_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CONTROLLER_INSTANT_ECONOMY_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      Controller_WriteAttrCB
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
static bStatus_t Controller_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                        uint8 *pValue, uint16 len, uint16 offset,
                                        uint8 method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;
  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF){
    if ( pAppCBs && pAppCBs->pfnChangeCb ){
      pAppCBs->pfnChangeCb( paramID ); // Call app function from stack task context.
    }
  }
  return status;
}


