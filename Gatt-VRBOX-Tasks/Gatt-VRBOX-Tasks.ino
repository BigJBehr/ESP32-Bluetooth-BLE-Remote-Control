//*****************************************************************************
// File Name:   Gatt-VRBOX-Tasks.ino
// Author:      James R. Behrens
// Project:     ESP32 Bluetooth BLE client for VRBOX BLE server
// Compiler:    Arduino IDE Ver 1.8.5 with ESP32 plugin
// Target:      ESP32 on DoIt Devkit Ver 1
//*****************************************************************************
//  REVISION HISTORY:
//
//  Date    Who Remark:
// ======== === ===============================================================
// 03-15-18 JRB Ported from Gatt-Client. Modified to work with VRBOX BLE server
// 04-18-18 JRB Modified to use FreeRTOS tasks for the notification events.
//*****************************************************************************
// This project is a port of the "how-to-use-arduino-esp32-ble-as-gatt-client"
// demo downloaded from; 
// http://www.iotsharing.com/2017/07/how-to-use-arduino-esp32-ble-as-gatt-client.html
// The demo has been extensively modified to work with the VRBOX server device.
//*****************************************************************************

// VRBOX is a handheld Bluetooth BLE device with a joystick and six useful
// buttons. This code shows how to setup the ESP32 as a BLE client to work
// with the VRBOX BLE server. Most of the code is generic to any BLE server.
// The name of the BLE server to search for and the services and characteristics
// are specific to the VRBOX server.
// You can use this code as a primer on how to connect other BLE servers to the
// ESP32 or as the basis of any ESP32 project that you want to control using the
// VRBOX device. The code should work with any ESP32 module.

// The VR BOX server will shut itself off if there is no activity for 5 minutes.
// This will cause the ESP32 to lose the Bluetooth connection and do a reset.

// Bluetooth files are here: C:\Users\BigJ\Documents\Arduino\hardware\espressif\esp32\tools\sdk\include\bluedroid
// The user name (BigJ) will be different on your computer

#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-fpermissive"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "controller.h"
#include "bt.h"
#include "bt_trace.h"
#include "bt_types.h"
#include "btm_api.h"
#include "bta_api.h"
#include "bta_gatt_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"

#pragma GCC diagnostic pop

// uncomment to see all of the Bluetooth debug messages
//#define VERBOSE

// Each defined task requires its own dedicated RAM allocation for its stack
// How much RAM to allocate is determined by how nested the task is and how
// many variables are stored on the stack (local variables). The default size
// defined here is for 5K * 4 bytes/32 bit word, so 20K bytes
// If you are getting;
// JGuru Meditation Error: Core  0 panoc'd (Unhandled debug exception)
// Debug excaption reason: Stack canary watchpoint triggered (task name)
// then increase the TaskStackSize by 1024 until the Stack canary errors stop. 
// The ESP32 VROOM module has 288K bytes of RAM.
#define TaskStackSize   5120

// The blue and green LEDs are used to indicate when the ESP32 is scanning for
// BLE servers and when the ESP32 has connected to the VRBOX server. The LEDs
// can be moved to any GPIO pin (except LED, that is the builtin blue LED) that
// is not input only.

// ===== Allocate GPIO of the ESP32 =====
#define LED           2         // built-in blue LED, high -> on, low -> off
#define BLUELED       15        // lit -> connected
#define GREENLED      5         // lit -> scanning
#define REDLED        4

#define SCL           22        // I2C SCL
#define SDA           21        // I2C SDA

// these values are for a common anode tri-color LED. if you use a common
// cathode then swap the on/off definitions
#define LEDON         LOW
#define LEDOFF        HIGH

// Services Available on VR Box
#define GATT_UUID_GENERIC_ACCESS              0x1800    // device name, appearance, peripheral preferred connection parameters
#define GATT_UUID_GENERIC_ATTRIBUTE           0x1801    // generic attribute

// These are defined in esp_gatt_defs.h
// ESP_GATT_UUID_DEVICE_INFO_SVC         0x180A    // PNP ID (Vid & Pid)
// ESP_GATT_UUID_BATTERY_SERVICE_SVC     0x180F    // Battery Service
// ESP_GATT_UUID_HID_SVC                 0x1812    // HID Info, report map, reports, HID control point, protocol mode

// forward declaration of callback functions
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_hid_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_battery_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

// name of BLE server that you want to connect to
static const char device_name[] = "VR BOX";
static bool connect = false;

// BLE scan params
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};

// HID service has ten characteristics
#define CHAR_NUM 10

// a GATT profile is a GATT service. we need an instance of this structure
// for each service we want to use.
// this structure holds the information for a GATT profile
typedef struct gattc_profile_inst
{
    esp_gattc_cb_t gattc_cb;          // callback handler for this profile
    uint16_t gattc_if;                // client interface number
    uint16_t app_id;                  // profile ID
    uint16_t conn_id;                 // connection ID
    uint16_t start_handle;            // service starting handle
    uint16_t end_handle;              // service ending handle
    uint16_t char_handle[CHAR_NUM];   // array of active characteristic handles for this service
    esp_bd_addr_t remote_bda;         // MAC address
} gattc_profile_t;


// these values will be used to index the array of service profiles
enum
{
  HID_PROFILE = 0,
  BATTERY_PROFILE,
  NUMPROFILES
};

// this is an array of gattc profiles. each entry is the profile for a service we want to use
static gattc_profile_t profiles[NUMPROFILES] = {
  {.gattc_cb = gattc_hid_event_handler,       // pointer to the GATTC event handler for the service 
   .gattc_if = ESP_GATT_IF_NONE},             // gsttc_if is unknown at this point
  {.gattc_cb = gattc_battery_event_handler,   // pointer to the GATTC event handler for the service  
   .gattc_if = ESP_GATT_IF_NONE},             // gsttc_if is unknown at this point
};


enum
{
  VB_TRIGGERS = 0,
  VB_JOYX,
  VB_JOYY,
  VB_BTNAB,
  VB_BTNCD,
  VB_NUMBYTES
};

// ===== VR Box Buttom Masks =====
#define VB_LOW_TRIGGER    0x01
#define VB_UPR_TRIGGER    0x02
#define VB_BUTTON_A       0x10
#define VB_BUTTON_B       0x20
#define VB_BUTTON_C       0x01
#define VB_BUTTON_D       0x02
#define FRESHFLAG         0x80

#define JOYTIMEOUT        30      // joystick no activity timeout in mS

#define JoyStickDeadZone  0

// This is where we store the data from the buttons and joystick
volatile byte   VrBoxData[VB_NUMBYTES];
volatile bool   flag = false;         // indicates new data to process
bool            scanning = false;

byte            lastTrig = 0;         // trigger buttons last value
byte            lastAB = 0;           // A/B buttons last value
byte            lastCD = 0;           // C/D buttons last value
byte            lastX = 0;            // joystick's last values
byte            lastY = 0;

// joyTimer is a 30 millisecond re-triggerable timer that sets the joystick 
// back to center if no activity on the joystick or trigger buttons. 
volatile uint32_t joyTimer = millis;

// task handles  
TaskHandle_t HandleJS = NULL;   // handle of the joystick task
TaskHandle_t HandleAB = NULL;   // handle of the joystick task
TaskHandle_t HandleCD = NULL;   // handle of the joystick task

//******************************************************************************
// This is the GATT Client callback handler for the HID service
//******************************************************************************
static void gattc_hid_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
#ifdef VERBOSE
  Serial.println("\nHID Profile event");
#endif
  gattc_profile_event_handler(event, gattc_if, param, HID_PROFILE);
} //  gattc_hid_event_handler

//******************************************************************************
// This is the GATT Client callback handler for the Battery service
//******************************************************************************
static void gattc_battery_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
#ifdef VERBOSE
  Serial.println("\nBattery Profile event");
#endif
  gattc_profile_event_handler(event, gattc_if, param, BATTERY_PROFILE);
} //  gattc_battery_event_handler

//******************************************************************************
// This callback will be invoked when GATT BLE events come.
// Refer GATT Client callback function events here: 
// https://github.com/espressif/esp-idf/blob/master/components/bt/bluedroid/api/include/esp_gattc_api.h
//******************************************************************************
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param, int idx)
{
  uint16_t conn_id = 0;
  esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

#ifdef VERBOSE
  Serial.printf("Entry Profile index: %d\n", idx);
#endif

  switch (event)
  {
    case ESP_GATTC_REG_EVT:
    {
      #ifdef VERBOSE
        Serial.println("ESP_GATTC_REG_EVT");
      #endif
      break;
    }
    
    // this event occurs when the connection is set up
    case ESP_GATTC_OPEN_EVT: 
    {
      #ifdef VERBOSE
        Serial.println("ESP_GATTC_OPEN_EVT");
      #endif
      
      conn_id = p_data->open.conn_id;

      // copy MAC address to both service profiles
      memcpy(profiles[HID_PROFILE].remote_bda,     p_data->open.remote_bda, sizeof(esp_bd_addr_t));
      memcpy(profiles[BATTERY_PROFILE].remote_bda, p_data->open.remote_bda, sizeof(esp_bd_addr_t));

      // initialte service search
      #ifdef VERBOSE
        Serial.printf("Service index: %d, gattc_if: %d, conn_id: %d\n",idx, gattc_if, conn_id); 
//      Serial.printf("Battery service gattc_if: %d, conn_id: %d\n", profiles[BATTERY_PROFILE].gattc_if, conn_id); 
      #endif
      
      if (profiles[HID_PROFILE].gattc_if == gattc_if)
      {
        // last parameter NULL -> find all services, UUID of a service -> find only that service
        esp_ble_gattc_search_service(profiles[HID_PROFILE].gattc_if, conn_id, NULL);
      }
      else if (profiles[BATTERY_PROFILE].gattc_if == gattc_if)
      {
        esp_ble_gattc_search_service(profiles[BATTERY_PROFILE].gattc_if, conn_id, NULL);
      }
      
      profiles[HID_PROFILE].conn_id      = conn_id;
      profiles[BATTERY_PROFILE].conn_id  = conn_id;
      break;
    }
    
    // this event occurs when GATT service discovery result happens
    // this event occurs once for each service discovered (five times for VRBOX)
    case ESP_GATTC_SEARCH_RES_EVT: 
    {
      #ifdef VERBOSE
        Serial.println("ESP_GATTC_SEARCH_RES_EVT");
      #endif
      
      esp_gatt_srvc_id_t *srvc_id = (esp_gatt_srvc_id_t *)&p_data->search_res.srvc_id;
      conn_id = p_data->search_res.conn_id;
      
      // see if the remote device has the services we are interested in
      if (srvc_id->id.uuid.len == ESP_UUID_LEN_16)
      {
        switch (srvc_id->id.uuid.uuid.uuid16)
        {
          case ESP_GATT_UUID_HID_SVC:
            #ifdef VERBOSE
              Serial.println("HID service found");
            #endif
            
            // save the start and end handles for the service's characteristics
            profiles[HID_PROFILE].start_handle = p_data->search_res.start_handle;
            profiles[HID_PROFILE].end_handle   = p_data->search_res.end_handle;
        
            #ifdef VERBOSE
              Serial.printf("HID handles: %d - %d\n", p_data->search_res.start_handle, p_data->search_res.end_handle);
            #endif
            break;

          case ESP_GATT_UUID_BATTERY_SERVICE_SVC:
            #ifdef VERBOSE
              Serial.println("Battery service found");
            #endif
            
            // save the start and end handles for the service's characteristics
            profiles[BATTERY_PROFILE].start_handle = p_data->search_res.start_handle;
            profiles[BATTERY_PROFILE].end_handle   = p_data->search_res.end_handle;
      
            #ifdef VERBOSE
              Serial.printf("Battery handles: %d - %d\n", p_data->search_res.start_handle, p_data->search_res.end_handle);
            #endif
            break;

          default:
            #ifdef VERBOSE
              Serial.printf("Service UUID: %X found\n", srvc_id->id.uuid.uuid.uuid16);
            #endif
            break;
        } //  switch
      }
      break;
    }
    
    // this event occurs when GATT service discovery is completed
    case ESP_GATTC_SEARCH_CMPL_EVT: 
    {
      #ifdef VERBOSE
        Serial.println("ESP_GATTC_SEARCH_CMPL_EVT");
        Serial.printf("Profile index: %d\n", idx);
      #endif
      
      // store connection id for later usage
      conn_id = p_data->search_cmpl.conn_id;

      esp_gattc_char_elem_t *char_elements;
      uint16_t char_offset = 0;
      uint16_t count = 0;
      gattc_profile_t *pProfile;  

      // we need to do this for each service we are using
//      for (int service = 0; service < NUMPROFILES; service++)
      int service = idx;
      {
        pProfile = &profiles[service];
        #ifdef VERBOSE
          Serial.printf("Characteristics for Service: %d, gattc_if: %d\n", service, pProfile->gattc_if);
        #endif
        
        // get characteristics of service. It just gets characteristic from local cache, won't get from remote devices
        esp_gatt_status_t status = esp_ble_gattc_get_attr_count(pProfile->gattc_if,
                                       pProfile->conn_id,
                                       ESP_GATT_DB_CHARACTERISTIC,
                                       pProfile->start_handle,
                                       pProfile->end_handle,
                                       ESP_GATT_INVALID_HANDLE,
                                       &count);
        if (count > 0)
        {
          char_elements = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
          status = esp_ble_gattc_get_all_char(pProfile->gattc_if, 
                                    pProfile->conn_id, 
                                    pProfile->start_handle, 
                                    pProfile->end_handle, 
                                    char_elements, &count, char_offset);                                     
          // show the characteristc info
          for (int i = 0; i < count; i++)
          {
            #ifdef VERBOSE
              Serial.printf("%d) char uuid = %x, char_handle = %d\n", i, char_elements[i].uuid.uuid.uuid16, char_elements[i].char_handle);
            #endif
            pProfile->char_handle[i] = char_elements[i].char_handle;
        
            // We use BLE notifications to monitor the buttons and joystick of the server.
            // VR Box sends a notification to a registered handle when a change occurs.
            // Here we register the characteristics we want to receive notifications for.
            // if notify bit set then register to receive notification from GATT server
            
            // if the characteristic is a report and can send notifications then register it for
            // notifications
            if (((ESP_GATT_UUID_BATTERY_LEVEL == char_elements[i].uuid.uuid.uuid16) ||
                 (ESP_GATT_UUID_HID_REPORT == char_elements[i].uuid.uuid.uuid16)) &&
                (char_elements[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
            {
              #ifdef VERBOSE
                Serial.println("Register for notification");
              #endif
            
              // if it set then we enable notify, the event ESP_GATTC_REG_FOR_NOTIFY_EVT will be raised when finishing
              esp_ble_gattc_register_for_notify (gattc_if, pProfile->remote_bda, pProfile->char_handle[i]);
            }
          } //  for each characteristic

          free(char_elements);
        }
        else
        {
          #ifdef VERBOSE
            Serial.println("No characteristics found");
          #endif
        }
      } // for each service

      if (idx == 0)
      {
        #ifdef VERBOSE
          Serial.printf("GATT Opening Battery service: %d\n", profiles[BATTERY_PROFILE].gattc_if);
        #endif
        
        esp_ble_gattc_open(profiles[BATTERY_PROFILE].gattc_if, profiles[BATTERY_PROFILE].remote_bda, true);
      }
      
      break;
    }
    
    // this event occurs when the physical connection is disconnected
    case ESP_GATTC_DISCONNECT_EVT:
    {
      #ifdef VERBOSE
        Serial.println("ESP_GATTC_DISCONNECT_EVT");
      #endif
      
      // brute force method, but works and is fairly quick !!!
      ESP.restart();
      break;
    }
    
    // this event occurs when register for notification of a characteristic completes
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
    {
      #ifdef VERBOSE
        Serial.println("ESP_GATTC_REG_FOR_NOTIFY_EVT");
      #endif
      break;
    }
        
    // this event occurs when the server sends a notification for a registered service/characteristic
    case ESP_GATTC_NOTIFY_EVT:
    {
      #ifdef VERBOSE
        Serial.println("ESP_GATTC_NOTIFY_EVT");
        // idx == HID_PROFILE -> HID service notification
        // idx == BATTERY_PROFILE -> Battery service notification
        Serial.printf("Profile index: %d\n", idx);
      #endif
      
      if (HID_PROFILE == idx)
      {
        // we are getting the two trigger buttons in the first byte, joyX & joyY in 2nd & 3rd bytes
        // A four byte report is the joystick/trigger buttons.
        // A two byte report is either the A/B buttons or the C/D buttons
        // Low nibble equal to 0x05 indicates A/B buttons.
        // A/B buttons auto-repeat if held. No other buttons do this.
        if (4 == p_data->notify.value_len)
        {
          // show the received data
//          for (int i = 0; i < p_data->notify.value_len; i++)
//            Serial.printf("%02X ", p_data->notify.value[i]);
//          Serial.println();
  
          // copy data to VrBoxData
          for (int i = VB_TRIGGERS; i < VB_BTNAB; i++)
            VrBoxData[i] = p_data->notify.value[i];

          // wake up the joystick/trigger buttons handler task
          if (HandleJS)
            vTaskResume(HandleJS);
            
          // restart the joystick timer
          joyTimer = millis() + JOYTIMEOUT;
        }
        else if (2 == p_data->notify.value_len)
        {
          // show the received data
//          Serial.printf("data: %02X\n", p_data->notify.value[0]);
          if (0x05 == (p_data->notify.value[0] & 0x0F))
          {
            // A/B button report, wake the A/B button handler task
            VrBoxData[VB_BTNAB] = p_data->notify.value[0] & 0xF0;
            if (HandleAB)
              vTaskResume(HandleAB);
          }
          else
          {
            // C/D button report, wake the C/D button handler task
            VrBoxData[VB_BTNCD] = p_data->notify.value[0];
            if (HandleCD)
              vTaskResume(HandleCD);
          }
        }
      }
      else if (BATTERY_PROFILE == idx)
      {
        Serial.println("Battery Data");
        for (int i = 0; i < p_data->notify.value_len; i++)
          Serial.printf("%02X ", p_data->notify.value[i]);
        Serial.println();
      }
      break;
    }
    
    default:
      #ifdef VERBOSE
        Serial.printf("Unhandled GATT Profile Event: %X\n", event);
      #endif
      break;
  } //  switch event
} //  gattc_profile_event_handler

//******************************************************************************
// This callback will be invoked when GAP advertising events come. GAP is used
// to scan for available servers.
// Refer GAP BLE callback event type here: 
// https://github.com/espressif/esp-idf/blob/master/components/bt/bluedroid/api/include/esp_gap_ble_api.h
//******************************************************************************
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
  uint8_t *adv_name = NULL;
  uint8_t adv_name_len = 0;
  switch (event)
  {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    {
      #ifdef VERBOSE
        Serial.println("ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT");
      #endif
      
      StartScan();
      break;
    }
    
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
    {
      #ifdef VERBOSE
        Serial.println("ESP_GAP_BLE_SCAN_START_COMPLETE_EVT");
      #endif
      
      //scan start complete event to indicate scan start successfully or failed
      if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
      {
          Serial.printf("\nScan start failed");
      }
      break;
    }

    // processing scan result
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
      #ifdef VERBOSE
        Serial.println("ESP_GAP_BLE_SCAN_RESULT_EVT");
      #endif
      
      esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
      switch (scan_result->scan_rst.search_evt)
      {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
          #ifdef VERBOSE
            Serial.println("ESP_GAP_SEARCH_INQ_RES_EVT");
          #endif
          
          adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
          if (adv_name != NULL)
          {
            if (strlen(device_name) == adv_name_len && strncmp((char *)adv_name, device_name, adv_name_len) == 0)
            {
              // if connection is established then stop scanning 
              if (connect == false)
              {
                connect = true;
                digitalWrite(BLUELED, LEDON);

                Serial.printf("Connected to the remote device %s\n", device_name);
                esp_ble_gap_stop_scanning();
                #ifdef VERBOSE
                  Serial.printf("GAP Opening HID service: %d\n", profiles[HID_PROFILE].gattc_if);
                #endif
                esp_ble_gattc_open(profiles[HID_PROFILE].gattc_if, scan_result->scan_rst.bda, true);
              }
            }
          }
          break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
          #ifdef VERBOSE
            Serial.println("ESP_GAP_SEARCH_INQ_CMPL_EVT");
          #endif
          scanning = false;
          digitalWrite(GREENLED, LEDOFF);
          break;
          
        default:
          break;
      }
      break;
    }
  
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
    {
      #ifdef VERBOSE
        Serial.println("ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT");
      #endif
      if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
      {
        Serial.println("Scan stop failed");
      }
      else
      {
        #ifdef VERBOSE
          Serial.println("Stop scan successfully");
        #endif
        scanning = false;
        digitalWrite(GREENLED, LEDOFF);
      }
      break;
    }
    
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
    {
      #ifdef VERBOSE
        Serial.printf("\nESP_GAP_BLE_ADV_STOP_COMPLETE_EVT\n");
      #endif
      if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
          Serial.printf("\nAdv stop failed\n");
      }
      else
      {
        #ifdef VERBOSE
          Serial.printf("\nStop adv successfully");
        #endif
      }
      break;
    }
      
    default:
      #ifdef VERBOSE
        Serial.printf("Unhandled GAP event: %X\n", event);
      #endif
      break;
  } //  switch event
} //  esp_gap_cb

//******************************************************************************
// This is the GATT event callback handler function. Handles GATTC events common
// to all services. Uses gattc_if parameter to decide which GATTC service
// handler to invoke.
//******************************************************************************
// modify this to trap ESP_GATTC_NOTIFY_EVT. then select handler based on gattc_if.

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
  // If event is register event, store the gattc_if for each profile
  if (event == ESP_GATTC_REG_EVT)
  {
    // aram->reg.app_id as the service profile index is only valid for this event !!
    #ifdef VERBOSE
      Serial.printf("ESP_GATTC_REG_EVT trap for: %d, gattc_if: %d\n", param->reg.app_id, gattc_if);
    #endif
  
    if (param->reg.status == ESP_GATT_OK)
    {
      // save gsttc_if value for the service profile selected by param->reg.app_id 
      profiles[param->reg.app_id].gattc_if = gattc_if;
    } 
    else
    {
      Serial.printf("Reg app failed, profile: %04x, status %d\n", param->reg.app_id, param->reg.status);
      return;
    }
  }

  // invoke the event handler of each registered service profile
  for (int idx = 0; idx < NUMPROFILES; idx++)
  {
    if (gattc_if == ESP_GATT_IF_NONE || gattc_if == profiles[idx].gattc_if)
    {
      if (profiles[idx].gattc_cb)
      {
        profiles[idx].gattc_cb(event, profiles[idx].gattc_if, param);
      }
    }
  } //  for
} //  esp_gattc_cb

//******************************************************************************
// This registers the functions we want to use to handler GAP and GATTC 
// callbacks. We only need to do this once for all profiles.
//******************************************************************************
void ble_client_appRegister(void)
{
  esp_err_t status;

  #ifdef VERBOSE
    Serial.printf("\nregister callback");
  #endif
  
  // register the scan callback function to the gap module
  if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK)
  {
    Serial.printf("\ngap register error, error code = %x\n", status);
    return;
  }
  
  // register the callback function to the gattc module
  if ((status = esp_ble_gattc_register_callback(esp_gattc_cb)) != ESP_OK)
  {
    Serial.printf("gattc register error, error code = %x\n", status);
    return;
  }
  
  esp_ble_gattc_app_register(HID_PROFILE);
  esp_ble_gattc_app_register(BATTERY_PROFILE);
} //  ble_client_appRegister

//******************************************************************************
//******************************************************************************
void gattc_client_test(void)
{
  esp_bluedroid_init();
  esp_bluedroid_enable();
  ble_client_appRegister();
} //  gattc_client_test

//******************************************************************************
//******************************************************************************
void StartScan(void)
{
  if (!scanning)
  {
    #ifdef VERBOSE
      Serial.println("Scanning Started");
    #endif
    
    scanning = true;
    digitalWrite(GREENLED, LEDON);
    // the unit of the duration is a second
    uint32_t duration = 30;
    esp_ble_gap_start_scanning(duration);
  }
} //  StartScan  

// All of these tasks are designed to run forever. The tasks are resumed when
// a notification message is received with new data.
//******************************************************************************
// Joystick handler Task.
// Moving the joystick off center causes this task to be resumed about every
// 15ms. Press or release of either trigger button will also resume this task.
// If this task does not complete in less than 15mS you will lose joystick
// movement data !!!
// Holding the lower button will prevent the server from detecting that the 
// upper button has been pressed. 
// Holding the upper trigger and pressing the lower trigger results in the server
// sending a notification that the lower trigger is pressed (the upper trigger
// will be zero!). Releasing the lower trigger will cause the server to send a
// notification that the upper trigger is pressed and the lower trigger is
// released.
//******************************************************************************
void taskJoyStick(void *parameter)
{
  int8_t  x;
  int8_t  y;
  uint8_t triggers;
  
  //===== if the task requires any one time initialization, put it here =====

  // forever loop
  while(true)
  {
    // give up the CPU, wait for new data
    vTaskSuspend(NULL);

    // we just woke up, new data is available, convert joystick data to
    // signed 8 bit integers
    x = (int8_t)VrBoxData[VB_JOYX];
    y = (int8_t)VrBoxData[VB_JOYY];
    triggers = VrBoxData[VB_TRIGGERS];
    
    Serial.printf("Joystick X: %d, Y: %d Triggers: %02X\n", x, y, triggers);

    if (y < -JoyStickDeadZone)
    {
      // move forward
      Serial.println("Forward");
      
      //===== add your code here =====
      
    }
    else if (y > JoyStickDeadZone)
    {
      // move backward
      Serial.println("Backward");

      //===== add your code here =====
      
    }
        
    if (x > JoyStickDeadZone)
    {
      // turn right
      Serial.println("Turn Right");

      //===== add your code here =====
      
    }
    else if (x < -JoyStickDeadZone)
    {
      // turn left
      Serial.println("Turn Left");

      //===== add your code here =====
      
    }

    if (triggers & VB_LOW_TRIGGER)
    {
      // the lower trigger button is pressed
      Serial.println("Low Trigger Pressed");

      //===== add your code here =====
    }

    if (triggers & VB_UPR_TRIGGER)
    {
      // the upper trigger button is pressed
      Serial.println("Upper Trigger Pressed");

      //===== add your code here =====
      
    }
  } //  for
} //  taskJoyStick

//******************************************************************************
// A & B Buttons handler Task.
// Holding the A or B button down will cause this task to be invoked about every
// 15ms. If this task does not complete within 15mS you will lose button events.
// The AB buttons work similar to the trigger buttons in that the A button will
// prevent the B button from being detected and will override the B button when
// pressed while the B button is held down.
//******************************************************************************
void taskButtonAB(void *parameter)
{
  uint8_t buttons;

  //===== if the task requires any one time initialization, put it here =====
  
  while(true)
  {
    // give up the CPU, wait for new data
    vTaskSuspend(NULL);
    
    // we just woke up, new data is available
    buttons = VrBoxData[VB_BTNAB];
    Serial.printf("A/B Buttons: %02X\n", buttons);
       
    if (buttons & VB_BUTTON_A)
    {
      // button A pressed or is being held down
      Serial.println("Button A");
      
      //===== add your code here =====
      
    }

    if (buttons & VB_BUTTON_B)
    {
      // button B pressed or is being held down
      Serial.println("Button B");

      //===== add your code here =====
      
    }
  } //  for
} //  taskButtonAB

//******************************************************************************
// C & D Buttons handler Task. 
// Press or release of either the C or D button will resume this task. Holding
// one button down blocks the Server from detecting the other button being 
// pressed.
//******************************************************************************
void taskButtonCD(void *parameter)
{
  uint8_t buttons;
  
  //===== if the task requires any one time initialization, put it here =====
  
  while(true)
  {
    // give up the CPU
    vTaskSuspend(NULL);

    // we just woke up, new data is available
    buttons = VrBoxData[VB_BTNCD];
    Serial.printf("C/D Buttons: %02X\n", buttons);
    
    if (buttons & VB_BUTTON_C)
    {
      // button C pressed
      Serial.println("Button C");
      
      //===== add your code here =====
      
    }

    if (buttons & VB_BUTTON_D)
    {
      // button D pressed
      Serial.println("Button D");

      //===== add your code here =====
      
    }
  } //  for
} //  taskButtonCD

//******************************************************************************
//******************************************************************************
void setup()
{
  BaseType_t xReturned;
  
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(BLUELED, OUTPUT);       // lit -> connected
  digitalWrite(BLUELED, LEDOFF);
  pinMode(REDLED, OUTPUT);
  digitalWrite(REDLED, LEDOFF);
  pinMode(GREENLED, OUTPUT);      // lit -> scanning
  digitalWrite(GREENLED, LEDOFF);

  xReturned = xTaskCreate(taskJoyStick,             // task to handle activity on the joystick.
                          "Joystick",               // String with name of task.
                          TaskStackSize,            // Stack size in 32 bit words.
                          NULL,                     // Parameter passed as input of the task
                          1,                        // Priority of the task.
                          &HandleJS);               // Task handle.
  if (pdPASS == xReturned)
  {
    Serial.println("Joystick Task Created");
  }
 
  xReturned = xTaskCreate(taskButtonAB,             // task to handle activity on the A & B buttons.
                          "ButtonsAB",              // String with name of task.
                          TaskStackSize,            // Stack size in 32 bit words.
                          NULL,                     // Parameter passed as input of the task
                          1,                        // Priority of the task.
                          &HandleAB);               // Task handle.
  if (pdPASS == xReturned)
  {
    Serial.println("AB Button Task Created");
    
  }
 
  xReturned = xTaskCreate(taskButtonCD,             // task to handle activity on the C & D buttons.
                          "ButtonsCD",              // String with name of task.
                          TaskStackSize,            // Stack size in 32 bit words.
                          NULL,                     // Parameter passed as input of the task
                          1,                        // Priority of the task.
                          &HandleCD);               // Task handle.
  if (pdPASS == xReturned)
  {
    Serial.println("CD Button Task Created");
    
  }
 
  btStart();
  gattc_client_test();
} //  setup

//******************************************************************************
// connect flag is not reset when onnection is lost !!!!
//******************************************************************************
void loop()
{
  if (connect)
  {
    // joystick no activity detector
    if (joyTimer && (joyTimer < millis()))
    {
//      Serial.println("Timeout");
      // no joystick notification for 30mS, center the joystik
      VrBoxData[VB_JOYX] = VrBoxData[VB_JOYY] = 0;

      // wake up the joystick task
      vTaskResume(HandleJS);
      
      joyTimer = 0;
    }
//    delay(10);
  }
  else if (!scanning)
  {
    digitalWrite(BLUELED, LEDOFF);
    delay(5000);
    StartScan();
  }
} //  loop

