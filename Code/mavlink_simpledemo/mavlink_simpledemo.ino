// Include MAVLink header
// Remember that it's necessary to init and update git submodule
#include "c_library_v2/common/mavlink.h"

struct DrifterData{ // Data to capture from each Drifter RFM signal
  uint8_t drifterID;
  int32_t lat;
  int32_t lon;
  float voltage;
  char Program_Status; // 'G' - go (recording data), 'R' - recovery, 'S' - shutdown
  elapsedMillis sinceRFM; //track: millis() time since last RFM signal (or use UTC time since last signal)
};

#define MAX_NUM_DRIFTERS 12
DrifterData drifters[MAX_NUM_DRIFTERS];
uint8_t drifterIDi=0; // current IDX to send mssgs
uint8_t totalnumdrifters=0; // the true length of drifterIDs

// Timing Variables
const unsigned int heartbeat_interval= 1000;
uint32_t lastTime_heartbeat = 0;

// MavLink Custom parameters
const char* param_names[] = {"RECOVERY_BAT_VOLTAGE_TRIGGER", "SHUTDOWN_BAT_VOLTAGE_TRIGGER"};
#define PARAM_COUNT (sizeof(param_names) / sizeof(param_names[0])) // Define PARAM_COUNT as the number of elements in the array
float param_values[PARAM_COUNT] = {3.75, 3.7};

/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                Helper Functions
 * ----------------- ------------------ ------------------ ------------------ ------------------*/

void updateDrifterData(uint8_t id, int32_t latitude, int32_t longitude, float battVoltage, char Program_Status) {
  bool drifterExists = false;
  //check if drifterID already exists
  for (uint8_t i = 0; i < MAX_NUM_DRIFTERS; i++) {
    if (drifters[i].drifterID == id) {
      drifters[i].lat = latitude;
      drifters[i].lon = longitude;
      drifters[i].voltage = battVoltage;
      drifters[i].Program_Status= Program_Status;
      drifterExists = true;
      break;
    }
  }
  // If the drifterID does not exist, add a new entry
  if (!drifterExists) {
    for (uint8_t i = 0; i < 12; i++) {
      if (drifters[i].drifterID == 0) { // Check for an uninitialized slot
        drifters[i].drifterID = id;
        drifters[i].lat = latitude;
        drifters[i].lon = longitude;
        drifters[i].voltage = battVoltage;
        drifters[i].Program_Status= Program_Status;
        break;
      }
    }
  }
}

/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                RFM Functions
 * ----------------- ------------------ ------------------ ------------------ ------------------*/


/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                MavLink Functions
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
void decode_messages() {
  static mavlink_message_t message;
  static mavlink_status_t status;

  // Read all data available in serial port and parse the mavlink message
  while(Serial.available() > 0) {
    uint8_t serial_byte = Serial.read();

    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, serial_byte, &message, &status)) {
      // Handle message
      switch(message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          // Blink when a HEARTBEAT is received
          //FLAG: you must perform serial.read on the returning Heartbeat mssg for Mission Planner to connect
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
          break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
          send_all_parameters();
          break;
        case MAVLINK_MSG_ID_COMMAND_LONG:{
          // Decode COMMAND_LONG message
          mavlink_command_long_t cmd;
          mavlink_msg_command_long_decode(&message, &cmd);
          if (cmd.command == MAV_CMD_DO_SET_HOME) {           // Check if the command is MAV_CMD_DO_SET_HOME
            if (cmd.param1 == 1) { // param1 == 1 means set to specific coordinates
              float home_lat = cmd.param5; // Latitude
              float home_lon = cmd.param6; // Longitude
              float home_alt = cmd.param7; // Altitude

              // Call your function to set the home position
              set_home_position(home_lat, home_lon, home_alt);

              // Optionally, send acknowledgment
              send_command_ack(message.sysid, message.compid, MAV_CMD_DO_SET_HOME, MAV_RESULT_ACCEPTED);
            }
          }
        } break;
        default:
          break;
      }
    }
  }
}

void send_heartbeat() {
  mavlink_message_t msg;
  uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t mavlink_message_length = 0;
  mavlink_msg_heartbeat_pack(
    drifterIDi, // Use each drifter's ID as system_id
    1,
    &msg,
    MAV_TYPE_SUBMARINE,
    MAV_AUTOPILOT_INVALID,
    MAV_MODE_FLAG_HIL_ENABLED, // hardware in the loop simulation.  Motors/actuators blocked.
    0x0000, // No custom flag
    MAV_STATE_ACTIVE
  );
  mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, &msg);
  Serial.write(mavlink_message_buffer, mavlink_message_length);
}

void send_command_ack(uint8_t system_id, uint8_t component_id, uint16_t command, uint8_t result) {
    mavlink_message_t ack_msg;
    mavlink_msg_command_ack_pack(system_id, component_id, &ack_msg, command, result);
    
    uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, &ack_msg);
    Serial.write(mavlink_message_buffer, mavlink_message_length);
}

// void sampleincomingRFM(){


// }

void send_all_parameters() {
  mavlink_message_t msg;
  uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t mavlink_message_length = 0;
  for (uint16_t i = 0; i < PARAM_COUNT; i++) {
    mavlink_msg_param_value_pack(
      1, // PARAMS ARE UNIVERSAL
      1,
      &msg,
      param_names[i],
      param_values[i],
      MAV_PARAM_TYPE_REAL32,
      PARAM_COUNT,
      i
    );
    mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, &msg);
    Serial.write(mavlink_message_buffer, mavlink_message_length);
  }
}

void send_battery_status(float voltage) {
    mavlink_message_t msg;
    uint16_t voltages[10] = {UINT16_MAX}; // Initialize all cells to UINT16_MAX
    uint16_t voltage_mv = voltage * 1000; // Convert voltage to millivolts
    voltages[0] = voltage_mv; // Assuming a single-cell battery, store total voltage in cell 0

    int16_t current_battery = -1; // Current in centiAmps, -1 if not measured
    int32_t current_consumed = -1; // Charge consumed in mAh, -1 if not measured
    int32_t energy_consumed = -1; // Energy consumed in hJ, -1 if not measured
    int8_t battery_remaining = -1; // Battery remaining percentage, -1 if not measured

    mavlink_msg_battery_status_pack(
        drifterIDi,                  // system ID
        1,                           // component ID
        &msg,
        0,                           // Battery ID, usually 0 for a single battery system
        MAV_BATTERY_FUNCTION_ALL,    // Battery function (e.g., MAV_BATTERY_FUNCTION_ALL)
        MAV_BATTERY_TYPE_LIPO,       // Battery type (e.g., LiPo)
        INT16_MAX,                   // Temperature, INT16_MAX if unknown
        voltages,                    // Battery voltage array
        current_battery,             // Battery current in centiAmps
        current_consumed,            // Consumed charge in mAh
        energy_consumed,             // Consumed energy in hJ
        battery_remaining,           // Remaining battery percentage
        0,                           // Remaining battery time in seconds, 0 if not available
        MAV_BATTERY_CHARGE_STATE_OK, // Battery charge state (e.g., OK, low, critical)
        {0},                         // Extended battery voltages for cells 11-14 (set to 0)
        0,                           // Battery mode (0 if not used)
        0                            // Fault/health bitmask (0 if not used)
    );

    uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, &msg);
    Serial.write(mavlink_message_buffer, mavlink_message_length);
}

void send_gps_position(int32_t lat, int32_t lon) {
    mavlink_message_t msg;
    // Packing the GLOBAL_POSITION_INT message
    mavlink_msg_global_position_int_pack(
        drifterIDi,        // system ID (set to drifterIDi)
        1,                 // component ID (can be left as 1)
        &msg,
        millis(),          // time since boot
        lat,               // latitude in 1E7 degrees
        lon,               // longitude in 1E7 degrees
        0,                 // altitude in mm (set to 0 if not used)
        0,                 // relative altitude in mm (set to 0 if not used)
        0,                 // velocity in X direction (set to 0 if not used)
        0,                 // velocity in Y direction (set to 0 if not used)
        0,                 // velocity in Z direction (set to 0 if not used)
        UINT16_MAX         // heading in centidegrees (set to UINT16_MAX if not used)
    );
    uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, &msg);
    Serial.write(mavlink_message_buffer, mavlink_message_length);
}

/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                            SETUP
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
void setup() {
  // MAVLink serial port
  Serial.begin(9600); // default baudrate for QGC

  // Enable LED
  pinMode(LED_BUILTIN, OUTPUT);

  //FLAG //DEBUG //WARNING: Automatically init a drifter1
  uint8_t id = 1; // would be radio.senderID
  int32_t lat = 328697060; // Replace with your latitude in 1E7 degrees
  int32_t lon = -1172555570; // Replace with your longitude in 1E7 degrees
    float voltage = 3.7; // Replace with your voltage reading
  char Program_Status = 'G'; // good
  updateDrifterData(id, lat, lon, voltage, Program_Status);

  //FLAG //DEBUG //WARNING: Automatically init a drifter2
  id = 2; // would be radio.senderID
  lat = 32.864353785463074e+7; // Replace with your latitude in 1E7 degrees
  lon = -117.25699964626796e+7; // Replace with your longitude in 1E7 degrees
  voltage = 3.33; // Replace with your voltage reading
  Program_Status = 'G'; // good
  updateDrifterData(id, lat, lon, voltage, Program_Status);
}

void loop() {
  const uint32_t current_time = millis();
  
  // Send heartbeat via serial port
  if (current_time - lastTime_heartbeat > heartbeat_interval){ // On a 1 hz interval:
    for(uint8_t i = 0; i < MAX_NUM_DRIFTERS; i++) { // loop through each connected drifterID
      if (drifters[i].drifterID != 0) {
        drifterIDi=drifters[i].drifterID; // set system ID to current drifter
        send_heartbeat();
        send_battery_status(drifters[i].voltage); //FLAG: voltage is temporary
        send_gps_position(drifters[i].lat, drifters[i].lon);
        lastTime_heartbeat=current_time;
      }
    }
  }

  // Check reception buffer
  decode_messages();
}