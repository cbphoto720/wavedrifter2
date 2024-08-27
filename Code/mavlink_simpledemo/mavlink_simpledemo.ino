// Include MAVLink header
// Remember that it's necessary to init and update git submodule
#include "c_library_v2/common/mavlink.h"

// Timing Variables
const unsigned int heartbeat_interval= 1000;
uint32_t lastTime_heartbeat = 0;

// Define your Custom parameters
#define PARAM_COUNT 3
const char* param_names[PARAM_COUNT] = {"PARAM1", "PARAM2", "PARAM3"};
float param_values[PARAM_COUNT] = {1.0, 2.0, 3.0};


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
        // case MAVLINK_MSG_ID_COMMAND_LONG:
        //   mavlink_command_long_t command;
        //   mavlink_msg_command_long_decode(&message, &command);
        //   if (command.command == MAV_CMD_REQUEST_MESSAGE && 
        //       command.param1 == MAVLINK_MSG_ID_GPS_RAW_INT) {
        //       // Send the GPS position when requested
        //       send_gps_status(current_lat, current_lon, current_alt, current_hdop);
        //   }
        //   break;
        default:
            break;
      }
    }
  }
}

void send_heartbeat() {
  static mavlink_message_t mavlink_message;
  static uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
  static uint16_t mavlink_message_length = 0;

  if (mavlink_message_length == 0) { // Create message if not
    const int system_id = 1;
    const int component_id = 1;
    const int mavlink_type = MAV_TYPE_GENERIC;
    const int autopilot_type = MAV_AUTOPILOT_INVALID;
    const int system_mode = MAV_MODE_PREFLIGHT;
    const int custom_mode = 0x0000; // No flag
    const int mavlink_state = MAV_STATE_ACTIVE;
    mavlink_msg_heartbeat_pack(
    system_id, component_id, &mavlink_message, mavlink_type, autopilot_type, system_mode, custom_mode, mavlink_state
  );
  mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, &mavlink_message);
  }
  Serial.write(mavlink_message_buffer, mavlink_message_length);
}

void send_all_parameters() {
  mavlink_message_t message;
  uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t mavlink_message_length = 0;
  int system_id = 1;
  int component_id = 1;

  for (uint16_t i = 0; i < PARAM_COUNT; i++) {
    mavlink_msg_param_value_pack(
      system_id,
      component_id,
      &message,
      param_names[i],
      param_values[i],
      MAV_PARAM_TYPE_REAL32,
      PARAM_COUNT,
      i
    );
    mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, &message);
    Serial.write(mavlink_message_buffer, mavlink_message_length);
  }
}

void send_battery_status(float voltage) {
    mavlink_message_t msg;
    uint16_t voltage_mv = voltage * 1000; // Convert voltage to millivolts

    mavlink_msg_sys_status_pack(
        1, // system ID
        1, // component ID
        &msg,
        0, // onboard control sensors present
        0, // onboard control sensors enabled
        0, // onboard control sensors health
        voltage_mv, // battery voltage in millivoltsD
        -1, // current (not being sent)
        -1, // battery remaining (not being sent)
        0, 0, 0, 0, 0, 0, 0, 0, 0 // other system status parameters
    );

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial.write(buffer, len);
}

void send_gps_status(int32_t lat, int32_t lon, int32_t alt, uint16_t hdop) {
    mavlink_message_t msg;

    mavlink_msg_gps_raw_int_pack(
        1, // system ID
        1, // component ID
        &msg,
        millis(), // time since boot
        3, // fix type (3D fix)
        lat, // latitude in 1E7 degrees
        lon, // longitude in 1E7 degrees
        alt, // altitude in mm
        65535, // GPS HDOP
        65535, // VDOP
        65535, // ground speed in cm/s
        65535, // course over ground in centidegrees
        hdop,  // Horizontal dilution of precision
        10     // number of satellites visible
    );

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial.write(buffer, len);
}

/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                            SETUP
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
void setup() {
  // MAVLink serial port
  Serial.begin(9600); // default baudrate for QGC

  // Enable LED
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  const uint32_t current_time = millis();
  
  float voltage = 3.7; // Replace with your voltage reading
  int32_t lat = 47356600; // Replace with your latitude in 1E7 degrees
  int32_t lon = 85445600; // Replace with your longitude in 1E7 degrees
  int32_t alt = 150000;   // Replace with your altitude in mm
  uint16_t hdop = 100;    // Replace with your HDOP

  // Send heartbeat via serial port
  if (current_time - lastTime_heartbeat > heartbeat_interval){
    send_heartbeat();
    lastTime_heartbeat=current_time;
  }

  // Check reception buffer
  decode_messages();
}