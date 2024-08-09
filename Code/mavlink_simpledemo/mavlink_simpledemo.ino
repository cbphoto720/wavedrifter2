// Include MAVLink header
// Remember that it's necessary to init and update git submodule
#include "c_library_v2/common/mavlink.h"

// Define your parameters
#define PARAM_COUNT 3
const char* param_names[PARAM_COUNT] = {"PARAM1", "PARAM2", "PARAM3"};
float param_values[PARAM_COUNT] = {1.0, 2.0, 3.0};

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
        voltage_mv, // battery voltage in millivolts
        -1, // current (not being sent)
        -1, // battery remaining (not being sent)
        0, 0, 0, 0, 0, 0, 0, 0, 0 // other system status parameters
    );

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial.write(buffer, len);
}

s

void setup() {
  // MAVLink serial port
  Serial.begin(9600); // default baudrate for QGC

  // Enable LED
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Send heartbeat via serial port
  send_heartbeat();

  // Check reception buffer
  decode_messages();

}

void send_heartbeat() {
  static uint32_t last_time = millis();
  const uint32_t current_time = millis();
  constexpr const uint32_t heartbeat_interval = 1000; // 1 second
  if (current_time - last_time > heartbeat_interval) {
    last_time = current_time; // Update for the next loop

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
}

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
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
          break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
          send_all_parameters();
          break;
        default:
            break;
      }
    }
  }
}