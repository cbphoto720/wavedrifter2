/*
  Drifter 2: A water-following, Inertial-Measuring, Sattelite-tracking, Position-Transceiving, wave drifter
  By: Carson Black
  Scripps Institute of Oceanography
  Date: April 19, 2024

  This Code is for the BASE station.  Each Drifter is configured to broadcast coordinates to node 0 on the network, the base node!
  This base station reads in telemetry from the broadcasting drifters and displays it on a field laptop.

  To-Do:
  - [x] read in broadcast strings from a drifter
  - [ ] handle broadcasts from multiple drifers (broadcast a timing command that sets each clock for broadcasting)
    - give a "token" to each drifter that gives it permission to broadcast
  - [X] relay broadcasts into external software (Mission Planner) for GPS mapping
  - [ ] [RFM] Send drifter commands
  - [ ] [RFM] Handle multiple drifter transmissions (timing)
  - [ ] interrupt based incoming transmission handling
  - [ ] Parse incoming RFM message to populate drifter

  Hardware Connections:
  - Sparkfun GPS board Ublox NEO-M8P-2 [TAOGLAS AGGP.25F.07.0060A antenna] (I2C) (Optional)
  - Sparkfun RFM69HCW Wireless Transceiver - 915MHz (SPI)
*/

#include <RFM69.h>
#include <RFM69_ATC.h>
#include <SPI.h>
#include "c_library_v2/common/mavlink.h"

struct DrifterData{ // Data to capture from each Drifter RFM signal
  uint8_t drifterID;
  int32_t lat;
  int32_t lon;
  float voltage;
  char Program_Status; // 'STR' Startup, 'LOG' (recording data), 'REC' (recovery), 'OFF' (shutdown)
  elapsedMillis sinceRFM; //track: millis() time since last RFM signal (or use UTC time since last signal)
};

#define MAX_NUM_DRIFTERS 12
DrifterData drifters[MAX_NUM_DRIFTERS];
uint8_t drifterIDi=0; // current IDX to send mssgs
uint8_t totalnumdrifters=0; // the true length of drifterIDs

// Timing Variables
const unsigned int heartbeat_interval= 1000;
uint32_t lastTime_heartbeat = 0; //FLAG should this be an unsigned long?


/* RFM */
#define NETWORKID     10   // Must be the same for all nodes (0 to 255)
#define MYNODEID      0   // My node ID (0 to 255)
#define TONODEID      1   // Destination node ID (0 to 254, 255 = broadcast)
#define MY_RF69_IRQ_PIN 8
#define MY_RF69_SPI_CS 21

#define FREQUENCY     RF69_915MHZ
// AES encryption
#define ENCRYPT       true // Set to "true" to use encryption
#define ENCRYPTKEY    "FILEDCREW1234567" // Use the same 16-byte key on all nodes
#define USEACK        true // Request ACKs or not
#define ENABLE_ATC  //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI -80 // Target RSSI for automatic transmission power

// Create a library object for our RFM69HCW module:
#ifdef ENABLE_ATC
RFM69_ATC radio(MY_RF69_SPI_CS,MY_RF69_IRQ_PIN);
#else
RFM69 radio(MY_RF69_SPI_CS,MY_RF69_IRQ_PIN);
#endif


/* MAVlink */
// MavLink Custom parameters
const char* param_names[] = {"RECOVERY_BAT_VOLTAGE_TRIGGER", "SHUTDOWN_BAT_VOLTAGE_TRIGGER"};
#define PARAM_COUNT (sizeof(param_names) / sizeof(param_names[0])) // Define PARAM_COUNT as the number of elements in the array
float param_values[PARAM_COUNT] = {3.78, 3.7};

/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                Helper Functions
 * ----------------- ------------------ ------------------ ------------------ ------------------*/

bool parseGPSData(const char* gpsData, int &drifterID, char* utcTime, float &lat, float &lon, float &voltage) {
  // Assuming GPS data is received as a comma-separated string "DrifterID#,UTCtime,Lat,Lon,Voltage"
  int parsed = sscanf(gpsData, "%d,%19[^,],%f,%f,%f", &drifterID, utcTime, &lat, &lon, &voltage);
  return (parsed == 5);
}

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

void initRFM(){
  Serial.println("Initializing RFM radio");
  int c1 = 0;
  while (!radio.initialize(FREQUENCY, MYNODEID, NETWORKID)) {
    Serial.print(".");
    delay(1);
    if(c1==100){ // try to connect for 100ms
      Serial.print("\r");
      Serial.println("ERROR: [RFM] radio init failed.");
      while (1); // Halt
    }
    c1++;
  }
  radio.setHighPower(); // Always use this for RFM69HCW
  // Turn on encryption if desired:
  if (ENCRYPT)
    radio.encrypt(ENCRYPTKEY);
  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif

  Serial.println("SUCCESS: RFM69 radio init OK!");
}

void sendcommand(){
  break; //FLAG [WIP] send drifter a command (like setting program status or flashing a light)
}

/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                MavLink Functions
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
void decode_messages() {
  static mavlink_message_t message;
  static mavlink_status_t status;

  // Read all available data from the serial port and parse the MAVLink message
  while(Serial.available() > 0) {
    uint8_t serial_byte = Serial.read();

    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, serial_byte, &message, &status)){
      // Handle the MAVLink message
      switch(message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          // Blink when a HEARTBEAT is received
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
          break;

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
          send_all_parameters();
          break;

        case MAVLINK_MSG_ID_COMMAND_LONG:
          handle_command_long(message);
          break;

        default:
          break;
      }
    }
  }

  delay(1); // Quick delay to avoid overwheliming requests
}

void handle_command_long(const mavlink_message_t& message) {
  mavlink_command_long_t cmd;
  mavlink_msg_command_long_decode(&message, &cmd);

  if (cmd.command == MAV_CMD_DO_SET_HOME) { // Check if it's the set-home command
    if (cmd.param1 == 1) { // param1 == 1 means set to specific coordinates
      float home_lat = cmd.param5; // Latitude
      float home_lon = cmd.param6; // Longitude
      float home_alt = cmd.param7; // Altitude

      // Set the home position
      set_home_position(home_lat, home_lon, home_alt);

      // Optionally, send acknowledgment
      send_command_ack(message.sysid, message.compid, MAV_CMD_DO_SET_HOME, MAV_RESULT_ACCEPTED);
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
void setup()
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT); // Enable LED

  initRFM(); // Initialize the RFM69HCW:

  Serial.println("SUCCESS: Base station initialized. Connect to MissionPlanner.");
}

/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                            LOOP
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
void loop()
{
  // Set up a "buffer" for characters that we'll send:
  
  static char sendbuffer[62];
  static int sendlength = 0;

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
  decode_messages(); //FLAG this function "bangs" on the serial door quite often.  

  // SENDING (all serial input is written to buffer.  CR or >61 char will send the data)
  if (Serial.available() > 0){
    char input = Serial.read();
    if (input != '\r'){ // not a carriage return
      sendbuffer[sendlength] = input;
      sendlength++;
    }    
    if ((input == '\r') || (sendlength == 61)){ // CR or buffer full
      // Send the packet!
      Serial.print("sending to node ");
      Serial.print(TONODEID, DEC);
      Serial.print(": [");
      for (byte i = 0; i < sendlength; i++)
        Serial.print(sendbuffer[i]);
      Serial.println("]");
      
      if (USEACK){
        if (radio.sendWithRetry(TONODEID, sendbuffer, sendlength, 10))
          Serial.println("ACK received!");
        else
          Serial.println("no ACK received :(");
      } else { // don't use ACK
        radio.send(TONODEID, sendbuffer, sendlength);
      }
      sendlength = 0; // reset the packet
    }
  }

  // RECEIVING
  if (radio.receiveDone()) // Got one!
  {    
    Serial.print("(Drifter ");
    Serial.print(radio.SENDERID, DEC);
    Serial.print("): [");
    
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
          
    Serial.print("], RSSI ");
    Serial.println(radio.RSSI);
    
    if (radio.ACKRequested())
    {
      radio.sendACK();
      Serial.println("ACK sent");
    }
  }
  
}