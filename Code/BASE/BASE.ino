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
  - [ ] relay broadcasts into external software (Mission Planner) for GPS mapping

  Hardware Connections:
  - Sparkfun GPS board Ublox NEO-M8P-2 [TAOGLAS AGGP.25F.07.0060A antenna] (I2C) (Optional)
  - Sparkfun RFM69HCW Wireless Transceiver - 915MHz (SPI)
*/

#include <RFM69.h>
#include <RFM69_ATC.h>
#include <SPI.h>
#include <mavlink.h>

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
#define ATC_RSSI -80

// Create a library object for our RFM69HCW module:
#ifdef ENABLE_ATC
RFM69_ATC radio(MY_RF69_SPI_CS,MY_RF69_IRQ_PIN);
#else
RFM69 radio(MY_RF69_SPI_CS,MY_RF69_IRQ_PIN);
#endif

/* MAVlink */
#define SYS_ID 1
#define COMP_ID 1

#define mavlinkSerial Serial

bool parseGPSData(const char* gpsData, int &drifterID, char* utcTime, float &lat, float &lon, float &voltage) {
  // Assuming GPS data is received as a comma-separated string "DrifterID#,UTCtime,Lat,Lon,Voltage"
  int parsed = sscanf(gpsData, "%d,%19[^,],%f,%f,%f", &drifterID, utcTime, &lat, &lon, &voltage);
  return (parsed == 5);
}

void setup()
{
  mavlinkSerial.begin(57600); // MAVLink baud rate
  Serial.begin(9600);
  Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready");  
   
  // Initialize the RFM69HCW:
  int c1 = 0;
  Serial.print("Initializing");
  while (!radio.initialize(FREQUENCY, MYNODEID, NETWORKID)) {
    Serial.print(".");
    delay(1);
    if(c1==100){
      Serial.print("\r");
      Serial.println("RFM69 radio init failed");
      while (1); // Halt
    }
    c1++;
  }
  Serial.println("RFM69 radio init OK!");
  radio.setHighPower(); // Always use this for RFM69HCW

  // Turn on encryption if desired:
  if (ENCRYPT)
    radio.encrypt(ENCRYPTKEY);
  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif

  Serial.println("Starting GPS to MAVLink bridge...");
}

void loop()
{
  // Set up a "buffer" for characters that we'll send:
  
  static char sendbuffer[62];
  static int sendlength = 0;

  // SENDING

  // In this section, we'll gather serial characters and
  // send them to the other node if we (1) get a carriage return,
  // or (2) the buffer is full (61 characters).
  
  // If there is any serial input, add it to the buffer:

  if (Serial.available() > 0)
  {
    char input = Serial.read();
    
    if (input != '\r') // not a carriage return
    {
      sendbuffer[sendlength] = input;
      sendlength++;
    }    
    if ((input == '\r') || (sendlength == 61)) // CR or buffer full
    {
      // Send the packet!
      Serial.print("sending to node ");
      Serial.print(TONODEID, DEC);
      Serial.print(": [");
      for (byte i = 0; i < sendlength; i++)
        Serial.print(sendbuffer[i]);
      Serial.println("]");
      
      if (USEACK)
      {
        if (radio.sendWithRetry(TONODEID, sendbuffer, sendlength, 10))
          Serial.println("ACK received!");
        else
          Serial.println("no ACK received :(");
      }      
      else // don't use ACK
      {
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
  
  int drifterID;
    char utcTime[20];
    float lat, lon, voltage;
    if (parseGPSData(gpsData, drifterID, utcTime, lat, lon, voltage)) {
      sendGPSData(lat, lon);
    }
}