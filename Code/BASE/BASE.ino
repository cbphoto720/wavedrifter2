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
  - [ ] relay broadcasts into external software for GPS mapping

  Hardware Connections:
  - Sparkfun GPS board Ublox NEO-M8P-2 [TAOGLAS AGGP.25F.07.0060A antenna] (I2C) (Optional)
  - Sparkfun RFM69HCW Wireless Transceiver - 915MHz (SPI)
*/

#include <RFM69.h>
#include <RFM69_ATC.h>
#include <SPI.h>

#define NETWORKID     10   // Must be the same for all nodes (0 to 255)
#define MYNODEID      0   // My node ID (0 to 255)
#define TONODEID      1   // Destination node ID (0 to 254, 255 = broadcast)
#define MY_RF69_IRQ_PIN 8
#define MY_RF69_SPI_CS 21

// RFM69 frequency, uncomment the frequency of your module:

//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ

// AES encryption (or not):

#define ENCRYPT       true // Set to "true" to use encryption
#define ENCRYPTKEY    "FILEDCREW1234567" // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):

#define USEACK        true // Request ACKs or not

// Use Automatic Transmisionn Control (ATC) to transmit power at close range

#define ENABLE_ATC  //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI -80

// Packet sent/received indicator LED (optional):

// #define LED           9 // LED positive pin
// #define GND           8 // LED ground pin

// Create a library object for our RFM69HCW module:

#ifdef ENABLE_ATC
RFM69_ATC radio(MY_RF69_SPI_CS,MY_RF69_IRQ_PIN);
#else
RFM69 radio(MY_RF69_SPI_CS,MY_RF69_IRQ_PIN);
#endif

void setup()
{
  // Open a serial port so we can send keystrokes to the module:
  
  Serial.begin(9600);
  Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready");  

  // Set up the indicator LED (optional):
  
  // pinMode(LED,OUTPUT);
  // digitalWrite(LED,LOW);
  // pinMode(GND,OUTPUT);
  // digitalWrite(GND,LOW);
    
  // Initialize the RFM69HCW:
  //  radio.setCS(10);  //uncomment if using Pro Micro
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

    // If the input is a carriage return, or the buffer is full:
    
    if ((input == '\r') || (sendlength == 61)) // CR or buffer full
    {
      // Send the packet!


      Serial.print("sending to node ");
      Serial.print(TONODEID, DEC);
      Serial.print(": [");
      for (byte i = 0; i < sendlength; i++)
        Serial.print(sendbuffer[i]);
      Serial.println("]");
      
      // There are two ways to send packets. If you want
      // acknowledgements, use sendWithRetry():
      
      if (USEACK)
      {
        if (radio.sendWithRetry(TONODEID, sendbuffer, sendlength, 10))
          Serial.println("ACK received!");
        else
          Serial.println("no ACK received :(");
      }

      // If you don't need acknowledgements, just use send():
      
      else // don't use ACK
      {
        radio.send(TONODEID, sendbuffer, sendlength);
      }
      
      sendlength = 0; // reset the packet
      // Blink(LED,10);
    }
  }

  // RECEIVING

  // In this section, we'll check with the RFM69HCW to see
  // if it has received any packets:

  if (radio.receiveDone()) // Got one!
  {
    // Print out the information:
    
    Serial.print("(Drifter ");
    Serial.print(radio.SENDERID, DEC);
    Serial.print("): [");

    // The actual message is contained in the DATA array,
    // and is DATALEN bytes in size:
    
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);

    // RSSI is the "Receive Signal Strength Indicator",
    // smaller numbers mean higher power.
    
    Serial.print("], RSSI ");
    Serial.println(radio.RSSI);

    // Send an ACK if requested.
    // (You don't need this code if you're not using ACKs.)
    
    if (radio.ACKRequested())
    {
      radio.sendACK();
      Serial.println("ACK sent");
    }
    // Blink(LED,10);
  }
}

// void Blink(byte PIN, int DELAY_MS)
// // Blink an LED for a given number of ms
// {
//   digitalWrite(PIN,HIGH);
//   delay(DELAY_MS);
//   digitalWrite(PIN,LOW);
// }
