/*
  Drifter 2: A water-following, Inertial-Measuring, Sattelite-tracking, Position-Transceiving, wave drifter
  By: Carson Black
  Scripps Institute of Oceanography
  Date: April 19, 2024

  Hardware Connections:
  - Sparkfun GPS board Ublox NEO-M8P-2 [TAOGLAS AGGP.25F.07.0060A antenna] (I2C)
  - Pimoroni ICM20948 9DoF Motion Sensor Breakout (I2C)
  - Sparkfun RFM69HCW Wireless Transceiver - 915MHz (SPI)
  - Pololu MicroSD breakout board (SPI)
*/
#define DRIFTER2_VERSION "1.11"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <elapsedMillis.h> //Flag remove extraneous libraries.  just do the math it is probably just as fast

bool debug = true; //DEBUG: TRUE enables Serial messages and extra data.
bool silent_start = false; // Disable buzzer on startup
#define PIMARONI // Define IMU
#define SparkfunGPS // Define GPS
#define SparkfunRFM // Define RFM
#define PoluluSD // Define MicroSD

/* drifter identification number */
const int drifterNumber = 1; // My node ID (1 to 254) (0 reserved for BASE STATION) (255 reserved for broadcast to ALL)
int filenumber=1;
char IMUfilename[33];
char GPSfilename[33];

/*global variables*/
float VOLTAGE = 0; // Battery voltage (V)
char DRIFTER_STATUS[4] = "STR";
unsigned long currentTime = 0;
const unsigned int IMU_UPDATE = 40; //ms
unsigned long lastTime_IMU = 0;
elapsedMillis sinceIMU;
const unsigned int GPS_UPDATE = 250; //ms
unsigned long lastTime_GPS = 0;
elapsedMillis sinceGPS;
const unsigned int RFM_UPDATE= 8000; //ms
unsigned long lastTime_RFM = 0;
const unsigned int SYSTEM_UPDATE = 1000; //ms
unsigned long lastTime_System = 0;

float lastlatitude = 0;
float lastlongitude = 0;
uint8_t LastUTC[4]; //FLAG.  handle UTC time offset for comms with multiple drifters
elapsedMillis sinceUTC;

const int VOLTpin = 16; // Voltage
const int LED_pin = 22;  // Indicator LED
const int recoveryLED_pin = 6;  // Recovery LED
const int BUZZ_pin = 1; // Buzzer
// FLAG [Drifter V2: const int BUZZ_pin = 15;]


/* IMU Variables */
#ifdef PIMARONI
  IntervalTimer IMUtimer;
  #include <ICM20948_WE.h>
  #define ICM20948_ADDR 0x68
  ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
  int IMU_Write_Counter = 0;
  char accelRange[4]; //include space for null term
  char gyroRange[9];
  float xMagOffset;
  float yMagOffset;
  float zMagOffset;
  float xMagGain;
  float yMagGain;
  float zMagGain;
#endif

/* GPS variables */
#ifdef SparkfunGPS
  #include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
  #include <MicroNMEA.h>
  SFE_UBLOX_GNSS myGNSS;
  unsigned long lastGNSSsend = 0;
  #define UbloxM8P_ADDR 0x42  // Define the I2C address of the Ublox M8P chip
  char NMEA_GPSbuffer[100]; // NMEA with buffer space (u-blox_structs.h) 100 bytes is plenty 
  MicroNMEA nmea(NMEA_GPSbuffer, sizeof(NMEA_GPSbuffer)); 
  #define RAWX_GPSbuffer 3000 // Total packet size for RAWX (UBX_RXM_RAWX_MAX_LEN = 16 + (32 * 92) = 2960)
  uint16_t GPSfilebuffer = 16000; // Internal file buffer of the M8P (store messages before writing to SD card)
  uint8_t *GPSbuffer;
  const int GPS_intteruptpin = 9; // Interrupt pin ON THE TEENSY (attach ISR to this pin HIGH -> LOW)
#endif

/* RFM variables */
#ifdef SparkfunRFM
  #include <RFM69.h>
  #include <RFM69_ATC.h>
  char sendbuffer[60]; // Buffer for RFM
  const size_t sendbuffer_size = sizeof(sendbuffer); // max transmission size per packet
  int sendlength = 0; // = sizeof(sendbuffer)

  #define NETWORKID     10   // Must be the same for all nodes (0 to 255)
  #define MYNODEID      drifterNumber   // My node ID (0 to 255)
  #define BASEID      0   // Destination node ID (0 to 254, 255 = broadcast)
  #define MY_RF69_IRQ_PIN 8
  #define MY_RF69_SPI_CS 21
  #define FREQUENCY     RF69_915MHZ
  #define ENCRYPTKEY    "FILEDCREW1234567" // Use the same 16-byte key on all nodes
  // optional configs
  #define USEACK        true // Request ACKs or not
  #define ENABLE_ATC  //comment out this line to disable AUTO TRANSMISSION CONTROL
  #define ATC_RSSI -80
  struct RFMbuf{

  };

  #ifdef ENABLE_ATC
  RFM69_ATC radio(MY_RF69_SPI_CS,MY_RF69_IRQ_PIN);
  #else
  RFM69 radio(MY_RF69_SPI_CS,MY_RF69_IRQ_PIN);
  #endif
#endif

/* SD variables */
File myfile;
const int SDpinCS = 5; // Pin 10 default on Teensy
char buf[100] = {'\0'}; //DEBUG - sprinf needs a buffer char to store data!  figure out how big you want this to be
// IMU buffer
const int IMUbufferSize = 50; // Adjust based on how many readings you want to buffer
// float IMUbuffer[IMUbufferSize][9]; // 9 float values per entry
// unsigned long IMUtimebuffer[IMUbufferSize];  // buffer for timestamps
char imuBuffer[IMUbufferSize][100]; // Assuming 92 chars per line
int bufferIndex = 0;

/* Debugging variables */
int time_in=0;
int time_out=0;

/* Comman Variables */
#define MAX_COMMAND_LENGTH 20
char commandBuffer[MAX_COMMAND_LENGTH];
int commandIndex=0;

/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                IMU CODE Pimoroni
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
#ifdef PIMARONI
 /*IMU initializtion functions*/
//this function checks that the IMU is working
void initIMU(){
  // Wire.beginTransmission(ICM20948_ADDR);
  // int error = Wire.endTransmission();
  // if(error == 0){
  //   Serial.println("ICM detected on bus");
  // }
  
  int linecount=0;
  Serial.println("waiting for ICM");
  while(!myIMU.init()){
    Serial.print(".");
    delay(50);
    linecount++;
    if(linecount==250){
      Serial.println("waiting for ICM");
      linecount=0;
    }
  }
  Serial.println("IMU initialized");

  //test Magnometer exists and works
  if(!myIMU.initMagnetometer()){
    Serial.println("ERROR: Magnetometer failed");
    while(1){}
  }
  else{
    Serial.println("Magnetometer initialized");
  }

  memset(imuBuffer, '\0', sizeof(imuBuffer)); // Set the imubuffer to null
}

void accelRangeSet(int range){
  /*choosing the argument of this functions sets the accel range and writes to the header*/
  if (range == 2){
    myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
    sprintf(accelRange, "2G");
  }
  else if (range == 4){
    myIMU.setAccRange(ICM20948_ACC_RANGE_4G);
    sprintf(accelRange, "4G");
  }
  else if (range == 8){
    myIMU.setAccRange(ICM20948_ACC_RANGE_8G);
    sprintf(accelRange, "8G");
  }
  else if (range == 16){
    myIMU.setAccRange(ICM20948_ACC_RANGE_16G); 
    sprintf(accelRange, "16G");
  }
  else{
    Serial.print ("accel range error");
    while(1){}  
  }
}

void gyroRangeSet(int range){
  /*choosing the argument of this functions sets the gyro range and writes to the header*/
  if (range == 250){
    myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
    sprintf(gyroRange, "250 dps");
  }
  else if (range == 500){
    myIMU.setGyrRange(ICM20948_GYRO_RANGE_500);
    sprintf(gyroRange, "500 dps");
  }
  else if (range == 1000){
    myIMU.setGyrRange(ICM20948_GYRO_RANGE_1000);
    sprintf(gyroRange, "1000 dps");
  }
  else if (range == 2000){
    myIMU.setGyrRange(ICM20948_GYRO_RANGE_2000);
    sprintf(gyroRange, "2000 dps");
  } 
  else{
    Serial.print("gyro range error");
    while(1){}   
  }
}

//this function sets the parameters for IMU calibration and data sampling
void calibrateIMU(){
  myIMU.setAccOffsets(-16384.0, 16384.0, -16384.0, 16384.0, -16384.0, 16384.0);
  myIMU.setGyrOffsets(-29.8, 79.2, -20.0);
  xMagOffset=0;
  yMagOffset=0;
  zMagOffset=0;
  xMagGain=1.0;
  yMagGain=1.0;
  zMagGain=1.0;
  accelRangeSet(16);
  myIMU.setAccDLPF(ICM20948_DLPF_5);    
  gyroRangeSet(2000);
  myIMU.setGyrDLPF(ICM20948_DLPF_5);  
  myIMU.setTempDLPF(ICM20948_DLPF_4);
  myIMU.setMagOpMode(AK09916_CONT_MODE_20HZ);
}

// /*this function retrieves data collected by the IMU every few 
//   milliseconds and prints them to the SD card.  It is inconsistent and inefficient*/
// void gatherIMUdata(unsigned long currentTime){
//   int timer = millis(); //DEBUG
//   int timediff = 0; //DEBUG

//   myfile=SD.open(IMUfilename,FILE_WRITE);
//   myIMU.readSensor();
//   xyzFloat gValue = myIMU.getAccRawValues();
//   xyzFloat gyr = myIMU.getGyrValues();
//   xyzFloat magValue = myIMU.getMagValues();
//   float magX = xMagGain*(magValue.x-xMagOffset);
//   float magY = yMagGain*(magValue.y-yMagOffset);
//   float magZ = zMagGain*(magValue.z-zMagOffset);
//   //display current time and IMU results on the IMU file
//   memset(buf, '\0', sizeof(buf));; //clear the buffer
//   sprintf(buf,"%lu, %.1f, %.1f, %.1f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f",currentTime,gValue.x,gValue.y,gValue.z,gyr.x,gyr.y,gyr.z,magX,magY,magZ);
//   myfile.println(buf);
//   // IMU_Write_Counter++;
//   // if (IMU_Write_Counter > 200) {
//   //    myfile.flush();
//   //    IMU_Write_Counter = 0;
//   // }
//   timediff = millis() - timer;  //DEBUG
//   myfile.print("logtimer: "); //DEBUG
//   myfile.println(timediff); //DEBUG
//   myfile.flush();
//   myfile.close();
// }

void bufferIMUData(unsigned long currentTime) {
  myIMU.readSensor();
  xyzFloat gValue = myIMU.getAccRawValues();
  xyzFloat gyr = myIMU.getGyrValues();
  xyzFloat magValue = myIMU.getMagValues();
  float magX = xMagGain * (magValue.x - xMagOffset);
  float magY = yMagGain * (magValue.y - yMagOffset);
  float magZ = zMagGain * (magValue.z - zMagOffset);

  // Store in buffer
  snprintf(imuBuffer[bufferIndex], 100, "%lu, %.1f, %.1f, %.1f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f",
          currentTime, gValue.x, gValue.y, gValue.z, gyr.x, gyr.y, gyr.z, magX, magY, magZ);
  imuBuffer[bufferIndex][97] = '\0';  // New line //FLAG is this new line termination correct? will it ever write over data?
  imuBuffer[bufferIndex][98] = '\r';  // New line
  imuBuffer[bufferIndex][99] = '\n';  // Null terminate
  bufferIndex++;

  if (bufferIndex >= IMUbufferSize) {
    if(debug){
      time_in=millis();
    }
    writeIMUDataToSD();
     if(debug){
      time_out=millis()-time_in;
    }
  }
}

void writeIMUDataToSD() {
    myfile = SD.open(IMUfilename, FILE_WRITE);
  
    myfile.write((uint8_t*)imuBuffer, bufferIndex * sizeof(imuBuffer[0]));
    if (debug) {
        char timeStr[20];  // Buffer to hold the formatted time string
        sprintf(timeStr, "$%d\r\n", time_out);  // Format the time with a $ prefix and newline
        myfile.write(timeStr, strlen(timeStr));  // Write the time string to the SD card
        memset(timeStr, ' ',sizeof(timeStr)); //reset timeStr
    }

    myfile.flush();
    myfile.close();
    memset(imuBuffer, '\0', sizeof(imuBuffer)); //Wipe the buffer with ' ' char
    bufferIndex = 0;
}

// void writeIMUDataToSD() {
//     myfile = SD.open(IMUfilename, FILE_WRITE);
//     // Calculate total length of valid data in the buffer
//     int totalLength = 0;
//     for (int i = 0; i < bufferIndex; i++) {
//         totalLength += strlen(imuBuffer[i]);
//     }
//     // Write the entire valid data in one go
//     myfile.write((uint8_t*)imuBuffer, totalLength);

//     myfile.flush();
//     myfile.close();
    
//     // Reset the buffer with spaces or null characters if needed
//     memset(imuBuffer, ' ', sizeof(imuBuffer)); // Initialize with spaces to avoid unwanted characters
//     bufferIndex = 0; // Reset buffer index
// }


void flushRemainingIMUData() {
    if (bufferIndex > 0) {
        writeIMUDataToSD();
    }
}

#endif

/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                GPS CODE Sparkfun/Ublox
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
#ifdef SparkfunGPS
void initGPS(){
  // Configure Teensy pin as an interrupt
  pinMode(GPS_intteruptpin, INPUT);
  attachInterrupt(digitalPinToInterrupt(GPS_intteruptpin), txReadyISR, FALLING);

  if (!myGNSS.setPacketCfgPayloadSize(sizeof(NMEA_GPSbuffer) + RAWX_GPSbuffer)){
    Serial.println(F("ERROR: [GPS] setPacketCfgPayloadSize failed. You will not be able to poll RAWX data. Freezing."));
    while(1){}; // Do nothing more
  }
  myGNSS.setFileBufferSize(GPSfilebuffer);
  if (myGNSS.getFileBufferSize() != GPSfilebuffer){
    Serial.println(F("ERROR: [GPS] setFileBufferSize failed. Freezing."));
    while(1){}; // Do nothing more
  }

  // Turn the GPS on
  // Serial.println("Ready to test");
  // if (myGNSS.isConnected()) {
  //     Serial.println("GNSS module is recognized");
  //     // Now proceed to initialize
  // }
  // else{
  //   Serial.println("GNSS module is not recognized");
  // }

  for (int address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    int error = Wire.endTransmission();
    if (error == 0) {
      Serial.println(F("Device found"));
    }
    else{

    }
  }

  int linecount=0;
  Serial.println("waiting for GPS");
  while(!myGNSS.begin()){
    Serial.print(".");
    delay(250);
    linecount++;
    if(linecount==50){
      Serial.println("waiting for GPS");
      linecount=0;
    }
  }
  Serial.println("GPS initialized");

  // Configure the GPS settings
  SendGPSconfiguration(); // Use Binary to send a bunch of configurations to the M8P
  myGNSS.disableUBX7Fcheck(); // RAWX data can legitimately contain 0x7F, so we need to disable the "7F" check in checkUbloxI2C
  myGNSS.setNMEAGNGGAcallbackPtr(&printGNGGA); // Function to run when NMEA GNGGA data is received
  myGNSS.setAutoRXMRAWXcallback(&printRAWX);

  //Save settings to flash and BBR on the M8P module
  if(!myGNSS.saveConfiguration()){
    Serial.println("ERROR: [GPS] failed to save configuration");
    while(1){};
  }
  else{
    Serial.println("SUCCESS: [GPS] saved configuration");
  }
}

void printRAWX(UBX_RXM_RAWX_data_t rawxData){
  if(debug){ // print RAWX recieved
    Serial.print(F("\r\nRAWX: Length: "));
    Serial.print(rawxData.header.numMeas); 
  }

  // Write data to SD card
  #ifdef PoluluSD 
    int timer = millis();
    int timediff = 0;
    myfile = SD.open(GPSfilename, FILE_WRITE);  
    static char buffer[128];  // Allocate buffer.  See u-blox_structs.h for byte size info
    const size_t buffer_size = sizeof(buffer);
    // const uint16_t UBX_RXM_RAWX_MAX_LEN = 16 + (32 * 92); 92+16=118

    // Format the header data into the buffer
    double rcvTowValue;  // Convert rcvTow to double
    memcpy(&rcvTowValue, rawxData.header.rcvTow, sizeof(rcvTowValue));
    int len = snprintf(buffer, buffer_size, "%.6f,%d,%d,%d\n", 
                      rcvTowValue, 
                      rawxData.header.week, 
                      rawxData.header.leapS, 
                      rawxData.header.numMeas);
    // Write the header to the file
    if (len > 0 && len < buffer_size) {
        myfile.write(buffer, len);  // Only write the actual content, not the full buffer size
    } else { // Buffer size is too small or another error occured
      if(debug){
        Serial.print('ERROR: [GPS] cannot compose rawx file write. len = ');
        Serial.println(len);
      }
    }

    // Write block information from each Satallite
    for (int i = 0; i < rawxData.header.numMeas; i++) {
      double prMesValue, cpMesValue, doMesValue; // decode RAWX bytes into double variable
      memcpy(&prMesValue, rawxData.blocks[i].prMes, sizeof(prMesValue));
      memcpy(&cpMesValue, rawxData.blocks[i].cpMes, sizeof(cpMesValue));
      memcpy(&doMesValue, rawxData.blocks[i].doMes, sizeof(doMesValue));
      // Format the data into the buffer
      int len = snprintf(buffer, buffer_size,
                        "%d,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                        i,
                        prMesValue,
                        cpMesValue,
                        doMesValue,
                        rawxData.blocks[i].gnssId,
                        rawxData.blocks[i].svId,
                        rawxData.blocks[i].sigId,
                        rawxData.blocks[i].freqId,
                        rawxData.blocks[i].lockTime,
                        rawxData.blocks[i].cno,
                        rawxData.blocks[i].prStdev,
                        rawxData.blocks[i].cpStdev,
                        rawxData.blocks[i].doStdev,
                        rawxData.blocks[i].trkStat.bits.prValid,
                        rawxData.blocks[i].trkStat.bits.cpValid,
                        rawxData.blocks[i].trkStat.bits.halfCyc,
                        rawxData.blocks[i].trkStat.bits.subHalfCyc
      );
      // Write the buffer to the file
      if (len > 0 && len < buffer_size) {
        myfile.write(buffer, len);
      }
      else{ // Buffer size is too small or another error occured
        if(debug){
          Serial.print('ERROR: [GPS] cannot compose rawx file write. len = ');
          Serial.println(len);
        }
      }
    }
    timediff = millis() - timer;  //DEBUG
    myfile.print("logtimer: "); //DEBUG
    myfile.println(timediff); //DEBUG
    myfile.flush();
    myfile.close();
    #endif
}

void printGNGGA(NMEA_GGA_data_t *nmeaData){
  //FLAG [SPEED] print NMEA to SD every 250ms.  You might need to cache a few as buffer to save power/compute time
  #ifdef PoluluSD //check that SD card is enabled
    myfile = SD.open(GPSfilename, FILE_WRITE);
    if(myfile){
      myfile.println((const char *)nmeaData->nmea);
    }
    else{
      Serial.print("ERROR: unable to open file: ");
      Serial.println(GPSfilename);
      // while(1); //ERROR - Freeze and stop program from continuing
    }
    myfile.close();
  #endif
  
  // Print out full NMEA on serial
  if(debug){
    Serial.print(F("\r\nGNGGA: Length: "));
    Serial.print(nmeaData->length);
    Serial.print(F("\tData: "));
    Serial.print((const char *)nmeaData->nmea); // .nmea is printable (NULL-terminated) and already has \r\n on the end
  }

  // use microNMEA to parse NMEA string in order to pull out Lat/Lon/ UTC time
  bool process= false;
  for (int i = 0; nmeaData->nmea[i] != '\n'; i++) {
    process=nmea.process((char)nmeaData->nmea[i]);
  }
  if (process) {
    Serial.print("Valid fix: ");
    Serial.println(nmea.isValid() ? "yes" : "no");
    if(nmea.isValid()){
      lastLatitude = nmea.getLatitude();
      lastLongitude = nmea.getLongitude();
      LastUTC[0] = nmea.getHour(); //GGA hhmmss.sss
      LastUTC[1] = nmea.getMinute();
      LastUTC[2] = nmea.getSecond();
      LastUTC[3] = nmea.getHundredths();

      sinceUTC = 0; // update last time since valid fix.

      if(debug){
        Serial.print("Latitude (deg): ");
        Serial.println(lastLatitude, 6);
        Serial.print("Longitude (deg): ");
        Serial.println(lastLongitude, 6);
        Serial.print("UTC Time: ");
        Serial.println(lastUTC);
        
      }
    }
  }
}

/// ISR for reading the data off the Sparkfun board
void txReadyISR() {
  //FLAG 
  if(debug){Serial.println("txReady ISR!");}
}

void SendGPSconfiguration() {
  // Through expirimentation, ublox M8P only listens to about 100 bytes per transmission
  uint8_t UBXdataToSend1[] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1B, 0x9A, // Revert to default configuration (ensure we don't have any extra messages)
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x00, 0x00, 0xB7, 0x14, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6F, 0xE4, // Configure PRT: I2C UBX+NMEA and enable Tx_ready with 41byte (328 bit) low-active threshold (empty NMEA-GNGGA = 42 byte)
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x68, //Hotstart, useful for clearing logs
  };
  uint8_t UBXdataToSend2[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38, // Disable NMEA-GxGSV on all channels
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A, // Disable NMEA-GxGLL on all channels 
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, // Disable NMEA-GxRMC on all channels
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46,  // Disable NMEA-GxVTG on all channels
  };
  uint8_t UBXdataToSend3[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31, // Disable NMEA-GxGSA on all channels
  };
  uint8_t UBXdataToSend4[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x4C, // Enable RXM-RAWX on I2C only
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, // Enable NMEA-GxGGA on I2C only
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96, // Set Nav rate to 4Hz (250 ms Measurement Period)
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB // SAVE CFG
  };
    //FLAG: is 41 bytes the right threshold to use?  Empty GGA strings are 42 bytes.  You could increase this threshold to a certain threshold for writing a bunch of microSD data at once (is this data lost in the event of shutdown?  Look into M8P LOG files)
  GPSbinaryWrite(UBXdataToSend1, sizeof(UBXdataToSend1));
  delay(25);
  GPSbinaryWrite(UBXdataToSend2, sizeof(UBXdataToSend2));
  delay(25);
  GPSbinaryWrite(UBXdataToSend3, sizeof(UBXdataToSend3));
  delay(25);
  GPSbinaryWrite(UBXdataToSend4, sizeof(UBXdataToSend4));
  delay(25);
}

void GPSbinaryWrite(uint8_t* datatosend, size_t datalen){
  Wire.beginTransmission(UbloxM8P_ADDR); // Start transmission to the Ublox I2C address 
  for (size_t i = 0; i < datalen; i++) {
    Wire.write(datatosend[i]); // Write each byte of data
  }
  byte error = Wire.endTransmission(); // End transmission

  // if(debug){Serial.print("[GPS] size of UBX packet: ");} //DEBUG
  // if(debug){Serial.println(datalen);} //DEBUG

  if (error == 0) {
    Serial.println("GPS config packet sent successfully");
  } else {
    Serial.print("Error sending GPS config: ");
    Serial.println(error);
  }
}
#endif
/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                RFM CODE Sparkfun
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
 #ifdef SparkfunRFM
void initRFM(){
  Serial.println("Initializing RFM radio");
  int linecount=0;
  while(!radio.initialize(FREQUENCY,MYNODEID,NETWORKID)){
    Serial.print(".");
    delay(250);
    linecount++;
    if(linecount==50){
      Serial.println("waiting for RFM module");
      linecount=0;
    }
  }
  radio.setHighPower(); //always set high power for HCW varient
  radio.encrypt(ENCRYPTKEY);

  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif 
  
  //FLAG this is a simple test startup.  Include more startup info like bat voltage, device mode, etc
  sprintf(sendbuffer,"D%01d:SU,,,,,%.2fV", drifterNumber, VOLTAGE); // sendlength 22 long
  if (USEACK){
    if (radio.sendWithRetry(BASEID, sendbuffer, strlen(sendbuffer), 10))
      Serial.println("Coms established with base");
    else
      Serial.println("ERROR: unable to establish coms to Base");
  }
  else{
    // If you don't need acknowledgements, just use send():
    radio.send(BASEID, sendbuffer, sizeof(sendbuffer));
  }
}
#endif

/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                MicroSD CODE Polulu
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
#ifdef PoluluSD
void initmicroSD(){
  // // SD Card Initialization
  pinMode(SDpinCS, OUTPUT); 
  Serial.print("Initializing SD card: ");
  int linecount=0;
  while(!SD.begin(SDpinCS)){
    Serial.print(".");
    delay(250);
    linecount++;
    if(linecount==50){
      Serial.println("waiting for microSD");
      linecount=0;
    }
  }
  if(!SD.begin(SDpinCS)){
    Serial.println("ERROR: microSD failed");
    while(1);
  } else{
    Serial.println("microSD initialized");
  }
}

void initfiles(){
  // Create a unique IMU and GPS log and write a header to the file.
  // Determine next filename
  Serial.println("Initializing datalog files");
  snprintf(IMUfilename, sizeof(IMUfilename), "Drifter%03d_IMUlog%03d.txt" , drifterNumber, filenumber); // Pair all sensor logs to IMU file numbering system
  Serial.print("initial filename: "); //DEBUG
  Serial.println(IMUfilename); //DEBUG
  while(SD.exists(IMUfilename)){
    filenumber++;
    snprintf(IMUfilename, sizeof(IMUfilename), "Drifter%03d_IMUlog%03d.txt" , drifterNumber, filenumber);
    Serial.println(IMUfilename); //DEBUG
  }
  Serial.print("     FINAL filename: "); //DEBUG
  Serial.println(IMUfilename); //DEBUG
  snprintf(IMUfilename, sizeof(IMUfilename), "Drifter%03d_IMUlog%03d.txt" , drifterNumber, filenumber); //Use new log# if others exist
  snprintf(GPSfilename, sizeof(GPSfilename), "Drifter%03d_GPSlog%03d.txt" , drifterNumber, filenumber); //Pair GPS file to IMUfile log#

  myfile = SD.open(IMUfilename, FILE_WRITE); // create the file
  myfile.close();
  myfile = SD.open(GPSfilename, FILE_WRITE); // create the file
  myfile.close();
  
  // Write header to IMUfile
  #ifdef PIMARONI
  myfile = SD.open(IMUfilename, FILE_WRITE);
  if(myfile){
    // IMU header
    myfile.println("//--------- NEW START ---------//");
    sprintf(buf,"// Drifter number: %01d, Battery: %.2f", drifterNumber, VOLTAGE);
    myfile.println(buf);
    sprintf(buf,"// IMU_update: %d ms, GPS_update: %d ms, Transciever_update: %d ms",IMU_UPDATE,GPS_UPDATE,RFM_UPDATE);
    myfile.println(buf);
    sprintf(buf,"// Accel_range: %s, Gyro_range: %s", accelRange, gyroRange);
    myfile.println(buf);
    myfile.println("IMUtime, Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz, P, T");
    if(debug){Serial.println("- wrote IMU file header");}
  }
  else{
    Serial.print("ERROR: unable to open file: ");
    Serial.println(IMUfilename);
    while(1); //ERROR - Freeze and stop program from continuing
  }
  myfile.close();
  #endif

  // Write header to GPSfile
  #ifdef SparkfunGPS
  myfile = SD.open(GPSfilename, FILE_WRITE);
  if (myfile)
  {
    // GPS header
    myfile.println("//--------- NEW START ---------//");
    sprintf(buf,"//Drifter number: %01d, Battery: %.2f", drifterNumber, VOLTAGE);
    myfile.println(buf);
    myfile.println("//$GPGGA,UTCtime,Latitude,N,Longitude,W,GPS_Quality,#SVs,HDOP,MSL_height,M(meters),Geoid-seperation,M(meters),Age_of_differentialGPS,Ref-StationID,*Checksum");  
    myfile.println("//rcvTow,week,leapS,numMeas,datablock#,prMes,cpMes,doMes,gnssId,svId,sigId,freqId,lockTime,cno,prStdev,cpStdev,doStdev,prValid,cpValid,halfCyc,subHalfCyc");
    //SPEED (do We really need to write this whole thing to the SD Card?  (header is fine but full GGA strings every 250ms + raw might be too much))
    // - using the INT pin for GPS could limit SD write until GPS data are available
    if(debug){Serial.println("- wrote GPS file header");}
  }
  else{
    Serial.print("ERROR: unable to open file: ");
    Serial.println(GPSfilename);
    while(1); //ERROR - Freeze and stop program from continuing
  }
  myfile.close();

  Serial.println("");
  Serial.println("SD card is ready to log data.");
  #endif
}
#endif

/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                      General Functions
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
float readBatteryVoltage(int numReadings) {
  int analogValue=0;
  if(numReadings==1){ // For speed
    analogValue=analogRead(VOLTpin); // Read the analog value (0 to 1023)
  }
  else{ // For accuracy
    long total = 0;
    for (int i = 0; i < numReadings; i++) {
      total += analogRead(VOLTpin);
      delay(5); // Small delay between readings to reduce correlation
    }
    analogValue = total/numReadings;  // avg the analog values
  }
  float voltageOut = (analogValue / 1023.0) * 3.3;   // Convert analog to voltage (assuming a 3.3V reference)
  float batteryVoltage = voltageOut * 1.303;  // Scale up to the original voltage using the scaling factor
  return batteryVoltage;
}


/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                            SETUP
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
void setup() {
  // Set all ellapsedMillis to allocate
  sinceIMU=0; 
  sinceGPS=0; 
  sinceUTC = 0;

  if(!silent_start){
    tone(BUZZ_pin, 800);
    delay(200);
    noTone(BUZZ_pin);
  }
  analogWrite(recoveryLED_pin, 10); //FLAG Do not set higher than 10 unless in low power mode
  delay(10);
  analogWrite(recoveryLED_pin, 0);

  if(false){ //DEBUG - change argument to debug for deployment release.  Otherwise loops waiting for serial
    Serial.begin(115200);
    while(!Serial){
      delay(20);
    }  
  }
  #if defined(SparkfunGPS) || defined(PIMARONI)
    pinMode(18, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);
    Wire.begin();
    Wire.setClock(400000);  //FLAG: I2C fast mode 400000 or normal 100000
  #endif

  Serial.println(" ");
  Serial.println("~ ~ ~ ~ ~ REBOOT ~ ~ ~ ~ ~");
  delay(1000);

/*
  - check bat voltage
  - init IMU, GPS, RFM, etc
*/
  pinMode(BUZZ_pin, OUTPUT);
  pinMode(LED_pin, OUTPUT);
  pinMode(recoveryLED_pin, OUTPUT);
  pinMode(VOLTpin, INPUT);
  noTone(BUZZ_pin);
  digitalWrite(LED_pin, LOW);
  analogWrite(recoveryLED_pin, 0);
  VOLTAGE= readBatteryVoltage(5);
  #ifdef PIMARONI 
    initIMU();
    calibrateIMU();
  #endif
  #ifdef SparkfunGPS
    initGPS();
  #endif
  #ifdef SparkfunRFM
    initRFM();
  #endif
  #ifdef PoluluSD
    initmicroSD();
    initfiles(); // Create drifter new file on startup with config info and data headers
  #endif

  //FLAG testing faster clock
  // SPI.begin;
  // SD.setClockDivider(SPI_CLOCK_DIV2); // Sets SPI clock (Teensy 4.0 default speed is 96 MHz / 4)

  strcpy(DRIFTER_STATUS, "OFF");

  // Startup sound (data is now logging)
  digitalWrite(LED_pin, HIGH);
  analogWrite(recoveryLED_pin, 10);
  if(!silent_start){
    tone(BUZZ_pin, 800);
    delay(75);
    noTone(BUZZ_pin);
    delay(25);
    tone(BUZZ_pin, 1200);
    analogWrite(recoveryLED_pin, 0);
    delay(75);
    noTone(BUZZ_pin);
    delay(25);
    tone(BUZZ_pin, 1800);
    analogWrite(recoveryLED_pin, 10);
    delay(75);
    noTone(BUZZ_pin);
    delay(25);
    tone(BUZZ_pin, 2700);
    analogWrite(recoveryLED_pin, 0);
    delay(75);
    noTone(BUZZ_pin);
    delay(250);
    noTone(BUZZ_pin);
    delay(75);
    analogWrite(recoveryLED_pin, 10);
    tone(BUZZ_pin, 6400);
    delay(350);
    noTone(BUZZ_pin);
  }
  digitalWrite(LED_pin, LOW);
  analogWrite(recoveryLED_pin, 0);

  // if(debug){
  //   sinceGPS=-125; //reset GPS timer

  // }

  if(debug){delay(5000);}
  sinceIMU=0; //reset IMU timer
  sinceGPS=0; //reset GPS timer //FLAG set GPS timer to 50% GPS_UPDATE to offset SD write cycle?
}

/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                            LOOP
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();
  if((currentTime - lastTime_System)>=SYSTEM_UPDATE){
    VOLTAGE= readBatteryVoltage(1);
    if(debug){
      Serial.print("Voltage: ");
      Serial.println(VOLTAGE);
    }
    lastTime_System=currentTime;
  }

  #ifdef SparkfunGPS
    if ((sinceGPS)>=GPS_UPDATE) {
      myGNSS.checkUblox(); // Check for the arrival of new data and process it.
      myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.
      // lastTime_GPS = currentTime;
      sinceGPS= sinceGPS - GPS_UPDATE;
    }
  #endif

  #ifdef PIMARONI
    if ((sinceIMU)>=IMU_UPDATE) {
      // gatherIMUdata(currentTime);
      bufferIMUData(currentTime);
      sinceIMU = sinceIMU - IMU_UPDATE;
      // Serial.print("IN IMU UPDATE time: ");   Serial.println(currentTime);
    }
  #endif

    // Send an RFM message to Base
  #ifdef SparkfunRFM
    if((currentTime - lastTime_RFM)>=RFM_UPDATE){
      //FLAG: configure 'sendbuffer' and 'sendlength'
      int len = snprintf(sendbuffer,sendbuffer_size,"%s,%f,%f,%lu,%.2fV", DRIFTER_STATUS, lastlongitude, lastlatitude, (unsigned long)sinceUTC , VOLTAGE); // ~ 32 bytes 
      //FLAG: fix lastlon and lastlat data type representation in snprintf
      if (USEACK){
        if (radio.sendWithRetry(BASEID, sendbuffer, strlen(sendbuffer), 10))
          Serial.println("SUCCESS: message sent to base");
        else
          Serial.println("ERROR: unable to reach Base");
      }
      else{
        // If you don't need acknowledgements, just use send():
        radio.send(BASEID, sendbuffer, strlen(sendbuffer));
      }
      lastTime_RFM = currentTime;
    }
  #endif

// FLAG work in progress -send drifter commands through serial terminal
  if (Serial.available() > 0) {
    char input = Serial.read();
    // Check if the input is a carriage return (end of the command)
    if (input == '\r') {
      commandBuffer[commandIndex] = '\0';  // Null-terminate the command string
      // Compare the command with known commands
      if (strcmp(commandBuffer, "shutdown") == 0) {
        if (debug) { Serial.println("Shutting down..."); }
        #ifdef PoluluSD
          flushRemainingIMUData();
        #endif
        #ifdef SparkfunGPS
        uint8_t UBXdataToSend[] = {
          0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x08, 0x00, 0x16, 0x74 //Stop GNSS with Hotstart option
        };
        GPSbinaryWrite(UBXdataToSend, sizeof(UBXdataToSend));
        #endif

        //FLAG do other shutdown steps (low power GPS)
        if (debug) { Serial.println("SHUTDOWN"); }
          strcpy(DRIFTER_STATUS, "OFF");
          while(1); // is this the best way to handle shutdown?  what about setting update rates to maxval?
      }
      else if (strcmp(commandBuffer, "recovery") == 0) {
        // Handle recovery command
      }
      else if (strcmp(commandBuffer, "normal") == 0) {
        // Handle normal mode command
        // set IMU_UPDATE back to normal 
      }
      else {
        Serial.println("Unknown command: ignore input");
      }
        // Reset the command buffer and index
        commandIndex = 0;
        memset(commandBuffer, '\0', sizeof(commandBuffer));
    } 
    // Otherwise, store the character in the command buffer
    else if (commandIndex < MAX_COMMAND_LENGTH - 1) {
      commandBuffer[commandIndex++] = input;
    }
  }
}