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

#include <SPI.h>
#include <SD.h>
#include <Wire.h>

bool debug = true; //DEBUG: TRUE enables Serial messages and extra data.
// #define PIMARONI // Define IMU
#define SparkfunGPS // Define GPS
//#define SparkfunRFM // Define RFM
// #define PoluluSD // Define MicroSD

/* drifter identification number */
const int drifterNumber = 1; // My node ID (1 to 254) (0 reserved for BASE STATION) (255 reserved for broadcast RFM)
int filenumber=1;
char IMUfilename[33];
char GPSfilename[33];

/*global variables*/
float VOLTAGE = 0; // Battery voltage (V)
unsigned long currentTime = 0;
const unsigned int IMU_UPDATE = 40; //ms
unsigned long lastTime_IMU = 0;
const unsigned int GPS_UPDATE = 250; //ms
unsigned long lastTime_GPS = 0;
const unsigned int RFM_UPDATE= 8000; //ms
unsigned long lastTime_RFM = 0;


// SD variables
File myfile;
const int SDpinCS = 5; // Pin 10 default on Teensy
char buf[100] = {'\0'}; //DEBUG - sprinf needs a buffer char to store data!  figure out how big you want this to be

/* IMU Variables */
#ifdef PIMARONI
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
  char sendbuffer[60]; // 62 bytes is max transmission size
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

/* Led setup */
const int LED_pin = 22;
// bool switchState = true;
const int recoveryLED_pin = 6;

/* Buzz setup */
const int BUZZ_pin = 1; //Drifter V2 = 15

/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                IMU CODE Pimoroni
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
#ifdef PIMARONI
 /*IMU initializtion functions*/
//this function checks that the IMU is working
void initIMU(){
  int linecount=0;
  Serial.println("waiting for ICM");
  while(!myIMU.init()){
    Serial.print(".");
    delay(250);
    linecount++;
    if(linecount==50){
      Serial.println("waiting for ICM");
      linecount=0;
    }
  }
  if(!myIMU.init()){
    Serial.println("ERROR: ICM failed");
    while(1);
  }
  else{
    Serial.println("ICM initialized");
  }
  //test Magnometer exists and works
  if(!myIMU.initMagnetometer()){
    Serial.println("ERROR: Magnetometer failed");
    while(1){}
  }
  else{
    Serial.println("Magnetometer initialized");
  }
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

/*this function retrieves data collected by the IMU every few 
  milliseconds and prints them to the SD card*/
void gatherIMUdata(unsigned long currentTime){
  myfile=SD.open(IMUfilename,FILE_WRITE);
  myIMU.readSensor();
  xyzFloat gValue = myIMU.getAccRawValues();
  xyzFloat gyr = myIMU.getGyrValues();
  xyzFloat magValue = myIMU.getMagValues();
  float magX = xMagGain*(magValue.x-xMagOffset);
  float magY = yMagGain*(magValue.y-yMagOffset);
  float magZ = zMagGain*(magValue.z-zMagOffset);
  //display current time and IMU results on the IMU file
  myfile.println(sprintf(buf,"%lu, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f,",currentTime,gValue.x,gValue.y,gValue.z,gyr.x,gyr.y,gyr.z,magX,magY,magZ));
  // IMU_Write_Counter++;
  // if (IMU_Write_Counter > 200) {
  //    myfile.flush();
  //    IMU_Write_Counter = 0;
  // }
  myfile.close();
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
  if(!myGNSS.begin()){
    Serial.println("ERROR: GPS failed");
    while(1);
  } else{
    Serial.println("GPS initialized");
  }

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
  Serial.print(F("\r\nRAWX: Length: "));
  Serial.print(rawxData.header.numMeas); 

  if (rawxData.header.numMeas > 0) {
    double prMesValue;
    memcpy(&prMesValue, rawxData.blocks[0].prMes, sizeof(prMesValue));
    Serial.print(F("\r\nBlock 0 prMes: "));
    Serial.println(prMesValue, 6); // Print with 6 decimal places
  }
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
  if(process){
    Serial.print("Valid fix: ");
    Serial.println(nmea.isValid() ? "yes" : "no");

    float latitude_mdeg = nmea.getLatitude();
    float longitude_mdeg = nmea.getLongitude();
    Serial.print("Latitude (deg): ");
    Serial.println(latitude_mdeg / 1000000., 6);

    Serial.print("Longitude (deg): ");
    Serial.println(longitude_mdeg / 1000000., 6);
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

  if(debug){Serial.print("[GPS] size of UBX packet: ");}
  if(debug){Serial.println(datalen);}

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
  if(!radio.initialize(FREQUENCY,MYNODEID,NETWORKID)){
    Serial.println("ERROR: [RFM] RFM initialize failed");
    while(1);
  } else{
    Serial.println("RFM initialized");
  }
  radio.setHighPower(); //always set high power for HCW varient
  radio.encrypt(ENCRYPTKEY);

  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif 
  
  //FLAG this is a simple test startup.  Include more startup info like bat voltage, device mode, etc
  sprintf(sendbuffer,"$SIO Drifter%01d reboot", drifterNumber); // sendlength 22 long
  if (USEACK){
    if (radio.sendWithRetry(BASEID, sendbuffer, sizeof(sendbuffer), 10))
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
  #ifdef Pimoroni
  myfile = SD.open(IMUfilename, FILE_WRITE);
  if(myfile){
    // IMU header
    myfile.println("//////////// NEW START ////////////");
    sprintf(buf,"// Drifter number: %01d, Battery: %.2f", drifterNumber, VOLTAGE);
    myfile.println(buf);
    sprintf(buf,"// IMU_update: %d ms, GPS_update: %d ms, Transciever_update: %d ms",IMU_UPDATE,GPS_UPDATE,RFM_UPDATE);
    myfile.println(buf);
    sprintf(buf,"// Accel_range: %s, Gyro_range: %s", accelRange, gyroRange);
    myfile.println(buf);
    myfile.println("IMUtime, Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz, P, T");
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
    myfile.println("//////////// NEW START ////////////");
    sprintf(buf,"//Drifter number: %01d, Battery: %.2f", drifterNumber, VOLTAGE);
    myfile.println(buf);
    myfile.println("//$GPGGA,UTCtime,Latitude,N,Longitude,W,GPS_Quality,#SVs,HDOP,MSL_height,M(meters),Geoid-seperation,M(meters),Age_of_differentialGPS,Ref-StationID,*Checksum");  
    //SPEED (do We really need to write this whole thing to the SD Card?  (header is fine but full GGA strings every 250ms + raw might be too much))
    // - using the INT pin for GPS could limit SD write until GPS data are available
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
 *                                            SETUP
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
void setup() {
  Serial.begin(115200);
  while(!Serial){
    delay(20);
  }  
  #ifdef SparkfunGPS
    Wire.begin(UbloxM8P_ADDR); //FLAG [GPS] testing
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

  // Startup sound (data is now logging)
  digitalWrite(LED_pin, HIGH);
  analogWrite(recoveryLED_pin, 10);
  tone(BUZZ_pin, 800);
  delay(75);
  noTone(BUZZ_pin);
  delay(50);
  tone(BUZZ_pin, 1200);
  delay(75);
  noTone(BUZZ_pin);
  delay(50);
  tone(BUZZ_pin, 1800);
  delay(75);
  noTone(BUZZ_pin);
  delay(50);
  tone(BUZZ_pin, 2700);
  delay(75);
  noTone(BUZZ_pin);
  delay(50);
  noTone(BUZZ_pin);
  delay(150);
  noTone(BUZZ_pin);
  delay(75);
  tone(BUZZ_pin, 6400);
  delay(350);
  noTone(BUZZ_pin);
  digitalWrite(LED_pin, LOW);
  analogWrite(recoveryLED_pin, 0);
}

/*------------------ ------------------ ------------------ ------------------ ------------------
 *                                            LOOP
 * ----------------- ------------------ ------------------ ------------------ ------------------*/
void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();

  #ifdef SparkfunGPS
    if ((currentTime - lastTime_GPS)>=GPS_UPDATE) {
      myGNSS.checkUblox(); // Check for the arrival of new data and process it.
      myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.
      lastTime_GPS = currentTime;
    }
  #endif

    // Log IMU data
    #ifdef PIMARONI
      if ((currentTime - lastTime_IMU)>=IMU_UPDATE) {
        gatherIMUdata(currentTime);
        lastTime_IMU = currentTime;
        // Serial.print("IN IMU UPDATE time: ");   Serial.println(currentTime);
      }
    #endif

    // Send an RFM message to Base
  #ifdef SparkfunRFM
    if((currentTime - lastTime_RFM)>=RFM_UPDATE){
      //FLAG: configure 'sendbuffer' and 'sendlength'
      sprintf(sendbuffer,"Drifter clock: %lu",currentTime);
      if (USEACK){
        if (radio.sendWithRetry(BASEID, sendbuffer, sizeof(sendbuffer), 10)){
          if(debug){Serial.println("Success!  ACK received");}
        }
        else{
          if(debug){Serial.println("ERROR: [RFM] no ACK received");}
        }
      }
      else{
        radio.send(BASEID, sendbuffer, sizeof(sendbuffer));
      }
      lastTime_RFM = currentTime;
    }
  #endif
}