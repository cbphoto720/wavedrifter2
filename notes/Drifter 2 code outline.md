-reporting requirements for drifter cost
- send falk bullet points itemizing the upgrades to the 2.0 drifter version vs the 1st
- magnet on/off switch
- photo of new drifter 

### SPI
- microsd
- Transceiver
### I2C
- imu
- GPS
##### Key
//WIP - Things to fix!
//DEBUG - useful for problem solving
//BLINK - good spots to communicate to user what is happening
//OFF - places where the program should shut down (may be able to reset with Flip-Flop board)
//ERROR - places where an error should be thrown
//SPEED


# WIP
- [x] use struct NMEA_GNGGA to parse data ([follow this tutorial](https://www.youtube.com/watch?v=ylxwOg2pXrc&t=0s)) and then broadcast valid fix lat/lon to base
- [ ] find a way to interally store RAWX in M8P LOG file before writing to SD card
- [ ] broadcast from 915 mHz

**SD_ReadWrite_example3**
- Working prototype of reading/writing data
- Definite connection issue on the breadboard creating weird null characters, even in the SD card FILENAME (a const char).  I am hoping these missing characters are just a connection problem on the breadboard.
**Drifter2.ino**
- determine when to blink LED and what color for indication (look for //BLINKS in code)
- [x] Write code to create the filename for each drifter (here is a good forum post that has a [file naming sprinrf](https://forum.arduino.cc/t/create-variable-text-file-name-to-be-saved-on-sd-card/1158544/3))
- [ ] change gatherIMUdata (160) to ONLY collect data
	- write function to print IMU data once stored in a buffer of sufficient size

**NEO M8P TX-ready indication**
- super useful for interrupt based programming! [Ublox M8P RecieverDescrProtocol](https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf)
	UBX - CFG - PRT (configure PIO port to Tx ready with kb threshold set)
	UBX - MON - HW (monitor Hardware to see PIO assignments)

___
# Functions
- Parse GNGGA -> see readme in drifter2

# VOID SETUP
**microSD card**
- [x] begin serial
- [x] checkIMU
- [x] init SD card /currently ugly
- [x] calibrateIMU
- [x] initialize files
- [x] make filename "drifterXX_log000X.txt"
- [x] choose max file size (Battery life should automatically limit these files to an acceptable amount)
- [x] init GPS
- [x] initialize GPS settings (set up from default (NMEA messages, RAWX, TX-ready interrupt, Nav Freq 4Hz, Full power mode)
	https://www.youtube.com/watch?v=TwhCX0c8Xe0&t=1025s
	https://www.youtube.com/watch?v=ylxwOg2pXrc&t=20s
	https://www.youtube.com/watch?v=n8PUyOtiGKo
	one of these videos shows how to send UBX messages over I2C or at least something similar.
	- [ ] Log UBX-RAWX data ([data structure](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf?utm_content=UBX-22008968) page 189 in the zed F9 manual)
- [ ] check the 915 mHz
- [ ] check comms with base station
- [ ] buzz to indicate data logging
- [ ] send mssg to base station when logging

# VOID LOOP
- [check voltage every](https://www.youtube.com/watch?v=gw72g4WBz-U) so often, activate a deep sleep state and the buzzer at some critical voltage level
- [ ] use microNMEA to read lat/long/time from drifter and send comm mssgs back on the RFM69
- [ ] attach interrupt for GPS to say it has new data
	Interrupt could simply set a flag to process the GPS data in the main loop
- [ ] attach interrupt for RFM when receiving data from BASE
- [ ] periodically (40ms) log IMU data
- [ ] periodically send 915 mHz mssg

___
# Base station code outline
- [ ] include all libraries

**Initialize**
- [ ] Drifter ID=0
- [ ] broadcast power

###### Specialize commands
- [ ] Calibrate IMU while in housing
- [ ] Activate sleep state (Disable GPS (may not be able to do that because GPS=UTC time clock, how will it know when to send messages?  Maybe it just won't be talking.  See how much the RFM69 draws when it is just listening for signals))
- [ ] Request data download

# Base station GUI
- [ ] KML position map of all drifters
- [ ] Drifter status list
	- [ ] battery %
	- [ ] Time since last communication
	- [ ] Time since last GPS fix
	- [ ] Time spent powered on
- [ ] send commands
	- [ ] [[#Specialize commands]]


- [ ] read serial interface mssgs from base
# Questions
- What information is required for RAW GNSS messages in order to do PPK?

# Datalog
```
\\\\\\\\\\\\\\\\\\\\ NEW START \\\\\\\\\\\\\\\\\\\\ 
\\Drifter_Number(DRIFTERNUM)_(FILENUM).txt
\\Bat Volt: (VOLTAGE) V, 
\\IMU_update: (IMUupdate) ms, GPS_update: (GPSupdate) ms, Transciever_update: (TRANSupdate) ms
\\ Accel_range: (ACCELrange)G, Gyro_range: (GYROrange) dps
\\ IMUtime, Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz, P, T
\\ NEMA MSSG: $GPGGA,UTCtime,Latitude,N,Longitude,W,GPS_Quality,#SVs,HDOP,MSL_height,M(meters),Geoid-seperation,M(meters),Age_of_differentialGPS,Ref-StationID,*Checksum
// RAWX MSSG:
$RAWX GNSS_ID,SV_ID:,Psuedorange(meters),CarrierPhase(cycles)

```



# Examples
**SparkFun_u-blox_GNSS_Arduino_Library\examples\ZED-F9P\Example23_getRXMRAWX**
- get RAWX GPS messages Psuedorange and Carrier Phase info
GNSS ID: 0     SV ID: XX     PR: XXXXXXXX.XXX m     CP: XXXXXXXXX.XXX cycles