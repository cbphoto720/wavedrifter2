***
- it seems GPS is ONLY used for time
- GPS read can easily cause a hang by waiting for lasteNEMAread command

***
- [ ] include all libraries

#### Initialize
- [ ] debug mode (serial output, etc)
- [ ] time=0
- [ ] Drifter ID = XX
- init pins
	- [ ] LED
	- [ ] MicroSD CS
	- [ ] GPS CS
	- [ ] IMU pins

- [ ] IMU interval
- [ ] GPS interval
- [ ] MicroSD interval
- [ ] Broadcast interval

**IMU calibration**
- magnetic, accel, gyro
- set accel ranges
- set gyro range

#### Functions
- calibrate IMU (manually sets the offsets from a static test)
	- was this only performed for 1 board than applied to every IMU??
	- accel range was set to 16G
- gatherIMUdata: read IMU sensor and .print 
	- file is only closed (file.flush) every 200 writes (40ms * 200 = 8s)
	- this is only assuming that the void loop doesn't experience any delays, it could be even longer
- checkSD: Check SD card is active
- createheader: Create file header for readability
- spacer: formats the correct number of " " characters based in number size
	- couldn't the file be seperated by character delimiters like tabs instead?

#### Void Setup
- run all setups and calibrations
- Turn LED on at the end of setup
	- if LED is off, Setup had failed

#### Void Loop
- very start of loop first thing
- currentTime= millis()
	- Wouldn't this just overwrite any GPS time
	- (yes it does, but the later code writes down the GPS corrected time if it is different than the lastGPStime).  Which means if more time has passed than the given GPS_UPDATE, the GPS function is ran: microcontroller time is printed and then the last NEMA message is printed as comparison.  LastGPStime is then updated to the current MICROCONTROLLER time.
	- It seems like the GPS time is not at ALL interacting with the internal clock and is only providing reference to UTC for post processing.