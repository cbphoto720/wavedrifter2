# Funding ends in **JUNE**
# WIP
- [x] solder recovery LED to test on breadboard
- [ ] recovery LED mounting position
- [ ] design new GPS antenna mount
- [ ] 3DP new internalmounts 5.1
- [ ] Manufacture drifter2.1
- [ ] bouyancy test



- [x]  cut the red wire inside one of your microUSB cables.  Use jumpers and power supply to spin up the box
- RFM69 should be power by teensy onboard 3V supply (Likely cleaner than the FlipFlop board)
- [x]  Internal mounts is just slightly too tall.  take off 0.5 or 1mm and include a cutout for the Teensy 4.0 backing SMD electronics.
- [ ] expiriment with a centering rectangular cutout for the M8P board
- [ ] add mounts for Status LED, Recovery LED, Buzzer
- [ ] [RAWX message structure](https://docs.ros.org/en/noetic/api/ublox_msgs/html/msg/RxmRAWX.html)
- [ ] u-center M8P config
	Only output NEMA strings and RAWX data
	[10Hz U-blox binary GPS data in 66 lines of code (arduino)](https://www.youtube.com/watch?v=TwhCX0c8Xe0&t=0s)
	save configuration so that it can be uploaded to each board.  (have arduino read a ublox startup file???  That would be the cleanest way to ensure the boards are config ready)
	then work on arduino parsing of the hex binary
- [ ] Interrupt based programming routine so that you know when the buffer is being used!  (dont want to have another message write over the buffer before you read it!)
	have a routine handle the interupt code before returning to what you are doing.
	- look up simple arduino examples of interupts
- [ ] [RFM69HCW datasheet](https://cdn.sparkfun.com/datasheets/Wireless/General/RFM69HCW-V1.1.pdf) Page 48 describes DIO mapping in order to select what type of interrupt you would like to know about the buffer.  This is useful for Interrupt routines to handle saving the data


- [ ] Write code to test RFM 
- [ ] test broadcast with different drifter SG & range
- [ ] Another test of speed by broadcasting IMU data live
- [x] Do not work on HousingMounts until you finalize what electronics you need.  (3d printed inside)
	top of battery can sit 16mm from the top of 2-140_UP internalring.
	O-ring crossed & stretched 10mm from top of arduino board
- [ ] Code that detects a large upsurge in waves (perhaps a good time to broadcast/look for GPS on the crest of a wave???)

- [x] UPDATE FALK
	- Housing final size
		- USE [F9P board](https://gnss.store/zed-f9p-gnss-modules/145-195-elt0128.html#/61-gnss_module-l1_l2_zed_f9p) to achieve a 2mm increase in size (smaller battery size too)
	- internal board mount
	- Transceivers are communicating
		- what if the board would receive GPS, Broadcast that is got a hit, then trigger a camera to take a picture as fast as possible since GPS aquisition
	*Questions
	- ask for 3dp company name
	- show code running

###### Housing
- [ ] PCBway 3d print: Somos Ledo resin + UTR-8100 (Translucent) (See if the spray varnish option changes the O-ring dimensions!)
- [x] Finish GPS mount with space for reed switch + LED + RFM board
- [x] place to mount flip flop board
- [x] place to mount buzzer somewhere
- [x] place to mount battery O ring hooks on M8P board
- [x] FLIP SCREWS they wont fit unless the nut is face up

- [x] build in tolerance for inner mating halves
- [x] build in tolerance for o-ring groove (for using a HSS lathe tool to make face grooves)??? **Decide if you are going to use the lathe**

- [x] how to connect lipos (and how to charge)
- [x] find a reed switch/ HAL sensor

- [x] mount board internals

###### Electronics
- [x] solder RFM69HCW boards to propotying pins
- [x] test transceiver boards (follow [this guide](https://learn.sparkfun.com/tutorials/rfm69hcw-hookup-guide?_ga=2.22057791.527000107.1709582640-989094217.1706894118&_gl=1*h3xy37*_ga*OTg5MDk0MjE3LjE3MDY4OTQxMTg.*_ga_T369JS7J9N*MTcwOTU5NzIyNy4yMy4xLjE3MDk1OTc3NDIuNTIuMC4w))
- [x] Work on HAL sensor board
- [x] Power solution for M8P + transmitting mssg (current draw getting close to 250 mA)
	- [x] power M8P through USB connector if needed (tap directly into battery through hall effect sensor switch board)
##### FIXME


# Ideas
**Transciever ideas**
- void setup:
	broadcast Drifter#, Voltage, IMUfilename, GPSfilename
- Recovery mode
	activate internal buzzer
	increase RFM broadcast Hz
	activate LED
- Wave-detect transmission
	look at IMU data to detect a wave, use wave as an opportunity to transmit/look for GPS from a taller height off the water
# Componets

# VHB tape
0.4" x 0.025" (10.16 x 0.635 mm)
## GPS
**glonass**
- 1598-1604 MHz
- GPS 1575 MHz L1

[SMA to U.FL connector](https://www.coaxrf.com/shop/4uflcables/smaconnector/gold-plated-sma-male-plug-to-ipx-u-fl-male-plug-center-rf-adapter-connector/) $5.49

[GNSS OEM F9P board ](https://gnss.store/zed-f9p-gnss-modules/145-195-elt0128.html#/61-gnss_module-l1_l2_zed_f9p)
**L1/L2 Antennas**
- [AHP2258.07.0060A](https://www.taoglas.com/product/active-gnss-l1-l2-band-antenna/) TAOGLAS (slightly too big with PCB) (70x70mm ground plane)
- [ 2JM3201C2F ](https://www.2j-antennas.com/antennas/single-internal-antennas/2jm3201c2f-high-precision-gnss-gps-beidou-galileo-glonass-sbas-rtk-l1-l2-active-stacked-patch-internal-antenna/621) 2J Antennas GREAT SIZE (slightly worse but tested W/O groundplane!)
- [ANT1825JB01BGNL2A](https://www.digikey.com/en/products/detail/pulse-electronics/ANT1825JB01BGNL2A/15290928) not a lot of datasheet (TOO BIG MOQ)
- [HP2258.A](https://www.digikey.com/en/products/detail/taoglas-limited/HP2258-A/21777748) Taoglas PASSIVE L1/L2
- [APRG2512F01](https://www.digikey.com/en/products/detail/abracon-llc/APRG2512F01-0100S/15912595) Good datasheet
### Transmitter
- [sparkfun dev board](https://www.sparkfun.com/products/15005)
###### Alternative hardware
- [Ublox NEO-M8P-0](https://www.digikey.com/en/products/detail/u-blox/NEO-M8P-0/6150644)
	- Smaller footprint than ZED-F9P
	- Low power (~35mA [ICC Acquisition. Average current from startup until the first fix])
	- 3V

- [mini usb dev board](https://gnss.store/neo-m8p-gnss-modules/90-elt0078.html)

### Antenna
- [TAOGLAS AGGP.25f](https://www.digikey.com/en/products/detail/taoglas-limited/AGGP-25F-07-0060A/3083243) dual 1.575/1.602GHZ PATCH with U.FL connector.  High beam width for more tilt tolerance

###### RF tranceiver
- [sparkfun transciever](https://www.sparkfun.com/products/13909) 915 or 434MHz comms (500m range in open air)  can remotely activate drifters, receive position, or use an [activate an LED](https://www.mouser.com/ProductDetail/Cree-LED/XPEWHT-L1-0000-00D01?qs=y%252BJdrdj3vZp8dXPKma%2Fv7Q%3D%3D&mgh=1&gclid=Cj0KCQjwsp6pBhCfARIsAD3GZubyWUVJ2V34ARcj87C8c-rvFd2lTmhpPyfEkLxeYFaurbkeSLINI4saAl-eEALw_wcB)  (time lapse of wave)
###### Combo antenna GSM / GPS
- internal circumference 188mm
- 1.2 to 1.6 GHz

- [Smallest combo ant](https://www.digikey.com/en/products/detail/te-connectivity-linx/ANT-LPC-FPC-100/10663172) 64mm long
- [ Molex](https://www.digikey.com/en/products/detail/molex/2133530100/11673870) flexible 119mm with 2 U.FL
- [Ceramic patch style](https://www.l-com.com/embedded-active-gps-gsm-3g-antenna-3dBi-5dBic-ufl-hg821-03pu-gps?gclid=Cj0KCQjwj5mpBhDJARIsAOVjBdpfcsfdirLczPW7nWBsZT66rSl9MC7bswMQGzcRA-H5rme-2XKaGBYaAuPtEALw_wcB) 42mm square ground plane
-[ TAOGLAS flex ant](https://www.digikey.com/en/products/detail/taoglas-limited/FXUB70-A-07-C-001/4965533) 150mm long
- [BIG external TAOGLAS](https://www.mouser.com/ProductDetail/Taoglas/MA104.C.AB.015?qs=Ajmft%252BTTukF16yQE2i0Gfw%3D%3D)
###### Alternative hardware
- [page 24](https://content.u-blox.com/sites/default/files/NEO-M8P_HardwareIntegrationManual_UBX-15028081.pdf) of ublox handbook reccomends GPS antenna
	- https://www.tallysman.com/product/tw3400-single-band-gnss-antenna/
	- https://www.digikey.com/en/products/detail/taoglas-limited/GP-1575-25-4-A-02/2332642 (i found this patch antenna through a 1.575GHz digikey filter)
	- https://www.digikey.com/en/products/detail/taoglas-limited/A-40-A-301111/4503583

**Patch antennas**
- [Taoglas](https://www.digikey.com/en/products/detail/taoglas-limited/AP-25F-07-0078A/2754153) 3" U.FL cable (SAW, LNA, LNA) (medium size)
- [Taoglas AP.10F.07.0039B](https://www.digikey.com/en/products/detail/taoglas-limited/AP-10F-07-0039B/3083256) 1" U.FL cable (SAW, LNA, LNA) (smallest package)
- [Taoglas AGGP.25F.07.0060A](https://www.digikey.com/en/products/detail/taoglas-limited/AGGP-25F-07-0060A/3083243) (13 grams, first one I ordered)
- [ a med sized offering AP.12F.07.0045A](https://www.digikey.com/en/products/detail/taoglas-limited/AP-12F-07-0045A/3877418) (wonky radiation pattern)


### Reed switch
- look for latching magnetic reed switch

- [potential option](https://www.mouser.com/ProductDetail/MEDER-electronic-Standex/KSK-1E66-1-BV14501?qs=0Ys4hG7ORMeIs5g5U1sRHg%3D%3D) (expensive $10)
- [potential ](https://www.digikey.com/en/products/detail/tdk-micronas-gmbh/HAL1502%2520UA/18151531?utm_adgroup=General&utm_source=google&utm_medium=cpc&utm_campaign=PMax%20Shopping_Product_Zombie%20SKUs&utm_term=&utm_content=General&gclid=CjwKCAjw6p-oBhAYEiwAgg2Pgo_VrlV3Lf4Kj5OMDCDW6jEguhsP8oEplCsBkeCoFAL67jCMFtUFtRoC08QQAvD_BwE)(HAL effect switch) HAL 1502
	- use with a relay ([maybe this relay](https://www.digikey.com/en/products/detail/cui-devices/SR5-5V-200-1C/16602072?utm_adgroup=&utm_source=google&utm_medium=cpc&utm_campaign=PMax%20Shopping_Product_Low%20ROAS%20Categories&utm_term=&utm_content=&utm_id=go_cmp-20243063506_adg-_ad-__dev-c_ext-_prd-16602072_sig-CjwKCAiA29auBhBxEiwAnKcSqqA3JBhmQzHEvP3XbStzRnMaG4i-ngLdPwxWR5hh68wfIDu3E7gJbRoCHyoQAvD_BwE&gad_source=1&gclid=CjwKCAiA29auBhBxEiwAnKcSqqA3JBhmQzHEvP3XbStzRnMaG4i-ngLdPwxWR5hh68wfIDu3E7gJbRoCHyoQAvD_BwE))
	- google "1a relay through hole 5v"
	- [low profile relay option?](https://www.digikey.com/en/products/detail/panasonic-electric-works/TQ2-L-3V/304092?utm_adgroup=&utm_source=google&utm_medium=cpc&utm_campaign=Pmax_Shopping_Boston%20Metro%20Category%20Awarness&utm_term=&utm_content=&utm_id=go_cmp-20837509568_adg-_ad-__dev-c_ext-_prd-_sig-Cj0KCQjwwYSwBhDcARIsAOyL0fiUQmbicv__H_5JpKJQAHd8regDIB73AawoDpxVZ5LbLQx_z_5NPuYaArsIEALw_wcB&gad_source=1&gclid=Cj0KCQjwwYSwBhDcARIsAOyL0fiUQmbicv__H_5JpKJQAHd8regDIB73AawoDpxVZ5LbLQx_z_5NPuYaArsIEALw_wcB)
	- [another option](https://www.digikey.com/en/products/detail/omron-electronics-inc-emc-div/G5V-1-T90%2520DC24/6650357?utm_adgroup=&utm_source=google&utm_medium=cpc&utm_campaign=PMax%20Shopping_Product_Low%20ROAS%20Categories&utm_term=&utm_content=&utm_id=go_cmp-20243063506_adg-_ad-__dev-c_ext-_prd-6650357_sig-Cj0KCQjwwYSwBhDcARIsAOyL0fiB_nFmz81REDRfJshzqF6cPRGxme7YECVVDc18KwlSxCpLoGWmS-MaAitpEALw_wcB&gad_source=1&gclid=Cj0KCQjwwYSwBhDcARIsAOyL0fiB_nFmz81REDRfJshzqF6cPRGxme7YECVVDc18KwlSxCpLoGWmS-MaAitpEALw_wcB)
	- [low power consumption option?](https://www.digikey.com/en/products/detail/cui-devices/SR5-3V-200-1C/16602081)
	- [Flip Flop switch instructions for MCU sleep](https://www.hackster.io/usini/easy-way-to-put-your-board-to-sleep-5184a2)
	- 
- [Sparkfun latching HAL sensor](https://www.sparkfun.com/products/9312)
- [Sparkfun Reed switch non-latching](https://www.sparkfun.com/products/8642)
- [latching reed switch](https://www.layouts4u.net/electrical-items/switches/latching-reed-switch#:~:text=is%20in%20stock%3F-,Latching%20Reed%20Switch,again%20to%20switch%20it%20off.) with magnet on the outside?

## Batteries
- 3.4 V cutoff voltage

**Old bat**
- [250mah lipo](https://www.amazon.com/EEMB-Battery-Rechargeable-Lithium-Connector/dp/B08FD3V6TF/ref=asc_df_B08FD3V6TF/?tag=hyprod-20&linkCode=df0&hvadid=475792708712&hvpos=&hvnetw=g&hvrand=4087725322206008454&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9031300&hvtargid=pla-1049797859410&th=1)

**New Bats**
- [820mah](https://www.amazon.com/EEMB-653042-Rechargeable-Connector-Certified/dp/B082152887/ref=sr_1_13_sspa?crid=H4ER7MUH2MUJ&keywords=500%2Bmah%2Bbattery&qid=1695338925&sprefix=500%2Bmah%2Bbattery%2Caps%2C137&sr=8-13-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9tdGY&th=1) 44x30.5x6.8mm
- [2000mah](https://www.amazon.com/EEMB-2000mAh-Battery-Rechargeable-Connector/dp/B08214DJLJ/ref=sr_1_12_sspa?crid=H4ER7MUH2MUJ&keywords=500%2Bmah%2Bbattery&qid=1695338925&sprefix=500%2Bmah%2Bbattery%2Caps%2C137&sr=8-12-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9tdGY&th=1) 56x34.5x10.6mm
- [1100mah](https://www.amazon.com/EEMB-1100mAh-Battery-Rechargeable-Connector/dp/B08FD39Y5R/ref=sr_1_1_sspa?crid=C6SPASPMG79M&keywords=1000mah%2Bbattery&qid=1695340141&sprefix=1000mah%2Bbattery%2Caps%2C130&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1) 51x34.5x6mm
- [tiny 500mah can fit up to 3](https://www.digikey.com/en/products/detail/tinycircuits/ASR00035/9808767)
- [Tiny 290mah (a little smaller than the 500)](https://www.digikey.com/en/products/detail/tinycircuits/ASR00007/7404517)
- alternative 500mAH supplier [bulk pricing](https://tinycircuits.com/products/lithium-ion-polymer-battery-3-7v-500mah)

[Website for all shapes of battery](https://www.lipolbattery.com/lithium%20polymer%20battery.html): LP883648 is 1600mAH and 48mm longest dimension

## Housing
- [BAL TEC](https://www.precisionballs.com/pictgal.php#hollowsteel) Potential manufacturing source?  Makes all kinds of spheres'
- google search terms "cnc machine sphere hollow"
- [MACRO RING cam filter](https://www.amazon.com/dp/B08XMLK592/ref=sspa_dk_detail_5?psc=1&pd_rd_i=B08XMLK592&pd_rd_w=dzDeW&content-id=amzn1.sym.f734d1a2-0bf9-4a26-ad34-2e1b969a5a75&pf_rd_p=f734d1a2-0bf9-4a26-ad34-2e1b969a5a75&pf_rd_r=C2MTKP5J451C7RXT4C2K&pd_rd_wg=n3r7H&pd_rd_r=d2f9c60b-74da-490c-838a-a5edf635e24d&s=photo&sp_csd=d2lkZ2V0TmFtZT1zcF9kZXRhaWw) Closure plate for interface?
## CLOSURE SYSTEM

Inspiration:

- https://www.indiamart.com/proddetail/stainless-steel-pipe-clamp-16592394673.html
- https://www.mscdirect.com/product/details/04700365
- THIS IS THE BEST ONE https://www.aliexpress.us/item/2255800979747014.html?gatewayAdapt=glo2usa4itemAdapt
- https://www.grainger.com/product/5DFH4?gucid=N:N:PS:Paid:GGL:CSM-2295:4P7A1P:20501231&gclid=CjwKCAjw9-6oBhBaEiwAHv1QvLACEpERJXDYCj8WeNHc0WpXMH83Urpzhi1nitsaOiHPDyyxZoS_ZBoCyYkQAvD_BwE&gclsrc=aw.ds
- https://www.amazon.com/Stainless-Strengthens-Circular-Fasteners-Clamps/dp/B0B35VBNF4
- binder clip?
- [Hose clamp style with cross dowel nuts](https://www.amazon.com/Stainless-Strengthens-Circular-Fasteners-Clamps/dp/B0B35VBNF4?th=1)
- [Stainless cross dowel nuts](https://www.aliexpress.us/item/3256804059479775.html?src=google&src=google&albch=shopping&acnt=708-803-3821&slnk=&plac=&mtctp=&albbt=Google_7_shopping&albagn=888888&isSmbAutoCall=false&needSmbHouyi=false&albcp=19108228023&albag=&trgt=&crea=en3256804059479775&netw=x&device=c&albpg=&albpd=en3256804059479775&gclid=Cj0KCQjwmvSoBhDOARIsAK6aV7hdAF9Ge9hwS7sbparpIIpSIh-8qD9Y9WU6TpKSP1bJI4YCTzxogLQaAkKWEALw_wcB&gclsrc=aw.ds&aff_fcid=5f3e3df63b95445da66c401064af7da5-1696460261608-08697-UneMJZVf&aff_fsk=UneMJZVf&aff_platform=aaf&sk=UneMJZVf&aff_trace_key=5f3e3df63b95445da66c401064af7da5-1696460261608-08697-UneMJZVf&terminal_id=478e3439f89f480783e166e36a736622&afSmartRedirect=y&gatewayAdapt=glo2usa)
- WALMART [pipe clamp](https://www.walmart.com/ip/Adjustable-pipe-clamp-hose-clamp-quick-release-round-duct-clamp-quick-pipe-clamp-barrel-clamp-ring-SS-galvanized-80-400mm-dia/1670025789)

**3d Printed inspiration**
- [Aquarium clamp](https://www.reef2reef.com/threads/uv-pipe-clamps.633090/)
- [gopro pipe clamp](https://www.instructables.com/GoPro-Hinged-Pipe-Clamp/)
- [SLS print service](https://www.artisanmodelmakers.co.uk/blog/portfolio-items/3d-printed-automotive-parts/)


## O-RINGS
Housing2_rev2 uses a 2-224 o-ring

- -036 rings are too small diameter (similar to what Falk used)
- 2-142 ID close to 60mm  https://www.mcmaster.com/9452K148/
- should increase o ring size for better sealing
- these 60mm metric rings are a better because they are 50A shore hardness https://www.mcmaster.com/2418T185/

**50a Durometer**
- Grainger [2-146 50A o ring](https://www.grainger.com/product/O-Ring-146-712U13)
- Mcmaster [2-231](https://www.mcmaster.com/2418T186/) or [2-230](https://www.mcmaster.com/2418T185/)
  Mcmaster [2-146](https://www.mcmaster.com/2418T161/) Don't go this small unless you really need to



## LED
- [1.5 W COB LED](https://www.digikey.com/en/products/detail/creeled,-inc./JK3030AWT-P-H50EA0000-N0000001/9974567?utm_adgroup=&utm_source=google&utm_medium=cpc&utm_campaign=Pmax_Shopping_Boston%20Metro%20Category%20Awareness&utm_term=&utm_content=&utm_id=go_cmp-20837509568_adg-_ad-__dev-c_ext-_prd-9974567_sig-Cj0KCQjw0_WyBhDMARIsAL1Vz8t7g8jTEnTvU7pAn2EhZ3dJouXCVCa7Tl-RsGRhepSoWFnkOTOHie8aAs1fEALw_wcB&gad_source=1&gclid=Cj0KCQjw0_WyBhDMARIsAL1Vz8t7g8jTEnTvU7pAn2EhZ3dJouXCVCa7Tl-RsGRhepSoWFnkOTOHie8aAs1fEALw_wcB) run with arduino +extra flip flop board.
- [0.5W LED amazon](https://www.amazon.com/dp/B01CUGACEA/?coliid=IID4XTLLN0KJC&colid=T766DFYAY5YQ&ref_=list_c_wl_lv_ov_lig_dp_it&th=1)
- 
# Electronics
- [ ] use [0.4" VHB tape](https://www.amazon.com/HitLights-Heavy-Double-Sided-Mounting/dp/B00PKI7IBG/ref=sr_1_2_sspa?crid=JABCASGN92ZW&dib=eyJ2IjoiMSJ9.Ij13Xm94FJr9PZn3aLfDPpOQpL0kG5guXpqFnrtxxAo5pK2Q5xUq0yUacQFbH-zSo2QgTBg_bFVhZWLL-Znq0wfS8BC_IIrBnbxDLqBdnO8Q-igDLeg4ONUTyYlpehOxUtNNXOFM6gMl7QsNsprx0l6HpbT-qT5OGABhd3b-VqpXe6xpUZdOO1sgo6wEzFaCYfCxRTW95CfyDyXYOm13IduR5LlephxemCihyQlua-s.dAm5T6WgCvachPD-ag3WeXT6-WCDmPzvjQ7tvbSkYTM&dib_tag=se&keywords=vhb%2Btape&qid=1708546653&sprefix=vhb%2Btpa%2Caps%2C279&sr=8-2-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1) to mount teensy

# Programming

### 20240424 Talking with Brian
- 915 mHz band is usually multiplexed frequency hopping.  You don't see all that behind the scenes stuff.
- 2 ways to do it
1. Have the master emit a call that asks a drifter for a transciever update.  (advantage that the timing always works out) (disadvantage: teensy has to be collecting data and listening for master at same time)
2. Have the Teensy collect and collect data and then brief pause to cast out their mssg when it is their time slot.  You have to sync with GPS time.  (maybe even having the base broadcast the most recent GPS time could be an advantage)
### RFM69HCW
[Hookup guide](https://learn.sparkfun.com/tutorials/rfm69hcw-hookup-guide/hardware-overview)
- [Tips for teensy guide](https://forum.pjrc.com/index.php?threads/rfm69w-rfm69hw-transceiver-t3.24576/page-3#post-47264)

[RFM69 Git Library LowPowerLab](https://github.com/LowPowerLab/RFM69)

TRANSMIT MESSAGE:

| UTC TIME | LAT | LON | # Sats | age (GPS data) |     | Bat voltage | GPS Freq. | **ATC setting | checksum |
| -------- | --- | --- | ------ | -------------- | --- | ----------- | --------- | ------------- | -------- |
**ATC - Auto Transmission Control**
- set RSSI floor to at least -60dBm, may want to disable to not try to broadcast too quietly
- [Regulater PSU tutorial sparkfun](https://www.sparkfun.com/tutorials/103)
- 
**CSMA**
- transceiver LISTENS before it tries to TALK.  If 2 radios are actively listening to each other, another raido can mess things up by trying to talk over

**Sparkfun m8p**
- Must ALWAYS be trying to aquire a satellite.

# Solidworks

### Housing2_Boss
- internal sphere radius will need clearance for mating feature


**Mechanical:**

- Talk to Kent about Grinding our own HSS groove tool
- Hemisphere half #2 could have an internal lip to improve mating strength
- 50A durometer O-ring and 1/8" cross section would give us the best change with our space.  Could drop to a 3/32 o-ring if space is required.  (Drifter #1 was 70A durometer, 1/16 ring with an improper groove design) 
- 1mm clearance on o-ring groove (need to make battery decision to reduce O-ring size