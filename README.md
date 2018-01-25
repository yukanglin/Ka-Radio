# ** ESP8266 & VS1053 Wifi WebRadio ** #

### NFC-Ka-Radio, a WiFi shoutcast player based on ESP8266, SIC9310 and VS1053b chips</br>
The repository is forked from https://github.com/karawin/Ka-Radio.git</br>

## Basic informations<BR/>
### Release 1.6.4- Built on 2018/01/25
New:<br/>
<ul>
	<li>1.6.4 R0: add nfc function.</li>
</ul>

<br/>

To load this release, please flash <BR/>
boot_v1.6.bin at 0x0000,<BR/>
user1.4096.new.4.bin at 0x1000 ,<BR/>
user2.4096.new.4.bin at 0X81000,<BR/>
esp_init_data_default.bin at 0x3FC000 <BR/>
and blank.bin at	0x3fe000 <BR/>
After that, all next updates are done with the On The Air (OTA) feature.<BR/>
<BR/><BR/>
See also https://hackaday.io/project/11570-wifi-webradio-with-esp8266-and-vs1053 <br/>


#### Loading the esp8266
The binaries are on ESP8266-Firmware/bin/upgrade/

#### First use
- If the acces point of your router is not known, the webradio inits itself as an AP. Connect the wifi of your computer to the ssid "WifiWebRadio",  
- Browse to 192.168.4.1 to display the page, got to "setting" "Wifi" and configure your ssid ap, the password if any, the wanted IP or use dhcp if you know how to retrieve the dhcp given ip (terminal or scan of the network).
- In the gateway field, enter the ip address of your router.
- Validate. The equipment restart to the new configuration. Connect your wifi to your AP and browse to the ip given in configuration.
- Congratulation, you can edit your own station list. Dont forget to save your stations list in case of problem or for new equipments.
- if the AP is already know by the esp8266, the default ip is given by dhcp.
- a sample of stations list is on https://github.com/yukanglin/Ka-Radio/blob/master/ESP8266-Firmware/WebStations.txt . Can be uploaded via the web page.   
- Karadio can be controlled by the web interface or by the uart interface or by telnet. List of commands: type help
- See the list of command at http://karadio.karawin.fr/Interface.txt

#### NFC
- The NFC-Ka-Radio device only support read a NFC tag, which type is 14443-A.
- It can parse URI record of NDEF message.
- You can write URI record by a NFC phone/reader, before NFC-Ka-Radio device read it.
- The NFC-Ka-Radio device will play a ringtone sound when it detect a right format of NFC tag.

#### Feedback
Please tell me if you succeded or something more can be done, thanks.<br/>

<img src="https://github.com/karawin/ESP8266-WebRadio/blob/master/Images/webradio1mini.jpg" alt="screenshot" border=0> 
<img src="https://github.com/karawin/ESP8266-WebRadio/blob/master/Images/webradio2mini.jpg" alt="screenshot" border=0> 
<img src="https://github.com/karawin/ESP8266-WebRadio/blob/master/Images/webradio3mini.jpg" alt="screenshot" border=0> 

### Generate NFC-Ka-Radio
see the https://github.com/yukanglin/Ka-Radio/blob/master/readme.txt file.

### History:
- Based on https://github.com/karawin/Ka-Radio.git<br />
- New development based on the new https://github.com/espressif/ESP8266_RTOS_SDK<br />
- Compiled with [esp-open-sdk](https://github.com/pfalcon/esp-open-sdk)<br />
- Compatible with mobile<br />
- Stable<br />
- tools to save and restore the stations database<br /><br />
- prototype made with:<br />
-- http://fr.aliexpress.com/item/MB102-Breadboard-Power-Supply-Module-3-3V-5V-For-Solderless-Breadboard/2027279953.html (0.74 euros)<br />
-- http://www.pcstore.com.tw/playrobot/M25817964.htm  (NODEMCU Lolin: 126 NTD)<br />
-- http://goods.ruten.com.tw/item/show?21649678441495  (625 NTD)<br />
-- http://www.ebay.fr/itm/131683794035?_trksid=p2060353.m2749.l2649&ssPageName=STRK%3AMEBIDX%3AIT  (1.05 euros)<br />
--
http://www.pcstore.com.tw/washow/M19198772.htm (NFC Tag: 289 NTD)<br />
-- some wires ...<br /><br />
- Wiring: <br />
From ESP8266_ESP12( 3.3 v) to VS1053 (5 v)<br />
From ESP8266_ESP12( VUSB, 5 V) to SIC9310 (VIN)<br />
REST<br />
ADC   if control panel is not used, this input must be grounded.<br />
CH_PD to 3.3v<br />
GPIO16 to SIC9310 RESET<br />
GPIO2 to SIC9310 CS<br />
GPIO14 to VS1053 & SIC9310 SCK<br />
GPIO12 to VS1053 & SIC9310 MISO<br />
GPIO13 to VS1053 & SIC9310 MOSI<br />
TXD to   CH340G UART rx<br />
RXD from CH340G UART tx<br />
GPIO05 to VS1053 XDCS<br />
GPIO04 to VS1053 DREQ<br />
GPIO00 to VS1053 XRST<br />
GPIO02<br />
GPIO15 to VS1053 XCS<br />
<br/>
If your chip has a /Vhold in place of /VBAT, the pin 7 must be wired to VCC (pin8)<br/>

## Used hardware
WiFi : ESP8266 (ESP-12 with 32Mbits flash)<br />
Additional MCU (as a bridge UART<=>UI): AVR<br />
Audio decoder: VS1053<br />
NFC reader: SIC9310<br />


