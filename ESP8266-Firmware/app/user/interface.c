/******************************************************************************
 * Copyright 2015 Piotr Sperka (http://www.piotrsperka.info)/*
 * Copyright 2016 karawin (http://www.karawin.fr)
*/
#include "interface.h"
#include "user_interface.h"
//#include "osapi.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "eeprom.h"
#include "ntp.h"
char parslashquote[] = {"(\""};
char parquoteslash[] = {"\")"};
char msgsys[] = {"##SYS."};
char msgcli[] = {"##CLI."};

const char stritWIFISTATUS[] STORE_ATTR ICACHE_RODATA_ATTR = {"#WIFI.STATUS#\nStatus: %d\nIP: %d.%d.%d.%d\nMask: %d.%d.%d.%d\nGateway: %d.%d.%d.%d\n##WIFI.STATUS#\n"};
const char stritWIFISTATION[] STORE_ATTR ICACHE_RODATA_ATTR = {"\n#WIFI.STATION#\n%s\n%s\n##WIFI.STATION#\n"};
const char stritPATCH[] STORE_ATTR ICACHE_RODATA_ATTR = {"\n##VS1053 Patch will be %s after power Off and On#\n"};
const char stritCMDERROR[] STORE_ATTR ICACHE_RODATA_ATTR = {"##CMD_ERROR#\n"};
const char stritHELP0[] STORE_ATTR ICACHE_RODATA_ATTR = {" \
Commands:\n\
---------\n\
 Wifi related commands\n\
//////////////////\n\
wifi.lis: give the list of received SSID\n\
wifi.con: Display the AP1 and AP2 SSID\n\
wifi.con(\"ssid\",\"password\"): Record the given AP ssid with password in AP1 for next reboot\n\
wifi.discon: disconnect the current ssid\n\
wifi.station: the current ssid and password\n\
wifi.status: give #WIFI.STATUS#\n\n\
//////////////////\n\
  Station Client commands\n\
//////////////////\n\
cli.url(\"url\"): the name or ip of the station to instant play\n\
cli.path(\"/path\"): the path of the station to instant play\n\
cli.port(\"xxxx\"): the port number of the station to instant play\n\
cli.instant: play the instant station\n\
cli.start: start to play the current station\n\
cli.play(\"xxx\"): play the xxx recorded station in the list (0 = stop)\n\
cli.prev (or cli.previous): select the previous station in the list and play it\n\
cli.next: select the next station in the list and play it%c"};

const char stritHELP1[] STORE_ATTR ICACHE_RODATA_ATTR = {" \
cli.stop: stop the playing station or instant\n\
cli.list: list all recorded stations\n\
cli.list(\"x\"): list only one of the recorded stations. Answer with #CLI.LISTINFO#: followed by infos\n\
cli.vol(\"xxx\"): set the volume to xxx with xxx from 0 to 254 (max volume)\n\
cli.vol: ask for  the current volume. respond with ##CLI.VOL# xxx\n\
cli.vol-: Decrement the volume by 10 \n\
cli.vol+: Increment the volume by 10 \n\
Every vol command from uart or web or browser respond with ##CLI.VOL#: xxx\n\
cli.info: Respond with nameset, all icy, meta, volume and stae playing or stopped. Used to refresh the lcd informations \n\n\
//////////////////\n\
  System commands\n\
//////////////////\n\
sys.uart(\"x\"): Change the baudrate of the uart on the next reset.\n\
 Valid x are: 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 76880, 115200, 230400\n\
sys.i2s: Display the current I2S speed%c"};

const char stritHELP2[] STORE_ATTR ICACHE_RODATA_ATTR = {" \
sys.i2s(\"x\"): Change and record the I2S clock speed of the vs1053 GPIO5 MCLK for the i2s interface to external dac.\n\
: 0=48kHz, 1=96kHz, 2=192kHz, other equal 0\n\
sys.erase: erase all recorded configuration and stations.\n\
sys.heap: show the ram heap size\n\
sys.update: start an OTA (On The Air) update of the software\n\
sys.prerelease: start an OTA of the next prerelease\n\
sys.boot: reboot the webradio.\n\
sys.patch(\"x\"): Change the status of the vs1053 patch at power on.\n\
0 = Patch will not be loaded, 1 or up = Patch will be loaded (default) at power On \n\
sys.patch: Display the vs1053 patch status\n\
sys.led(\"x\"): Change the led indication:\n\
0 = Led is in Play mode (lighted when a station is playing), 1 or up = Led is in Blink mode (default)\n\
sys.led: Display the led indication status\n\
sys.version: Display the Release and Revision numbers\n\
sys.tzo(\"xx\"): Set the timezone offset of your country.%c"\
};

const char stritHELP3[] STORE_ATTR ICACHE_RODATA_ATTR = {" \
sys.tzo: Display the timezone offset\n\
sys.date: Send a ntp request and Display the current locale time\n\
: Format ISO-8601 local time   https://www.w3.org/TR/NOTE-datetime\n\
: YYYY-MM-DDThh:mm:ssTZD (eg 2017-07-16T19:20:30+01:00)\n\
sys.adc: Display the adc value\n\
sys.version: Display the release and Revision of KaraDio\n\
///////\n\
  Other\n\
///////\n\
help: this command\n\
<enter> will display\n\
#INFO:\"\"#\n\
\n\
A command error display:\n\
##CMD_ERROR#\n\r%c"}; 
uint16_t currentStation = 0;

//extern uint16_t currentStation;
extern void wsVol(char* vol);
extern void playStation(char* id);
extern void setVolume(char* vol);
extern void setRelVolume(int8_t vol);
extern uint16_t getVolume(void);


ICACHE_FLASH_ATTR void clientVol(char *s);

#define MAX_WIFI_STATIONS 50
bool inside = false;

void setVolumePlus()
{
	setRelVolume(10);
}
void setVolumeMinus()
{
	setRelVolume(-10);
}		
void setVolumew(char* vol)
{
	setVolume(vol);	
	wsVol(vol);
}	

unsigned short adcdiv;	



void readAdc()
{
	uint16_t adc;
	adc = system_adc_read();
	kprintf(PSTR("##ADC: %d * %d = %d\n"),adc,adcdiv,adc*adcdiv);
}
void readRssi()
{
	int8_t rssi;
	rssi = wifi_station_get_rssi();
	kprintf(PSTR("##RSSI: %d\n"),rssi);
}
// Read the command panel
void switchCommand() {
	uint16_t adc;
	int i = 0;
	char Vol[22];
	if (adcdiv == 0) return; // no panel

	adc = system_adc_read(); 
	adc *= adcdiv;
	
//	if (adc < 930) 
//		os_printf("adc: %d  div: %d\n",adc,adcdiv);

	if (inside&&(adc > 930)) 
	{
		inside = false;
		return;
	}	
	
//	vTaskDelay(1);
	Delay(1);
	adc = system_adc_read(); 
	adc *= adcdiv;
		
	if ((adc >400) && (adc < 580)) // volume +
	{
		setVolumePlus();
	}
	else if ((adc >730) && (adc < 830)) // volume -
	{
		setVolumeMinus();
	}		
	if (!inside)
	{	
		if (adc < 220) // stop
		{
			inside = true;
			clientDisconnect(PSTR("Adc Stop"));
		}
		else if ((adc >278) && (adc < 380)) //start
		{
			inside = true;
			sprintf(Vol,"%d",currentStation);
			playStation	(Vol);
		}
		else if ((adc >830) && (adc < 920)) // station+
		{
			inside = true;
			wsStationNext();
		}
		else if ((adc >590) && (adc < 710)) // station-
		{
			inside = true;
		wsStationPrev();
		}
	}
}



uint8_t startsWith(const char *pre, const char *str)
{
    size_t lenpre = strlen(pre),
           lenstr = strlen(str);
    return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

ICACHE_FLASH_ATTR void printInfo(char* s)
{
	kprintf(PSTR("#INFO:\"%s\"#\n"), s);
}
char msg[] = {"#WIFI.LIST#"};

ICACHE_FLASH_ATTR void wifiScanCallback(void *arg, STATUS status)
{
	if(status == OK)
	{
		int i = MAX_WIFI_STATIONS;
//		char msg[] = {"#WIFI.LIST#"};
		char* buf;
		struct bss_info *bss_link = (struct bss_info *) arg;
//		kprintf(PSTR("\n%s"),msg);
		buf = malloc(128);
		if (buf == NULL) return;
		while(i > 0)
		{
			i--;
			bss_link = bss_link->next.stqe_next;
			if(bss_link == NULL) break;
			sprintf(buf, "\n%s;%d;%d;%d", bss_link->ssid, bss_link->channel, bss_link->rssi, bss_link->authmode);
			kprintf(PSTR("%s\n"),buf);
		}
		kprintf(PSTR("\n#%s"),msg);
		free(buf);
	}
}

ICACHE_FLASH_ATTR void wifiScan()
{
	wifi_station_scan(NULL, wifiScanCallback);
}

ICACHE_FLASH_ATTR void wifiConnect(char* cmd)
{
	int i;
	struct device_settings* devset = getDeviceSettings();
	for(i = 0; i < 32; i++) devset->ssid[i] = 0;
	for(i = 0; i < 64; i++) devset->pass[i] = 0;	
	char *t = strstr(cmd, parslashquote);
	if(t == 0)
	{
		kprintf(stritCMDERROR);
		free(devset);
		return;
	}
	char *t_end  = strstr(t, "\",\"");
	if(t_end == 0)
	{
		kprintf(stritCMDERROR);
		free(devset);
		return;
	}
	
	strncpy( devset->ssid, (t+2), (t_end-t-2) );
	
	t = t_end+3;
	t_end = strstr(t, parquoteslash);
	if(t_end == 0)
	{
		kprintf(stritCMDERROR);
		free(devset);
		return;
	}
	
	strncpy( devset->pass, t, (t_end-t)) ;
	devset->dhcpEn = 1;
	saveDeviceSettings(devset);
	kprintf(PSTR("\n##AP1: %s with dhcp on next reset#\n"),devset->ssid);
	free(devset);
}

ICACHE_FLASH_ATTR void wifiConnectMem()
{
	
	struct device_settings* devset = getDeviceSettings();
	kprintf(PSTR("\n##AP1: %s#"),devset->ssid);
	kprintf(PSTR("\n##AP2: %s#\n"),devset->ssid2);
	free(devset);
	 
/*	int i;
	struct station_config* cfg = malloc(sizeof(struct station_config));
	
	for(i = 0; i < 32; i++) cfg->ssid[i] = 0;
	for(i = 0; i < 64; i++) cfg->password[i] = 0;
	cfg->bssid_set = 0;
	
	wifi_station_disconnect();
	
	struct device_settings* devset = getDeviceSettings();
	for(i = 0; i < strlen(devset->ssid); i++) cfg->ssid[i] = devset->ssid[i];
	for(i = 0; i < strlen(devset->pass); i++) cfg->password[i] = devset->pass[i];
	free(devset);

	wifi_station_set_config(cfg);

	if( wifi_station_connect() ) kprintf(PSTR("\n##WIFI.CONNECTED#"));
	else kprintf(PSTR("\n##WIFI.NOT_CONNECTED#"));
	
	free(cfg);
	*/
}

ICACHE_FLASH_ATTR void wifiDisconnect()
{
	if(wifi_station_disconnect()) kprintf(PSTR("\n##WIFI.NOT_CONNECTED#"));
	else kprintf(PSTR("\n##WIFI.DISCONNECT_FAILED#"));
}

ICACHE_FLASH_ATTR void wifiStatus()
{
	struct ip_info ipi;
	uint8_t t = wifi_station_get_connect_status();	
	wifi_get_ip_info(0, &ipi);
	kprintf(stritWIFISTATUS,
			  t, (ipi.ip.addr&0xff), ((ipi.ip.addr>>8)&0xff), ((ipi.ip.addr>>16)&0xff), ((ipi.ip.addr>>24)&0xff),
			 (ipi.netmask.addr&0xff), ((ipi.netmask.addr>>8)&0xff), ((ipi.netmask.addr>>16)&0xff), ((ipi.netmask.addr>>24)&0xff),
			 (ipi.gw.addr&0xff), ((ipi.gw.addr>>8)&0xff), ((ipi.gw.addr>>16)&0xff), ((ipi.gw.addr>>24)&0xff));
}

ICACHE_FLASH_ATTR void wifiGetStation()
{
	struct station_config cfgg;
	wifi_station_get_config(&cfgg);
	kprintf(stritWIFISTATION, cfgg.ssid, cfgg.password);
}

ICACHE_FLASH_ATTR void clientParseUrl(char* s)
{
    char *t = strstr(s, parslashquote);
	if(t == 0)
	{
		kprintf(stritCMDERROR);
		return;
	}
	char *t_end  = strstr(t, parquoteslash)-2;
    if(t_end <= 0)
    {
		kprintf(stritCMDERROR);
		return;
    }
    char *url = (char*) malloc((t_end-t+1)*sizeof(char));
    if(url != NULL)
    {
        uint8_t tmp;
        for(tmp=0; tmp<(t_end-t+1); tmp++) url[tmp] = 0;
        strncpy(url, t+2, (t_end-t));
        clientSetURL(url);
        free(url);
    }
}

ICACHE_FLASH_ATTR void clientParsePath(char* s)
{
    char *t = strstr(s, parslashquote);
	if(t == 0)
	{
		kprintf(stritCMDERROR);
		return;
	}
//	kprintf(PSTR("cli.path: %s\n"),t);
	char *t_end  = strstr(t, parquoteslash)-2;
    if(t_end <= 0)
    {
		kprintf(stritCMDERROR);
		return;
    }
    char *path = (char*) malloc((t_end-t+1)*sizeof(char));
    if(path != NULL)
    {
        uint8_t tmp;
        for(tmp=0; tmp<(t_end-t+1); tmp++) path[tmp] = 0;
        strncpy(path, t+2, (t_end-t));
	kprintf(PSTR("cli.path: %s\n"),path);
        clientSetPath(path);
        free(path);
    }
}

ICACHE_FLASH_ATTR void clientParsePort(char *s)
{
    char *t = strstr(s, parslashquote);
	if(t == 0)
	{
		kprintf(stritCMDERROR);
		return;
	}
	char *t_end  = strstr(t, parquoteslash)-2;
    if(t_end <= 0)
    {
		kprintf(stritCMDERROR);
		return;
    }
    char *port = (char*) malloc((t_end-t+1)*sizeof(char));
    if(port != NULL)
    {
        uint8_t tmp;
        for(tmp=0; tmp<(t_end-t+1); tmp++) port[tmp] = 0;
        strncpy(port, t+2, (t_end-t));
        uint16_t porti = atoi(port);
        clientSetPort(porti);
        free(port);
    }
}


ICACHE_FLASH_ATTR void clientPlay(char *s)
{
    char *t = strstr(s, parslashquote);
	if(t == 0)
	{
		kprintf(stritCMDERROR);
		return;
	}
	char *t_end  = strstr(t, parquoteslash)-2;
    if(t_end <= 0)
    {
		kprintf(stritCMDERROR);
		return;
    }
   char *id = (char*) malloc((t_end-t+1)*sizeof(char));
    if(id != NULL)
    {
        uint8_t tmp;
        for(tmp=0; tmp<(t_end-t+1); tmp++) id[tmp] = 0;
        strncpy(id, t+2, (t_end-t));
		playStation(id);
        free(id);
    }	
}

const char strilLIST[] STORE_ATTR ICACHE_RODATA_ATTR = {"##CLI.LIST#%c"};
const char strilINFOND[] STORE_ATTR ICACHE_RODATA_ATTR = {"#CLI.LISTINFO#: %3d: not defined\n"};
const char strilINFO[] STORE_ATTR ICACHE_RODATA_ATTR = {"#CLI.LISTINFO#: %3d: %s, %s:%d%s\n"};
const char strilINFO1[] STORE_ATTR ICACHE_RODATA_ATTR = {"#CLI.LISTNUM#: %3d: %s, %s:%d%s\n"};
const char strilDINFO[] STORE_ATTR ICACHE_RODATA_ATTR = {"\n#CLI.LIST#%c"};


ICACHE_FLASH_ATTR void clientList(char *s)
{
	struct shoutcast_info* si;
	uint16_t i = 0,j = 255;
	bool onlyOne = false;
	
	char *t = strstr(s, parslashquote);
	if(t != NULL) // a number specified
	{	
		char *t_end  = strstr(t, parquoteslash)-2;
		if(t_end <= 0)
		{
			kprintf(stritCMDERROR);
			return;
		}	
		i = atoi(t+2);
		if (i>254) i = 0;
		j = i+1;
		onlyOne = true;
		
	} 
	{	
		kprintf(strilDINFO,0x0d);	
		for (i ;i <j;i++)
		{
			vTaskDelay(1);
			si = getStation(i);
			
			if ((si == NULL) || (si->port ==0))
			{
				//kprintf(strilINFOND,i);
				if (si != NULL) {free(si);}
				continue;
			}

			if (si !=NULL)
			{
				if(si->port !=0)
				{	
					if (onlyOne)
						kprintf(strilINFO,i,si->name,si->domain,si->port,si->file);	
					else
						kprintf(strilINFO1,i,si->name,si->domain,si->port,si->file);
				}
				free(si);
			}	
		}	
		kprintf(strilLIST,0x0d);
	}
}
ICACHE_FLASH_ATTR void clientInfo()
{
	struct shoutcast_info* si;
	si = getStation(currentStation);
	if (si != NULL)
	{
		ntp_print_time();
		clientSetName(si->name,currentStation);
		clientPrintHeaders();
		clientVol("");
		clientPrintState();
		free(si);
	}
}

ICACHE_FLASH_ATTR char* webInfo()
{
	struct shoutcast_info* si;
	si = getStation(currentStation);
	char* resp = malloc(1024);
	if (si != NULL)
	{
		if (resp != NULL)
		{	
			sprintf(resp,"vol: %d\nnum: %d\nstn: %s\ntit: %s\nsts: %d\n",getVolume(),currentStation,si->name,getMeta(),getState());
		}
		free(si);
	}
	return resp;

}
ICACHE_FLASH_ATTR char* webList(int id)
{
	struct shoutcast_info* si;
	si = getStation(id);
	char* resp = malloc(1024);
	if (si != NULL)
	{
		if (resp != NULL)
		{
			sprintf(resp,"%s\n",si->name);
		}
		free(si);
	}
	return resp;

}

ICACHE_FLASH_ATTR void sysI2S(char* s)
{
    char *t = strstr(s, parslashquote);
	struct device_settings *device;
	device = getDeviceSettings();
	if(t == NULL)
	{
		kprintf(PSTR("\n##I2S speed: %d, 0=48kHz, 1=96kHz, 2=192kHz#\n"),device->i2sspeed);
		free(device);
		return;
	}
	char *t_end  = strstr(t, parquoteslash);
    if(t_end == NULL)
    {
		kprintf(stritCMDERROR);
		free(device);
		return;
    }	
	uint8_t speed = atoi(t+2);
	VS1053_I2SRate(speed);

	device->i2sspeed = speed;
	saveDeviceSettings(device);	
	kprintf(PSTR("\n##I2S speed: %d, 0=48kHz, 1=96kHz, 2=192kHz#\n"),speed);
	free(device);
}
ICACHE_FLASH_ATTR void sysUart(char* s)
{
	bool empty = false;
	char *t ;
	char *t_end;
	if (s != NULL)
	{	
		t = strstr(s, parslashquote);
		if(t == NULL)
		{
			empty = true;
		} else
		{
			t_end  = strstr(t, parquoteslash);
			if(t_end == NULL)
			{
				empty = true;
			}	
		}
	}
	struct device_settings *device;
	device = getDeviceSettings();
	if ((!empty)&&(t!=NULL))
	{
		uint32_t speed = atoi(t+2);
		speed = checkUart(speed);
		device->uartspeed= speed;
		saveDeviceSettings(device);	
		kprintf("Speed: %d\n",speed);
	}
	kprintf(PSTR("\n%sUART= %d# on next reset\n"),msgsys,device->uartspeed);	
	free(device);
}
ICACHE_FLASH_ATTR void clientVol(char *s)
{
    char *t = strstr(s, parslashquote);
	if(t == 0)
	{
		// no argument, return the current volume
		kprintf(PSTR("%sVOL#: %d\n"),msgcli,getVolume());
		return;
	}
	char *t_end  = strstr(t, parquoteslash)-2;
    if(t_end <= 0)
    {

		kprintf(stritCMDERROR);
		return;
    }
   char *vol = (char*) malloc((t_end-t+1)*sizeof(char));
    if (vol != NULL)
    {
        uint8_t tmp;
        for(tmp=0; tmp<(t_end-t+1); tmp++) vol[tmp] = 0;
        strncpy(vol, t+2, (t_end-t));
		if ((atoi(vol)>=0)&&(atoi(vol)<=254))
		{	
			setVolumew(vol);	}	
			free(vol);
    }	
}

ICACHE_FLASH_ATTR void syspatch(char* s)
{
    char *t = strstr(s, parslashquote);
	struct device_settings *device;
	device = getDeviceSettings();
	if(t == NULL)
	{
		if ((device->options & T_PATCH)!= 0)
			kprintf(PSTR("\n##VS1053 Patch is not loaded#%c"),0x0d);
		else
			kprintf(PSTR("\n##VS1053 Patch is loaded#%c"),0x0d);
		free(device);
		return;
	}
	char *t_end  = strstr(t, parquoteslash);
    if(t_end == NULL)
    {
		kprintf(stritCMDERROR);
		free(device);
		return;
    }	
	uint8_t value = atoi(t+2);
	if (value ==0) 
		device->options |= T_PATCH; 
	else 
		device->options &= NT_PATCH; // 0 = load patch
	
	saveDeviceSettings(device);	
	kprintf(stritPATCH,(device->options & T_PATCH)!= 0?"unloaded":"Loaded");
	free(device);	
}

ICACHE_FLASH_ATTR void sysled(char* s)
{
    char *t = strstr(s, parslashquote);
	struct device_settings *device;
	device = getDeviceSettings();
	extern bool ledStatus;
	if(t == NULL)
	{
		kprintf(PSTR("##Led is in %s mode#\n"),((device->options & T_LED)== 0)?"Blink":"Play");
		free(device);
		return;
	}
	char *t_end  = strstr(t, parquoteslash);
    if(t_end == NULL)
    {
		kprintf(stritCMDERROR);
		free(device);
		return;
    }	
	uint8_t value = atoi(t+2);
	if (value ==0) 
	{device->options |= T_LED; ledStatus = false; }
	else 
	{device->options &= NT_LED; ledStatus =true;} // options:0 = ledStatus true = Blink mode
	
	saveDeviceSettings(device);	
	kprintf(PSTR("##LED is in %s mode#\n"),((device->options & T_LED)== 0)?"Blink":"Play");
	free(device);
	
}

ICACHE_FLASH_ATTR void tzoffset(char* s)
{
	char *t = strstr(s, parslashquote);
	struct device_settings *device;
	
	device = getDeviceSettings();
	if(t == NULL)
	{
		kprintf(PSTR("##SYS.TZO#: %d\n"),device->tzoffset);
		free(device);
		return;
	}
	char *t_end  = strstr(t, parquoteslash);
    if(t_end == NULL)
    {
		kprintf(stritCMDERROR);
		free(device);
		return;
    }	
	uint8_t value = atoi(t+2);
	device->tzoffset = value;	
	saveDeviceSettings(device);	
	kprintf(PSTR("##SYS.TZO#: %d\n"),device->tzoffset);
	free(device);		
}

ICACHE_FLASH_ATTR void heapSize()
{
	int hps = xPortGetFreeHeapSize( );
	kprintf(PSTR("%sHEAP: %d #\n"),msgsys,hps);
}

ICACHE_FLASH_ATTR void checkCommand(int size, char* s)
{
	char *tmp = (char*)malloc((size+1)*sizeof(char));
	int i;
	for(i=0;i<size;i++) tmp[i] = s[i];
	tmp[size] = 0;
	// kprintf(PSTR("size: %d, cmd=%s\n"),size,tmp);
	if(startsWith ("wifi.", tmp))
	{
		if     (strcmp(tmp+5, "list") == 0) 	wifiScan();
		else if(strcmp(tmp+5, "con") == 0) 	wifiConnectMem();
		else if(startsWith ("con", tmp+5)) 	wifiConnect(tmp);
		else if(strcmp(tmp+5, "rssi") == 0) 	readRssi();
		else if(strcmp(tmp+5, "discon") == 0) wifiDisconnect();
		else if(strcmp(tmp+5, "status") == 0) wifiStatus();
		else if(strcmp(tmp+5, "station") == 0) wifiGetStation();
		else printInfo(tmp);
	} else
	if(startsWith ("cli.", tmp))
	{
		if     (startsWith (  "url", tmp+4)) 	clientParseUrl(tmp);
		else if(startsWith (  "path", tmp+4))	clientParsePath(tmp);
		else if(startsWith (  "port", tmp+4)) 	clientParsePort(tmp);
		else if(strcmp(tmp+4, "instant") == 0) {clientDisconnect(PSTR("cli instantplay"));clientConnectOnce();}
		else if(strcmp(tmp+4, "start") == 0) 	clientPlay("(\"255\")"); // outside value to play the current station
		else if(strcmp(tmp+4, "stop") == 0) 	clientDisconnect(PSTR("cli stop"));
		else if(startsWith (  "list", tmp+4)) 	clientList(tmp);
		else if(strcmp(tmp+4, "next") == 0) 	wsStationNext();
		else if(strncmp(tmp+4,"previous",4) == 0) wsStationPrev();
		else if(startsWith (  "play",tmp+4)) 	clientPlay(tmp);
		else if(strcmp(tmp+4, "vol+") == 0) 	setVolumePlus();
		else if(strcmp(tmp+4, "vol-") == 0) 	setVolumeMinus();
		else if(strcmp(tmp+4, "info") == 0) 	clientInfo();
		else if(startsWith (  "vol",tmp+4)) 	clientVol(tmp);
		else printInfo(tmp);
	} else
	if(startsWith ("sys.", tmp))
	{
			 if(startsWith (  "i2s",tmp+4)) 	sysI2S(tmp);
		else if(strcmp(tmp+4, "adc") == 0) 		readAdc();
		else if(startsWith (  "uart",tmp+4)) 	sysUart(tmp);
		else if(strcmp(tmp+4, "erase") == 0) 	eeEraseAll();
		else if(strcmp(tmp+4, "heap") == 0) 	heapSize();
		else if(strcmp(tmp+4, "boot") == 0) 	system_restart();
		else if(strcmp(tmp+4, "update") == 0) 	update_firmware("new");
		else if(strcmp(tmp+4, "prerelease") == 0) 	update_firmware("prv");
		else if(startsWith (  "patch",tmp+4)) 	syspatch(tmp);
		else if(startsWith (  "led",tmp+4)) 	sysled(tmp);
		else if(strcmp(tmp+4, "date") == 0) 	ntp_print_time();
		else if(strncmp(tmp+4, "version",4) == 0) 	kprintf(PSTR("Release: %s, Revision: %s\n"),RELEASE,REVISION);
		else if(startsWith(   "tzo",tmp+4)) 	tzoffset(tmp);
		else if(startsWith(   "log",tmp+4)) 	; // do nothing
		else printInfo(tmp);
	}
	else 
	{
		if(strcmp(tmp, "help") == 0)
		{
			kprintfl(stritHELP0,0x0d);
			vTaskDelay(1);
			kprintfl(stritHELP1,0x0d);
			vTaskDelay(1);
			kprintfl(stritHELP2,0x0d);
			vTaskDelay(1);
			kprintfl(stritHELP3,0x0d);
		}
		else printInfo(tmp);
	}	
	free(tmp);
	
}
