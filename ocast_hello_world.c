#include <wiringPi.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <strings.h>


typedef struct{
  int y;
  int mo;
  int d;
  int h;
  int mi;
  int s;
  unsigned long unix;
}mcp_timestamp_t;

typedef union { // Union makes conversion from 4 bytes to an unsigned 32-bit int easy
  unsigned char bytes[4];
  unsigned long uint32;
  unsigned uint16;
  unsigned char uint8;
  long  int32;
  int int16;
  char  int8;
} data_u;

typedef struct{
 // mcp_timestamp_t t;

  unsigned long Ai[8];
  unsigned long Ti;
  unsigned A0i;
  int Ai16[8];
  float A[8];
  float A0;
  float TC,TF;
}measurements_t;//ADS1118_t;

typedef struct{
	char buff[32];
	unsigned char pos;
	unsigned char ready;
}buff_t;


//	gcc -Wall -o OCAST_heyworld ocast_hello_world.c -lwiringPi
//	sudo ./OCAST_heyworld
#define SPIMODE 0
#define ADS_DEG_PER_BIT     (0.03125*0.25)
#define VperBIT_6144      (187.5e-6)
#define VperBIT_4096      (125.0e-6)
#define VperBIT_2048      (62.5e-6)
#define VperBIT_1024      (31.25e-6)
#define VperBIT_0512      (15.625e-6)
#define VperBIT_0256      (7.8125e-6)

#define MUX_A0A1  0x00
#define MUX_A0A3  0x01
#define MUX_A1A3  0x02
#define MUX_A2A3  0x03
#define MUX_A0    0x04
#define MUX_A1    0x05
#define MUX_A2    0x06
#define MUX_A3    0x07

#define FSR_6144  0x00
#define FSR_4096  0x01
#define FSR_2048  0x02
#define FSR_1024  0x03
#define FSR_0512  0x04
#define FSR_0256  0x05

#define MODE_CONT 0x00
#define MODE_SING 0x01

#define DR_SPS_8  0x00
#define DR_SPS_16 0x01
#define DR_SPS_32 0x02
#define DR_SPS_64 0x03
#define DR_SPS_128  0x04
#define DR_SPS_250  0x05
#define DR_SPS_475  0x06
#define DR_SPS_860  0x07

#define TS_MODE_ADC 0x00
#define TS_MODE_TMP 0x01

#define PU_ENABLE 0x01
#define PU_DISABLE  0x00

#define BUILD_ADS_CMD(mux,fsr,mode,dr,tsmode,puen)  ((1<<15)|(mux<<12)|(fsr<<9)|(mode<<8)|(dr<<5)|(tsmode<<4)|(puen<<3)|(0x03<<0))



//measurements_t ADC;





data_u Data;


#define GPIO2		8	//SDA
#define GPIO3		9	//SCL
#define GPIO4		7
#define GPIO17		0
#define GPIO22		3
#define GPIO10		12		//MOSI
#define GPIO9		13		//MISO
#define GPIO11		14		//SCK

#define GPIO14		15	//TXD
#define GPIO15		16	//RXD
#define GPIO18		1
#define GPIO23		4
#define GPIO24		5
#define GPIO25		6
#define GPIO8		10	//SS0
#define GPIO7		11	//SS1


// #define GPIO5	5
// #define GPIO6	0
// #define GPIO13	0
// #define GPIO19	0
// #define GPIO26	0
// #define GPIO27	


#define STATLED_ERROR_pin		GPIO17
#define STATLED_OK_pin			GPIO22

#define SW_MOSI					GPIO10
#define SW_MISO					GPIO9
#define SW_SCK					GPIO11


#define MCP79510_CS_pin			GPIO25
#define MCP79510_enable()		pinMode(MCP79510_CS_pin,OUTPUT); digitalWrite(MCP79510_CS_pin,LOW)
#define MCP79510_disable()		pinMode(MCP79510_CS_pin,OUTPUT); digitalWrite(MCP79510_CS_pin,HIGH)



int ADS1118_CS_pin2 = GPIO7;
int ADS1118_CS_pin = GPIO8;


#define ADS1118_enable()    pinMode(ADS1118_CS_pin2,OUTPUT);digitalWrite(ADS1118_CS_pin2,HIGH);pinMode(ADS1118_CS_pin,OUTPUT);digitalWrite(ADS1118_CS_pin,LOW);
#define ADS1118_disable()   pinMode(ADS1118_CS_pin2,OUTPUT);digitalWrite(ADS1118_CS_pin2,HIGH);pinMode(ADS1118_CS_pin,OUTPUT);digitalWrite(ADS1118_CS_pin,HIGH);


#define STATUS_LIGHT_OK()			digitalWrite(STATLED_OK_pin,HIGH);digitalWrite(STATLED_ERROR_pin,LOW);
#define STATUS_LIGHT_ERROR()		digitalWrite(STATLED_OK_pin,LOW);digitalWrite(STATLED_ERROR_pin,HIGH);


float data_V[8] = {
	VperBIT_0256,
	VperBIT_0256,
	VperBIT_0256,
	VperBIT_4096,
	VperBIT_0256,
	VperBIT_0256,
	ADS_DEG_PER_BIT,
	//1.0,
	VperBIT_0256
};
unsigned char data_V_fsr[8] = {
	FSR_0256,
	FSR_0256,
	FSR_0256,
	FSR_4096,
	FSR_4096,
	FSR_0256,
	FSR_0256,
	FSR_6144
};
int cont_conv = MODE_CONT;

unsigned short curr_cmd = 0;


int chan = 0;


void loop();
void setup();
unsigned char SPI_transfer_byte(unsigned char byte_out, unsigned char mode);
void ADS1118_update(int *rdg);
void ADS1118_cmd(unsigned short cmd);
void change_chip(int chip);
void sw_SPI_begin();
void init_ADS();
void ILLUMINATE_STAT_LED(unsigned char a);

short MCP79510_writeRegister( unsigned char addr , unsigned char Data );
unsigned char MCP79510_readRegister(unsigned char addr);
mcp_timestamp_t get_fw_time();
mcp_timestamp_t MCP79510_getTime();
void MCP79510_init();
int month_to_val(const char *mostr);




float t0 = 0;


int main (void)
{
//	printf("Hello world!\n");
	t0 = clock();
	wiringPiSetup () ;
	setup();
	
	for (;;)
	{
		loop();
	}
	return 0 ;
}



FILE *ofp;          /*assigns a pointer to string *ofp */
	
void setup()
{
///	pinMode(GPIO17, OUTPUT) ;
	//pinMode(GPIO22, OUTPUT) ;

	pinMode(STATLED_ERROR_pin,OUTPUT);	//status LED pin
	pinMode(STATLED_OK_pin,OUTPUT);	//status LED pin
	STATUS_LIGHT_ERROR();

	init_ADS();
//	ADS1118_cmd(ADS_READ_A0A1);
	ADS1118_cmd( BUILD_ADS_CMD( MUX_A0A1 , data_V_fsr[chan] , cont_conv , DR_SPS_128 , TS_MODE_ADC , PU_DISABLE) );
	
	
}

#define filepath	"/home/pi/Desktop/"
#define filepath_thumbdrive	"/media/pi/THUMB1GB/"


char filename[16] = {0};
char filenamepath[64] = {0};
char filenamepath_thumbdrive[64] = {0};
	


void loop()
{
	static int last_yy = 0;
	static int last_mm = 0;
	static int last_dd = 0;
	float dt = clock() - t0;
	float t_sec = dt / (
	
	ILLUMINATE_STAT_LED(0);

//	digitalWrite(GPIO17, HIGH); 
	delay(100);
	ADS1118_cmd( BUILD_ADS_CMD( MUX_A0A1 , data_V_fsr[chan] , cont_conv , DR_SPS_128 , TS_MODE_ADC , PU_DISABLE) );

//	digitalWrite(GPIO17,  LOW); 
	delay(50);



	
//	static int seconds_last = 99;
	char TimeString[128];

// 	timeval curTime;
// 	gettimeofday(&curTime, NULL);
//	if (seconds_last == curTime.tv_sec)return;
	
//	seconds_last = curTime.tv_sec;
	
//	strftime(TimeString, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));
	
	static int yy = 2016;
	static int mm = 2;
	static int dd = 18;
	static int HH = 17;
	static int MM = 26;
	static int SS = 42;
	
	
	if( (last_yy != yy)||(last_mm != mm)||(last_dd != dd) ){
		last_yy = yy;
		last_mm = mm;
		last_dd = dd;
		sprintf(filename,"%04d%02d%02d.txt",yy,mm,dd);
		sprintf(filenamepath,"%s%s",filepath,filename);
		sprintf(filenamepath_thumbdrive,"%s%s",filepath_thumbdrive,filename);
	
	}
	sprintf(TimeString,"%04d-%02d-%02d\t%02d:%02d\t%02d\t",yy,mm,dd,HH,MM,SS);
	
	int rdg = 0;	
	ADS1118_update(&rdg);
	float rdg_v = rdg;
    rdg_v *= data_V[chan];
    rdg_v *= 1e6;
    
    printf("%s\t%7.1f uV\n",TimeString,rdg_v);
    

	//printf("Opening file...\n");
    ofp = fopen(filenamepath, "a+"); 
    if(ofp){
		fprintf(ofp,"%s\t%7.1f uV\n",TimeString,rdg_v);
		fclose(ofp);
    }

    ofp = fopen(filenamepath_thumbdrive, "a+"); 
    if(ofp){
		fprintf(ofp,"%s\t%7.1f uV\n",TimeString,rdg_v);
		fclose(ofp);
    }


    
    
    
	
//	printf("Rdg = %d\n",(int)rdg);

	SS++;
	if(SS > 59){
		SS = 0;
		MM++;
		if(MM > 59){
			MM = 0;
			HH++;
			if(HH > 23){
				HH = 0;
				dd++;
				if(dd > 28){
					dd = 1;
					mm++;
					if(mm > 12){
						mm++;
						yy++;
					}
				}
			}
		}
	}
	
		


}
//0001 0101 0000 0101
//1000 1010 1000 0011
//.000 1010 1000 010.

// afp://manbearpig central._afpovertcp._tcp.local/Data/NashStorage/thumb
// smb://"Michael Nash":" "@10.0.1.1/Data

void ILLUMINATE_STAT_LED(unsigned char a)
{
	if(a==1){
		STATUS_LIGHT_ERROR();
	}
	else{
		STATUS_LIGHT_OK();
	}
}//	

void init_ADS()
{
  
  digitalWrite(ADS1118_CS_pin,HIGH);
  
  pinMode (ADS1118_CS_pin, OUTPUT); 
  
  sw_SPI_begin();
  
}

void sw_SPI_begin()
{
  pinMode(SW_SCK, OUTPUT); // set SCK pin to output
  
  pinMode(SW_MOSI, OUTPUT); // set MOSI pin to output
  
  pinMode(SW_MISO, INPUT); // set MOSI pin to output
  
  digitalWrite(ADS1118_CS_pin, HIGH); // hold slave select 1 pin high, so that chip is not selected to begin with
}

void change_chip(int chip)
{

	pinMode(ADS1118_CS_pin2,OUTPUT);
	pinMode(ADS1118_CS_pin,OUTPUT);

	digitalWrite(ADS1118_CS_pin2,HIGH);
	digitalWrite(ADS1118_CS_pin,HIGH);

	if(chip == 0){
		ADS1118_CS_pin = 9;
		ADS1118_CS_pin2 = 10;
	}
	else{
		ADS1118_CS_pin = 10;
		ADS1118_CS_pin2 = 9;
	}
}



void ADS1118_cmd(unsigned short cmd)
{
//  static unsigned long last_call_time = 0;
  data_u dat_cmd;
  dat_cmd.uint16 = cmd; 
//  data_u adc_rdg;
//  data_u readback;
//  adc_rdg.uint32 = 0;

//Send configuration
  ADS1118_enable();
    SPI_transfer_byte(dat_cmd.bytes[1], SPIMODE);
    SPI_transfer_byte(dat_cmd.bytes[0], SPIMODE);
//     readback.bytes[1] = SPI_transfer_byte(dat_cmd.bytes[1], SPIMODE);
//     readback.bytes[0] = SPI_transfer_byte(dat_cmd.bytes[0], SPIMODE);
	SPI_transfer_byte(dat_cmd.bytes[1], SPIMODE);
	SPI_transfer_byte(dat_cmd.bytes[0], SPIMODE);

  ADS1118_disable();
  

//  printf("{%02X %02X}--RDG:[%02X %02X] (cmd)\n",(unsigned char)readback.bytes[1],(unsigned char)readback.bytes[0],(unsigned char)adc_rdg.bytes[1],(unsigned char)adc_rdg.bytes[0]);    
//  printf("Readback:  %02X %02X\n",(unsigned char)readback.bytes[1],(unsigned char)readback.bytes[0]);

  curr_cmd = dat_cmd.uint16;
  return;
}


void ADS1118_update(int *rdg)
{
//  unsigned char good_read = 0;
//  int tries = 0;

  static data_u adc_rdg;
  static unsigned char first_run = 1;
  

//  data_u readback;


  if(first_run == 1)
  {
    first_run = 0;
    adc_rdg.uint32 = 0;
  }

//  unsigned long time_since_last_read = millis() - last_read_t0;
  //DRDY is active...get new reading
//  printf("(%d)\n",(int)GET_SD_CS_STATE());

  ADS1118_enable();
    adc_rdg.bytes[1] = SPI_transfer_byte(0xFF, SPIMODE);
    adc_rdg.bytes[0] = SPI_transfer_byte(0xFF, SPIMODE);
//    readback.bytes[1] = SPI_transfer_byte(0xFF, SPIMODE);
//    readback.bytes[0] = SPI_transfer_byte(0xFF, SPIMODE);
	SPI_transfer_byte(0xFF, SPIMODE);
	SPI_transfer_byte(0xFF, SPIMODE);
  ADS1118_disable();    

//    if((readback.uint16&0x7FF8) == (curr_cmd&0x7FF8)){
      *rdg = (short)adc_rdg.int32;
//    float rdg_v = (short)adc_rdg.int32;
  //  rdg_v *= data_V[chan];
    //rdg_v *= 1e6;
    
//      printf("{%02X %02X}--RDG:[%02X %02X]  %7.1f uV\n",(unsigned char)readback.bytes[1],(unsigned char)readback.bytes[0],(unsigned char)adc_rdg.bytes[1],(unsigned char)adc_rdg.bytes[0],rdg_v);
//      (short)adc_rdg.uint16);
//    }
/*    else{
    	int v = (short)adc_rdg.uint16;
      printf("{%02X %02X}--RDG:[%02X %02X]  (BAD, expected %02X %02X) (%d)\n",
      (unsigned char)readback.bytes[1],
      (unsigned char)readback.bytes[0],
      (unsigned char)adc_rdg.bytes[1],
      (unsigned char)adc_rdg.bytes[0],
      (unsigned char)(curr_cmd>>8),
      (unsigned char)(curr_cmd&0xFF),
      v);
      
    }  */

/*
printf("{%02X %02X}--RDG:[%02X %02X]\n",
      (unsigned char)readback.bytes[1],
      (unsigned char)readback.bytes[0],
      (unsigned char)adc_rdg.bytes[1],
      (unsigned char)adc_rdg.bytes[0]);     
      */
//    ADS1118_cmd(ADS_READ_A0A1);

//    last_read_t0 = millis();
//    actual_last_adc_update_t0 = last_read_t0;
}

unsigned char SPI_transfer_byte(unsigned char byte_out, unsigned char mode)
{
// #define SPI_SCLK_LOW_TIME 5
// #define SPI_SCLK_HIGH_TIME 5
#define SPI_SCLK_LOW_TIME 5
#define SPI_SCLK_HIGH_TIME 5


  //digitalWrite(SS_pin,LOW);

    unsigned char byte_in = 0;
    unsigned char bit;
    
    ILLUMINATE_STAT_LED(1);


    digitalWrite(SW_SCK, LOW);    //ensure low initial state
  
    for (bit = 0x80; bit; bit >>= 1) {
    
      digitalWrite(SW_MOSI, (byte_out & bit) ? HIGH : LOW );    /* Shift-out a bit to the MOSI line */
    
      delayMicroseconds(SPI_SCLK_LOW_TIME); /* Delay for at least the peer's setup time */
    
      digitalWrite(SW_SCK, HIGH); /* Pull the clock line high */
    
      if (digitalRead(SW_MISO) == HIGH) byte_in |= bit; /* Shift-in a bit from the MISO line */
    
      delayMicroseconds(SPI_SCLK_HIGH_TIME);  /* Delay for at least the peer's hold time */
    
      digitalWrite(SW_SCK,LOW); /* Pull the clock line low */
    }
  
  
  ILLUMINATE_STAT_LED(0);

  return byte_in;
}



















int month_to_val(const char *mostr)
{
	switch(mostr[0]){
		case 'J':
			switch(mostr[1]){
				case 'a': return 1;
				case 'u':
					switch(mostr[2]){
						case 'l': return 7;
						case 'n': return 6;
						default: return 0;
					}
					break;
				default: return 0;
			}
			break;
		case 'F': return 2;
		case 'M': 
			switch(mostr[2]){
				case 'r': return 3;
				case 'y': return 5;
				default: return 0;
			}
			break;
		case 'A':
			switch(mostr[1]){
				case 'p': return 4;
				case 'u': return 8;
				default: return 0;
			}
			break;
		case 'S': return 9;
		case 'O': return 10;
		case 'N': return 11;
		case 'D': return 12;
		default: return 0;
	}
	return 0;	
}





void MCP79510_init()
{
	Serial.print("Initializing RTC...");
	MCP79510_disable();
//	SPI.begin();
// 	SPI.setFrequency(MCP79510_FREQ);			//SPI.setClockDivider(SPI_CLOCK_DIV128);
// 	SPI.setDataMode(MCP79510_SPI_DATAMODE);
	MCP79510_disable();

	MCP79510_writeRegister(0x01,0x80);
	MCP79510_writeRegister(0x07,0x12);
	Serial.print("OK\n");
// 	0x12
// 	0x01
// 	0x80
	get_fw_time();
	
}



mcp_timestamp_t MCP79510_getTime()
{
#define rtc_to_dec(y,m)		(((y>>4)&m)*10+(y&0xF))
	RTC_Time.s  = rtc_to_dec( MCP79510_readRegister(0x01) , 0x3 );
	RTC_Time.mi = rtc_to_dec( MCP79510_readRegister(0x02) , 0x3 );
	RTC_Time.h  = rtc_to_dec( MCP79510_readRegister(0x03) , 0x1 );

	RTC_Time.d  = rtc_to_dec( MCP79510_readRegister(0x05) , 0x3 );
	RTC_Time.mo = rtc_to_dec( MCP79510_readRegister(0x06) , 0x1 );
	RTC_Time.y  = rtc_to_dec( MCP79510_readRegister(0x07) , 0xF ) + 2000;

	return RTC_Time;
}


mcp_timestamp_t get_fw_time()
{
#define rtc_reg_format(y)		( ((y/10)<<0x4) | ( (y-((y/10)*10)) ) )

	char month[4] = {0};
	
	char date[32] = {0};
	char time[32] = {0};
	
	strcpy(date,__DATE__);
	strcpy(time,__TIME__);
	
	month[0] = date[0];
	month[1] = date[1];
	month[2] = date[2];
	FirmwareUploadTime.mo = month_to_val(month);
	
	int n = 5;
	FirmwareUploadTime.d = date[n++] - 0x30;
	
	if(date[n] != ' '){
		FirmwareUploadTime.d = FirmwareUploadTime.d*10 + (date[n]-0x30);
		n++;
	}
	n++;
	FirmwareUploadTime.y = (date[n++]-0x30)*1000;
	FirmwareUploadTime.y += (date[n++]-0x30)*100;
	FirmwareUploadTime.y += (date[n++]-0x30)*10;
	FirmwareUploadTime.y += (date[n++]-0x30)*1;
	
	FirmwareUploadTime.h  = (time[0]-0x30)*10 + (time[1]-0x30);
	FirmwareUploadTime.mi = (time[3]-0x30)*10 + (time[4]-0x30);
	FirmwareUploadTime.s  = (time[6]-0x30)*10 + (time[7]-0x30);

	FirmwareUploadTime.unix = (((FirmwareUploadTime.y-1970)*31556926)+((FirmwareUploadTime.mo-1)*2629743)+((FirmwareUploadTime.d-1)*86400)+(FirmwareUploadTime.h*3600)+(FirmwareUploadTime.mi*60)+(FirmwareUploadTime.s));
	
	unsigned char xs = (rtc_reg_format(FirmwareUploadTime.s) | 0x80);
	printf("Sec:  %d -> %02X\n",FirmwareUploadTime.s,xs);


	unsigned char xmi = rtc_reg_format(FirmwareUploadTime.mi);
	printf("Min:  %d -> %02X\n",FirmwareUploadTime.mi,xmi);


	unsigned char xh  = rtc_reg_format(FirmwareUploadTime.h);
	printf("Hour:  %d -> %02X\n",FirmwareUploadTime.h,xh);


	unsigned char xd  = rtc_reg_format(FirmwareUploadTime.d);
	printf("Day:  %d -> %02X\n",FirmwareUploadTime.d,xd);


	unsigned char xmo = rtc_reg_format(FirmwareUploadTime.mo);
	printf("Month:  %d -> %02X\n",FirmwareUploadTime.mo,xmo);

	printf("Year:  %d -> %02X\n",2015,0x15);


	printf("Writing...\n");

	MCP79510_writeRegister(0x01, xs  );
	MCP79510_writeRegister(0x02, xmi );
	MCP79510_writeRegister(0x03, xh  );

	MCP79510_writeRegister(0x05, xd  );
	MCP79510_writeRegister(0x06, xmo );
	MCP79510_writeRegister(0x07, 0x15  );

	printf("RTC Clock updated with firmware timestamp\n");
	
	return FirmwareUploadTime;
}





unsigned char MCP79510_readRegister(unsigned char addr)
{				
//	MCP79510_disable();
//	SPI.begin();
//	SPI.setFrequency(MCP79510_FREQ);			//SPI.setClockDivider(SPI_CLOCK_DIV128);
//	SPI.setDataMode(MCP79510_SPI_DATAMODE);
	MCP79510_disable();


  	MCP79510_enable();
  //	SPI.beginTransaction(SPISettings(MCP79510_FREQ, MSBFIRST, MCP79510_SPI_DATAMODE));
		SPI_transfer_byte(0x13,SPIMODE);
		SPI_transfer_byte(addr,SPIMODE);
		unsigned char b = SPI_transfer_byte(0x00,SPIMODE);
//	SPI.endTransaction();
	MCP79510_disable();
	
  	return b;
}


short MCP79510_writeRegister( unsigned char addr , unsigned char Data )
{
	MCP79510_enable();
//	SPI.beginTransaction(SPISettings(MCP79510_FREQ, MSBFIRST, MCP79510_SPI_DATAMODE));
		SPI_transfer_byte(0x12,SPIMODE);
		SPI_transfer_byte(addr,SPIMODE);
		SPI_transfer_byte(Data,SPIMODE);
//	SPI.endTransaction();
  	MCP79510_disable();
  	
  	MCP79510_enable();
  	MCP79510_disable();
  	
  	delay(15);
  	
  	MCP79510_enable();
		SPI_transfer_byte(0x13,SPIMODE);
		SPI_transfer_byte(addr,SPIMODE);
		unsigned char b = SPI_transfer_byte(0x00,SPIMODE);
	MCP79510_disable();
	
	
	if(b != Data){
		printf("WRITE FAILED!!\n");
		//printf("<br><br>WRITE FAILED (expected 0x%02X, read 0x%02X)<br><br>",Data,b);
		return -1;
	}
	else{
		printf("Write to address %02X (%02X) successful!\n",addr,Data);
		return 1;
	}
  	
  	return 1;
}
