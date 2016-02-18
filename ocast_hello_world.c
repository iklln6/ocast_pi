#include <wiringPi.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>



/*typedef struct{
  int y;
  int mo;
  int d;
  int h;
  int mi;
  int s;
  unsigned long unix;
}timestamp_t;*/

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
 // timestamp_t t;

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









int main (void)
{
	printf("Hello world!\n");

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


void loop()
{
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
	
	int yy = 2016;
	int mm = 2;
	int dd = 18;
	int HH = 8;
	int MM = 26;
	int SS = 42;
	
	sprintf(TimeString,"%d-%d-%d\t%d:%d\t%d\t",yy,mm,dd,HH,MM,SS);
	
	int rdg = 0;	
	ADS1118_update(&rdg);
	float rdg_v = rdg;
    rdg_v *= data_V[chan];
    rdg_v *= 1e6;
    
    printf("%s\t%7.1f uV\n",TimeString,rdg_v);
    
    ofp = fopen("~/Desktop/mytestfile.txt", "w+");          /*creates mytestfile.txt, opens it*/
    fprintf(ofp,"%s\t%7.1f uV\n",TimeString,rdg_v);
    fclose(ofp);
    
	
//	printf("Rdg = %d\n",(int)rdg);


		


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


int dontrun = 0;
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
