#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
//#include <mach/gpio_sf02.h>
//#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of_gpio.h>

#define DRIVER_NAME "ds28e10"
#define EdbgOutputDebugString printk
//#define DEBUG printk
#define DEBUG(...)

//#define	SETGPG11_OUTPUT	(rGPGCON |= (1<<22))
//#define	SETGPG11_OUTPUT2	(rGPGCON &= ~(1<<23))
//#define	SETGPG11_INPUT	(rGPGCON &= ~(3<<22))
//#define	WR1_GPG11		ioctl(fd,0,1)
//#define	WR0_GPG11		ioctl(fd,0,0)

//#define	WR1_VOLT		do {__gpio_as_output(GPIO_VOLT); __gpio_set_pin(GPIO_VOLT);} while(0)
//#define	WR0_VOLT	        do {__gpio_as_output(GPIO_VOLT); __gpio_clear_pin(GPIO_VOLT);} while(0)

//+++MQ
#define MXS_GPIO_NR(bank, nr)	((bank) * 32 + (nr))
//+++MQ

#define DS28E10_IO  MXS_GPIO_NR(1, 2)
static int gpio_io;
//#define PAD_PULL_UP_REGX   PAD_PULL_UP_REG0
//#define BABBAGE_DS28E10_IO ((GPIOC_bank_bit0_15(7)<<16) |GPIOC_bit_bit0_15(7))
//#define BABBAGE_DS28E10_IO ((GPIOA_bank_bit0_27(22)<<16) |GPIOA_bit_bit0_27(22))
//PAD_PULL_UP_REG2
static int gpio_io;

static void gpio_output(int value)
{
	gpio_direction_output(gpio_io, value);
}

static int gpio_input(void)
{
	gpio_direction_input(gpio_io);
	return gpio_get_value(gpio_io);
}

unsigned char SHAVM_Message[64];
unsigned char	OW_RomID[8];
unsigned char	OW_Buffer;
unsigned char CRC8;
unsigned int CRC16;

//define return constant for authenticate_DS28E10 and Write Memory subroutines
#define No_Device -2;
#define CRC_Error  -1;
#define UnMatch_MAC  0;
#define Match_MAC  1
#define Write_OK 2
#define Write_Failed 3


//define 64-bit secret and 256-bit extension secret attrib, unique=1, same secret=0
#define BasicSecretOption 1
#define ExtensionSecretOption 1 





// Hash buffer, in wrong order for Dallas MAC
unsigned long  SHAVM_Hash[5];
// MAC buffer, in right order for Dallas MAC
unsigned char SHAVM_MAC[20];

// Temp vars for SHA calculation
static unsigned long SHAVM_MTword[80];
static long SHAVM_Temp;
static int SHAVM_cnt;

static long SHAVM_KTN[4];

unsigned char Entropy;

//define extension secret

unsigned char     ExtensionSecret[28] =    //OTP memory data 0x00~0x1B can be as extension secret while read-protected is enabled
{0x00,0x00,0x00,0x00,  //OTP data in address 0x0000
0x00,0x00,0x00,0x00,   //OTP data in address 0x0004
0x00,0x00,0x00,0x00,   //OTP data in address 0x0008
0x00,0x00,0x00,0x00,   //OTP data in address 0x000C
0x4c,0x76,0x34,0x1b,   //OTP data in address 0x0010 
0x35,0x53,0x4a,0xad,   //OTP data in address 0x0014
0x30,0x9d,0x6e,0x3b};   //OTP data in address 0x0018


//define basic 64-bit secret for DS28E10
//unsigned char	DeviceSecret[8] = {0x12,0x29,0xfd,0x23,0x43,0x22,0x44,0x52};
//#if 0
unsigned char   DeviceSecret1_android[8] = {0x20,0x12,0x41,0xd0,0x40,0x22,0x41,0x51};
//unsigned char	DeviceSecret3[8] = {0x12,0x29,0xfd,0x23,0x43,0x22,0x44,0x52};
unsigned char   DeviceSecret2_android[8] = {0x11,0x04,0x48,0x0a,0x1a,0x02,0x32,0x14};
//unsigned char	DeviceSecret4[8] = {0x12,0x29,0xfd,0x23,0x43,0x22,0x44,0x52};
//unsigned char   DeviceSecret[8]; //  = {0x31,0x16,0x89,0xda,0x5a,0x24,0x73,0x65};
//#else

unsigned char   DeviceSecret1_ubuntu[8] = {0x20,0x12,0x41,0xc9,0x40,0x22,0x41,0x51};
//unsigned char   DeviceSecret3[8] = {0x12,0x29,0xfd,0x23,0x43,0x22,0x44,0x52};
unsigned char   DeviceSecret2_ubuntu[8] = {0x22,0x07,0x37,0x00,0x08,0x14,0x21,0x03};
//unsigned char   DeviceSecret4[8] = {0x12,0x29,0xfd,0x23,0x43,0x22,0x44,0x52};
unsigned char   DeviceSecret[8]; //  = {0x42,0x19,0x78,0xc9,0x48,0x36,0x62,0x54};
//#endif

//define for opening I/O
//int fd;
unsigned int get;

//#define delay_ow udelay

//+++MQ
static void getInfo(int id) {
	int i;

	for(i=0; i<8; i++)
	{
		if (id == 0) {
			DeviceSecret[i] = DeviceSecret1_android[i] + DeviceSecret2_android[i];
		} else if (id == 1) {
			DeviceSecret[i] = DeviceSecret1_ubuntu[i] + DeviceSecret2_ubuntu[i];
		}
	}
}
//+++MQ

static void delay_ow(int len)
{
//	len = len*10;
//	udelay(len>>2);
unsigned long flags ;
	local_irq_save(flags);
	while( len > 50 )
	{
		udelay(50);
		len = len-50;
	}
	udelay(len);
	
	local_irq_restore(flags);
}


// Calling this routine takes about 1us
/*void delay_ow(int us)
{
	volatile k1;
	for(us*=10;us > 0;us--)
	{
		k1++;
		k1=0;

		//EdbgOutputDebugString ("");
										//EdbgOutputDebugString ("k1=%Xh\r\n", k1);		
	}
}*/

// one wire reset
unsigned char ow_reset(void)
{
//	volatile int i;
	//unsigned char presence;
//	SETGPG11_OUTPUT; // set GPG11 to output
//	SETGPG11_OUTPUT2;
	//WR0_GPG11; //pull GPG11 line low
	gpio_output(0);
	delay_ow(750/*480*/); // leave it low for 480us
//	set_gpio_val(GPIOA_bank_bit0_27(22)<<16, GPIOA_bit_bit0_27(22), 0);
//	set_gpio_mode(GPIOA_bank_bit0_27(22)<<16, GPIOA_bit_bit0_27(22), GPIO_OUTPUT_MODE);
//	for( i=0; i<30000; i++ );
//	delay_ow(280);
	//WR1_GPG11; // allow line to return high
	gpio_output(1); 
//	SETGPG11_INPUT; // set GPG11 to input
	delay_ow(70); // wait for presence 
	//presence = ioctl(fd,1,&get); // get presence signal
//	set_gpio_mode(GPIOA_bank_bit0_27(22)<<16, GPIOA_bit_bit0_27(22), GPIO_INPUT_MODE);
//	gpio_direction_input(BABBAGE_DS28E10_IO);
//	delay_ow(70);	// Frank Added
//	get = get_gpio_val(GPIOA_bank_bit0_27(22)<<16, GPIOA_bit_bit0_27(22));
	get = gpio_input();
	delay_ow(430); // wait for end of timeslot
//	delay_ow(40);
//	printk("DS28E01 ow_reset: ret=%d\n", get);
	return(get); // presence signal returned
} // presence = 0, no part = 1

// one wire read bit
unsigned char read_bit(void)
{
	//unsigned char vamm;
//	SETGPG11_OUTPUT;
//	SETGPG11_OUTPUT2;
	//WR0_GPG11;
	gpio_output(0);
	delay_ow(5);
	//WR1_GPG11;
	gpio_output(1);
//	SETGPG11_INPUT;
	delay_ow(15); // delay 15μs from start of timeslot
	//vamm = ioctl(fd,1,&get);
	get = gpio_input();
	return(get); // return value of GPG11 line
}


// one write write bit
void write_bit(char bitval)
{
	//WR0_GPG11;
	gpio_output(0);
	delay_ow(5);
	if(bitval==1)
		gpio_output(1);
	delay_ow(70);// hold value for remainder of timeslot
	//WR1_GPG11;
	gpio_output(1);
	delay_ow(6);	// must add this delay because the cup frequence is 203MHZ.
}


// one wire read byte
unsigned char read_byte(void)
{
	unsigned char i;
	unsigned char value = 0;
	for (i = 0; i < 8; i++)
	{
		if(read_bit()) value |= 0x01<<i; // reads byte in, one byte at a time and then shifts it left
			delay_ow(70); // wait for rest of timeslot
	}
	return(value);
}

// one wire write byte
void write_byte(char val)
{
	unsigned char i;
	unsigned char temp; 
//	SETGPG11_OUTPUT;
//	SETGPG11_OUTPUT2;
	for (i = 0; i < 8; i++) // writes byte, one bit at a time
	{
		temp = val>>i; // shifts val right ‘i’ spaces
		temp &= 0x01; // copy that bit to temp
		write_bit(temp); // write bit in temp into
	}
	delay_ow(10);

}



/*--------------------------------------------------------------------------
 * Update the Dallas Semiconductor One Wire CRC (CRC8) from the global
 * variable CRC8 and the argument.  Return the updated CRC8.
 */
unsigned char dscrc_tab[] = {
        0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
      157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
       35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
      190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
       70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
      219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
      101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
      248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
      140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
       17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
      175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
       50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
      202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
       87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
      233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
      116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};
//--------
unsigned char dowcrc(unsigned char x)
{
   CRC8 = dscrc_tab[CRC8 ^ x];
   return CRC8;
}


/*--------------------------------------------------------------------------
 * Calculate a new CRC16 from the input data shorteger.  Return the current
 * CRC16 and also update the global variable CRC16.
 */
static short oddparity[16] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

unsigned short docrc16(unsigned short data)
{
   data = (data ^ (CRC16 & 0xff)) & 0xff;
   CRC16 >>= 8;

   if (oddparity[data & 0xf] ^ oddparity[data >> 4])
     CRC16 ^= 0xc001;

   data <<= 6;
   CRC16   ^= data;
   data <<= 1;
   CRC16   ^= data;

   return CRC16;
}


//--------------------------------------------------------------------------
//Issue a power-on reset sequence to DS28E10
// Returns: TRUE (1) success
//          FALSE (0) failed
//

static int PowerOnResetDS28E10(void)
{
     short i,cnt=0;
     unsigned char pbuf[20];
     int ret;

//     printk("PowerOnResetDS28E10: 0\n");
     ret = ow_reset();
     
     printk("PowerOnResetDS28E10: reset ret=%d\n", ret);

//     return false; // Frank
     write_byte(0xCC);
     cnt=0;
  // construct a packet to send
     pbuf[cnt++] = 0x55; // write memory command
     pbuf[cnt++] = 0x00; // address LSB
     pbuf[cnt++] = 0x00; 		   // address MSB

// data to be written
     for (i = 0; i < 4; i++) pbuf[cnt++] = 0xff;
// perform the block writing 
     for(i=0;i<cnt;i++) write_byte(pbuf[i]);
// for reading crc bytes from DS28E10
     pbuf[cnt++] = read_byte();
     pbuf[cnt++] = read_byte();
     printk("DS28E10: pbuf[cnt-2]=0x%x, pbuf[cnt-1]=0x%x\n",
		     pbuf[cnt-2], pbuf[cnt-1]);
// check CRC
     CRC16 = 0;
     for (i = 0; i < cnt; i++) docrc16(pbuf[i]);
// return result of inverted CRC
     if(CRC16 != 0xB001) 
     {
	     printk(KERN_INFO "HAHAH0: CRC error at command 0xCC, cnt=%d, %02x, %02x", cnt, pbuf[cnt-2], pbuf[cnt-1]);
	     return false; //not 0 because that the calculating result is CRC16 and the reading result is inverted CRC16
     }
//     for(i=0;i<100;i++)  delay_ow(1100);       // wait for 110 ms
     delay_ow(100);
     write_byte(0x00);     // clock 0x00 byte as required
     delay_ow(100);
     ow_reset();

//     printk(KERN_INFO "DS28E10: CRC okay at command 0xCC");
     return true;
}
//----------------------------------------------------------------------
// computes a SHA given the 64 byte MT digest buffer.  The resulting 5
// long values are stored in the long array, hash.
//
// 'SHAVM_Message' - buffer containing the message digest
// 'SHAVM_Hash'    - result buffer
// 'SHAVM_MAC'     - result buffer, in order for Dallas part
//
static void SHAVM_Compute(void)
{
   SHAVM_KTN[0]=(long)0x5a827999;
   SHAVM_KTN[1]=(long)0x6ed9eba1;
   SHAVM_KTN[2]=(long)0x8f1bbcdc;
   SHAVM_KTN[3]=(long)0xca62c1d6;   
   for(SHAVM_cnt=0; SHAVM_cnt<16; SHAVM_cnt++)
   {
      SHAVM_MTword[SHAVM_cnt]
         = (((long)SHAVM_Message[SHAVM_cnt*4]&0x00FF) << 24L)
         | (((long)SHAVM_Message[SHAVM_cnt*4+1]&0x00FF) << 16L)
         | (((long)SHAVM_Message[SHAVM_cnt*4+2]&0x00FF) << 8L)
         |  ((long)SHAVM_Message[SHAVM_cnt*4+3]&0x00FF);
   }

   for(; SHAVM_cnt<80; SHAVM_cnt++)
   {
      SHAVM_Temp
         = SHAVM_MTword[SHAVM_cnt-3]  ^ SHAVM_MTword[SHAVM_cnt-8]
         ^ SHAVM_MTword[SHAVM_cnt-14] ^ SHAVM_MTword[SHAVM_cnt-16];
      SHAVM_MTword[SHAVM_cnt]
         = ((SHAVM_Temp << 1) & 0xFFFFFFFE)
         | ((SHAVM_Temp >> 31) & 0x00000001);
   }

   SHAVM_Hash[0] = 0x67452301;
   SHAVM_Hash[1] = 0xEFCDAB89;
   SHAVM_Hash[2] = 0x98BADCFE;
   SHAVM_Hash[3] = 0x10325476;
   SHAVM_Hash[4] = 0xC3D2E1F0;

   for(SHAVM_cnt=0; SHAVM_cnt<80; SHAVM_cnt++)
   {
      SHAVM_Temp
         = ((SHAVM_Hash[0] << 5) & 0xFFFFFFE0)
         | ((SHAVM_Hash[0] >> 27) & 0x0000001F);
      if(SHAVM_cnt<20)
         SHAVM_Temp += ((SHAVM_Hash[1]&SHAVM_Hash[2])|((~SHAVM_Hash[1])&SHAVM_Hash[3]));
      else if(SHAVM_cnt<40)
         SHAVM_Temp += (SHAVM_Hash[1]^SHAVM_Hash[2]^SHAVM_Hash[3]);
      else if(SHAVM_cnt<60)
         SHAVM_Temp += ((SHAVM_Hash[1]&SHAVM_Hash[2])
                       |(SHAVM_Hash[1]&SHAVM_Hash[3])
                       |(SHAVM_Hash[2]&SHAVM_Hash[3]));
      else
         SHAVM_Temp += (SHAVM_Hash[1]^SHAVM_Hash[2]^SHAVM_Hash[3]);
      SHAVM_Temp += SHAVM_Hash[4] + SHAVM_KTN[SHAVM_cnt/20]
                  + SHAVM_MTword[SHAVM_cnt];
      SHAVM_Hash[4] = SHAVM_Hash[3];
      SHAVM_Hash[3] = SHAVM_Hash[2];
      SHAVM_Hash[2]
         = ((SHAVM_Hash[1] << 30) & 0xC0000000)
         | ((SHAVM_Hash[1] >> 2) & 0x3FFFFFFF);
      SHAVM_Hash[1] = SHAVM_Hash[0];
      SHAVM_Hash[0] = SHAVM_Temp;
   }

   //iButtons use LSB first, so we have to turn
   //the result around a little bit.  Instead of
   //result A-B-C-D-E, our result is E-D-C-B-A,
   //where each letter represents four bytes of
   //the result.
   for(SHAVM_cnt=0; SHAVM_cnt<5; SHAVM_cnt++)
   {
      SHAVM_Temp = SHAVM_Hash[4-SHAVM_cnt];
      SHAVM_MAC[((SHAVM_cnt)*4)+0] = (unsigned char)SHAVM_Temp;
      SHAVM_Temp >>= 8;
      SHAVM_MAC[((SHAVM_cnt)*4)+1] = (unsigned char)SHAVM_Temp;
      SHAVM_Temp >>= 8;
      SHAVM_MAC[((SHAVM_cnt)*4)+2] = (unsigned char)SHAVM_Temp;
      SHAVM_Temp >>= 8;
      SHAVM_MAC[((SHAVM_cnt)*4)+3] = (unsigned char)SHAVM_Temp;
   }
   
}

//Compute unique basic secret
//input parameter: 64-bit unique ROM ID
//the result in DeviceBasicSecret
static void ComputeBasicSecret_ds28e10(unsigned char *DeviceBasicSecret)
{
	printk("%s<<<<<<<\n", __func__);
   if( BasicSecretOption )
   {
//calculate unique 64-bit secret in the DS28E10-100
  	memset(&SHAVM_Message[4], 0x00, 32);
   	memcpy(SHAVM_Message, DeviceSecret, 4);
   	memset(&SHAVM_Message[36], 0xff, 4);
        SHAVM_Message[40]=OW_RomID[0]&0x3f;
  	memcpy(&SHAVM_Message[41], &OW_RomID[1], 7);
    	memcpy(&SHAVM_Message[48], &DeviceSecret[4], 4);
        memset(&SHAVM_Message[52],0xff, 3);
   	SHAVM_Message[55] = 0x80;
   	memset(&SHAVM_Message[56], 0x00, 6);
   	SHAVM_Message[62] = 0x01;
   	SHAVM_Message[63] = 0xB8;

        SHAVM_Compute();     //unique basic secret now in SHAVM_MAC[]
	memcpy(DeviceBasicSecret, SHAVM_MAC, 8);   //now unique 64-bit basic secret in DeviceBasicSecret
     }
     else
     {
       memcpy(DeviceBasicSecret, DeviceSecret, 8);     //load basic 64-bit secret into DeviceBasicSecret
     }

}
//read the 64-bit ROM ID of DS28E10-100
//Output ROM ID in OW_RomID
static int ReadRomID(void)
{
   short j1;
   int i;
// read rom id
	if ( (ow_reset())!=0 ) return false;
    	write_byte(0x33);
   	for(j1 = 0;j1 < 8;j1++)
	{
       OW_RomID[j1] = read_byte();
    }
   //CRC8 check if reading ROM ID is right
    CRC8=0;
    for(i=0;i<8;i++)
    	dowcrc(OW_RomID[i]); //check if reading ROM ID is right by CRC8 result
    if(CRC8!=0) return false;
    else return true; 

}
#if 0
//--------------------------------------------------------------------------
// Load the secret to DS28E10
//
// Returns: TRUE (1) success
//          FALSE (0) failed
//
int LoadSecret(uchar *buf)
{
     short i,cnt=0;
     uchar ch, pbuf[20];
     ow_reset();
     write_byte(0xCC);
     cnt=0;
// construct a packet to send
     pbuf[cnt++] = 0x5a; // write seret command

// data to write
     for (i = 0; i < 8; i++) pbuf[cnt++] = (uchar)(buf[i]);
// perform the block writing and reading 2 bytes of CRC16
     for(i=0;i<cnt;i++) write_byte(pbuf[i]);
// for reading crc bytes
     pbuf[cnt++] = read_byte();
     pbuf[cnt++] = read_byte();

// check CRC
     CRC16 = 0;
     for (i = 0; i < cnt; i++) docrc16(pbuf[i]);

// return result of inverted CRC
     if(CRC16 != 0xB001) return CRC_Error;  //not 0,0xB001 because that the calculating result is CRC16 and the reading result is inverted CRC16
//programming the first 4 byte of secret
     write_byte(0x00);     // clock 0x00 byte as required
     WR1_VOLT;		    //generating 6.5V programming voltage 产生第一次编程电压!
     for(i=0;i<100;i++)  delay_ow(1100);       // wait for 110 ms
     WR0_VOLT;    	    //return to 3.3V operating voltage
     delay_ow(1500);	    //wait voltage down from 6.5V to 3.3V
//programming the last 4 byte of secret
     write_byte(0x00);     // clock 0x00 byte as required
     WR1_VOLT;		    //generating 6.5V programming voltage 产生第二次编程电压!
     for(i=0;i<100;i++)  delay_ow(1100);       // wait for 110 ms
     WR0_VOLT;    	    //return to 3.3V operating voltage
     delay_ow(1500);	    //wait voltage down from 6.5V to 3.3V
     write_byte(0x00);     // clock 0x00 byte as required
     ow_reset();
     return true;
}

//--------------------------------------------------------------------------
// Program the EPROM memory in 4 byte length
//
// Returns: TRUE (1) success
//          FALSE (0) failed
//

int _fastcall Write4Byte(short TargetAddress, uchar *buf)
{
     short i,cnt=0,flag;
     uchar ch, pbuf[20];
     ow_reset();
     write_byte(0xCC);
     cnt=0;
  // construct a packet to send
     pbuf[cnt++] = 0x55; // write memory command
     pbuf[cnt++] = TargetAddress%256; // address LSB
     pbuf[cnt++] = 0x00; 		   // address MSB

// data to be written
     for (i = 0; i < 4; i++) pbuf[cnt++] = (uchar)(buf[i]);
// perform the block writing 
     for(i=0;i<cnt;i++) write_byte(pbuf[i]);
// for reading crc bytes from DS28E10
     pbuf[cnt++] = read_byte();
     pbuf[cnt++] = read_byte();
// check CRC
     CRC16 = 0;
     for (i = 0; i < cnt; i++) docrc16(pbuf[i]);
// return result of inverted CRC
     if(CRC16 != 0xB001) return false; //not 0 because that the calculating result is CRC16 and the reading result is inverted CRC16
//programming the 4-byte data
     WR1_VOLT;		    //generating 6.5V programming voltage 产生编程电压!
     for(i=0;i<100;i++)  delay_ow(1100);       // wait for 110 ms
     WR0_VOLT;    	    //return to 3.3V operating voltage
     delay_ow(1500);	    //wait voltage down from 6.5V to 3.3V
     write_byte(0x00);     // clock 0x00 byte as required
     ow_reset();
     return true;
}

#endif

//Read DS28E10 page memory data
//Input parameter: page number, register page number=4
//Output page data in GetPageData
static int ReadMemoryPage(unsigned char PageNumber, char *GetPageData)
{
      short i;
      unsigned char pbuf[40], cnt; 
      if(PageNumber<4)
      {
        ow_reset();
    	write_byte(0xCC);
        cnt=0;
    	pbuf[cnt++]=0xA5;          //Function Command, Read authentication memory
    	pbuf[cnt++]=PageNumber<<5;
    	pbuf[cnt++]=PageNumber>>3;
        for(i=0;i<3;i++) write_byte(pbuf[i]);

    	for(i = 0;i < 31;i++)
        {
          pbuf[cnt++] = read_byte();
        }

//copy the EEPROM page data to PageData
        memcpy(GetPageData, &pbuf[3], 28);
// run the CRC over this part
        CRC16 = 0;
        for (i = 0; i < cnt; i++) docrc16(pbuf[i]);
        if( CRC16 != 0xB001) return false;  //not 0 because that the calculating result is CRC16 and the reading result is inverted CRC16
	else return true;
       }
       else   //read the register page
       {
         ow_reset();
    	 write_byte(0xCC);
         write_byte(0xf0);     // read memory command
         write_byte(0x1C );     // write target address LSB, register page beginning address=0x0088
         write_byte(0x00 );     // write target address MSB
         for(i=0x1C;i<0x23;i++) GetPageData[i-0x1C]=read_byte();
//repeat reading to chech if error in reading data        
	 ow_reset();
    	 write_byte(0xCC);
         write_byte(0xf0);     // read memory command
         write_byte(0x1C );     // write target address LSB, register page beginning address=0x0088
         write_byte(0x00 );     // write target address MSB
         for(i=0x1C;i<0x23;i++) if( GetPageData[i-0x1C]!=read_byte() ) break;
  	 if(i==0x23) return true;
	 else return false;

       }
}



//authenticate DS28E10 based on Authentication Page number
//input parameter is random number in Challenge, Unique Secret enbled by UniqueSecret, Authentication page number(0,1,2)
//Output code to indicate the result in No_Device, CRC_Error, UnMatch_MAC and Match_MAC
static int Authenticate_DS28E10_By_64bitSecret(unsigned char *Challenge, unsigned char UniqueSecret,unsigned char AcctPageNum)
{
   int i,j1;

   unsigned char pbuf[40], PageData[32], cnt;
	printk("%s<<<<<<<\n", __func__);

// read rom id
	if ( (ow_reset())!=0 ) return No_Device;
    	write_byte(0x33);
   for(j1 = 0;j1 < 8;j1++)
   {
      OW_RomID[j1] = read_byte();
   }
   //CRC8 check if reading ROM ID is right
    CRC8=0;
    for(i=0;i<8;i++)
	{
    	dowcrc(OW_RomID[i]); //check if reading ROM ID is right by CRC8 result
//		DEBUG("OW_RomID:%x \n", OW_RomID[i]);
	}
	if(CRC8!=0) { DEBUG("Reading ROM ID error\n"); return CRC_Error; }

// identify the 64-bit basic secret in the DS28E10 and load into basic 64-bit secret
   if( UniqueSecret==true )
   {
//calculate unique 64-bit secret in the DS28E10
  		memset(&SHAVM_Message[4], 0x00, 32);
   		memcpy(SHAVM_Message, DeviceSecret, 4);
   		memset(&SHAVM_Message[36], 0xff, 4);
        SHAVM_Message[40]=OW_RomID[0]&0x3f;
  		memcpy(&SHAVM_Message[41], &OW_RomID[1], 7);
    	memcpy(&SHAVM_Message[48], &DeviceSecret[4], 4);
        memset(&SHAVM_Message[52],0xff, 3);
   		SHAVM_Message[55] = 0x80;
   		memset(&SHAVM_Message[56], 0x00, 6);
   		SHAVM_Message[62] = 0x01;
   		SHAVM_Message[63] = 0xB8;

        SHAVM_Compute();     //unique secret now in SHAVM_MAC[]
     }
     else
     {
       memcpy(SHAVM_MAC, DeviceSecret, 8);     //load basic 64-bit secret into SHAVM[]
     }

//read MAC from the DS28E10
/****************************************************************/
			//write Challenge
    	ow_reset();                 //reset 1-wire bus and detect response pulse
    	write_byte(0xCC);          //SKIP ROM command
    	write_byte(0x0f);          //Function Command, Write challenge to scratchpad
        for(i=0;i<12;i++) write_byte(Challenge[i]);  //write 12-byte challenge
        for(i=0;i<12;i++) if( read_byte()!=Challenge[i]) break;
        if(i!=12) { DEBUG("Wrinting Scracthpad error\n"); return CRC_Error; } 

/****************************************************************/
			//read page_data and authMAC
        ow_reset();
    	write_byte(0xCC);
        cnt=0;
    	pbuf[cnt++]=0xA5;          //Function Command, Read authentication memory
    	pbuf[cnt++]=AcctPageNum<<5;
    	pbuf[cnt++]=AcctPageNum>>3;
        for(i=0;i<3;i++) write_byte(pbuf[i]);

    	for(i = 0;i < 31;i++)
        {
          pbuf[cnt++] = read_byte();
        }

//copy the EEPROM page data to PageData
        memcpy(PageData, &pbuf[3], 28);
//check if there is OTP data in read-protected mode, if yes, then displace its data with the real data in ExtensionSecret[]
        for(i=0;i<7;i++)
        {
           if(PageData[i*4]==0xff && PageData[i*4+1]==0xff && PageData[i*4+2]==0xff && PageData[i*4+3]==0xff) 
                           memcpy(&PageData[i*4],&ExtensionSecret[i*4], 4);
        }
          
     
// run the CRC over this part
        CRC16 = 0;
        for (i = 0; i < cnt; i++) docrc16(pbuf[i]);
        if( CRC16 != 0xB001) { DEBUG("Reading  page data error\n"); return CRC_Error; } //not 0 because that the calculating result is CRC16 and the reading result is inverted CRC16

		delay_ow(2000);      //waiting for finishing SHA-1 algorithm
		delay_ow(1000);      //waiting for finishing SHA-1 algorithm
        cnt=0;
		for(i = 0;i < 22;i++)
        {
          pbuf[cnt++] = read_byte();
        }
// run the CRC over this part MAC
        CRC16 = 0;
        for (i = 0; i < cnt; i++) docrc16(pbuf[i]);
        if( CRC16 != 0xB001) { DEBUG("Reading  MAC data error\n"); return CRC_Error; } //not 0 because that the calculating result is CRC16 and the reading result is inverted CRC16

//calculate the corresponding MAC by the host, device secret reserved in SHAVM_MAC[]


  		memcpy(&SHAVM_Message[4], PageData, 28);
        memcpy(&SHAVM_Message[32], &Challenge[8], 4);
        memcpy(&SHAVM_Message[36], Challenge, 4);
   		SHAVM_Message[40] =Challenge[7];
  		memcpy(&SHAVM_Message[41], OW_RomID, 7);
   		memcpy(&SHAVM_Message[52], &Challenge[4], 3);
   		memcpy(SHAVM_Message, SHAVM_MAC, 4);
   		memcpy(&SHAVM_Message[48], &SHAVM_MAC[4], 4);
   		SHAVM_Message[55] = 0x80;
   		memset(&SHAVM_Message[56], 0x00, 6);
   		SHAVM_Message[62] = 0x01;
   		SHAVM_Message[63] = 0xB8;
        SHAVM_Compute();     //MAC generated based on authenticated page now in SHAVM_MAC[]

//Compare calculated MAC with the MAC from the DS28E10-100
        for(i=0;i<20;i++){ if( SHAVM_MAC[i]!=pbuf[i] )  break;}
        if( i==20 ){ return Match_MAC;}
        else return UnMatch_MAC;
}
static int ds28e10_probe(struct platform_device* device)
{
	int i, ret;
	struct resource* res;
	//unsigned char	Scratchpad_data[13];
	unsigned char Challenge[12] = {0x12,0x24,0x4d,0x2a,0x73,0x12,0x24,0x4d,0x2a,0x73,0x24,0x56};
	DEBUG("\n\n\ndevice name: %s \n", device->name);
	printk("MQ=1==GPIO_2:%x\n", __raw_readl(ioremap(0x020e0224, 4)));
	printk("MQ=1==GPIO_2 conf:%x\n", __raw_readl(ioremap(0x020e05f4, 4)));
	__raw_writel(0x6008, ioremap(0x020e05f4, 4));
	printk("MQ=2==GPIO_2 conf:%x\n", __raw_readl(ioremap(0x020e05f4, 4)));

  struct device_node *np = device->dev.of_node;
  printk("gpio driver probe is called==========\n");
  gpio_io = of_get_named_gpio(np, "ds28e10_io", 0);
  if (gpio_is_valid(gpio_io)) {
       ret = gpio_request(gpio_io, "DS28E10 IO");
       if (ret) {
               //dev_err(&pdev->dev, "Failed requesting gpio_ro %d\n", gpio_ro);
               printk("gpio_request failed ret = %d\n", ret);
       }
       gpio_direction_output(gpio_io, 0);
  }else{
    printk("gpio is not valid===============WWJ\n");  
  }

	//gpio_request(DS28E10_IO, "DS28E10 IO");
	//gpio_output(0);
	/*
	if (!gpio_request_one(DS28E10_IO, GPIOF_DIR_OUT,
			      "DS28E10 IO")) {
		printk("request gpio 1\n");
	}
	
	printk("request gpio 2\n");
	while (1) {
		gpio_output(1);
		msleep(50);
		gpio_output(0);
		msleep(50);
	}
  */
//	return 0; // Frank Lee added
//	WRITE_CBUS_REG_BITS(0x2018, 0, 29, 1);
//	SET_CBUS_REG_MASK(PAD_PULL_UP_REGX, (1<<29)); 
/*	res = platform_get_resource_byname(device, IORESOURCE_IO, DRIVER_NAME);
	if (!res)
	{
		DEBUG("get resource failed!");
		return 1;
	}
	ret = gpio_request(res->start, DRIVER_NAME);
	if (unlikely(ret)) {
		gpio_free(res->start);
		return 0;
	}
*/
	DEBUG("\n\n--------------probe ds28e10 driver -------------\n");
	for ( i=0; i<300; i++ )
	{
		gpio_output(1);
		msleep(100);

		PowerOnResetDS28E10();    //must execute the subroutine at least one time to gurrantee the DS28E10 to perform power-on reset
//+++MQ
		getInfo(0);
//+++MQ
		if((ret=Authenticate_DS28E10_By_64bitSecret(Challenge, BasicSecretOption, 0))==Match_MAC) 
		{
			EdbgOutputDebugString ("IT is very tumultuous here, let's move forward!\n" 
					/*"Pass the authentication based on Page 0\r\n"*/);
			break;
		}
//+++MQ
		getInfo(1);
		if((ret=Authenticate_DS28E10_By_64bitSecret(Challenge, BasicSecretOption, 0))==Match_MAC) 
		{
			EdbgOutputDebugString ("IT is very tumultuous here, let's move forward!\n" 
					/*"Pass the authentication based on Page 0\r\n"*/);
			break;
		}
//+++MQ
	}
	if( i >= 300)	
	{
//      	EdbgOutputDebugString ("Verify Failed, the ret of Authenticate is %d\r\n",ret);
		panic("keyboard error");
	}

//	gpio_free(res->start);
	return 0;
}

static ssize_t get_mac_id(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
			OW_RomID[0],
			OW_RomID[1],
			OW_RomID[2],
			OW_RomID[3],
			OW_RomID[4],
			OW_RomID[5],
			OW_RomID[6],
			OW_RomID[7]);
}

static DEVICE_ATTR(mac_id, S_IRUGO, get_mac_id, NULL);

struct kobject *mac_id_kobj;

static void mac_id_sys_init(void)
{
	int ret;

	printk(KERN_INFO "mac_id_sys_init: start ********************************\n");
	mac_id_kobj = kobject_create_and_add("mac_id", NULL);
	if( mac_id_kobj == NULL )
	{
		printk(KERN_ERR "mac_id: kobject_create_and_add failed\n");
		return;
	}

	ret = sysfs_create_file(mac_id_kobj, &dev_attr_mac_id.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_version_file failed\n", __func__);
		return;
	}
}

static const struct of_device_id imx_ds28e10_dt_ids[] = {
	{ .compatible = "fsl,imx_ds28e10", },
	{ /* sentinel */ }
};

static struct platform_driver ds28e10_driver =
{
	.probe = ds28e10_probe,
	.driver = 
	{
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = imx_ds28e10_dt_ids,
	}
};

//module_platform_driver(ds28e10_driver);

static int __init ds28e10_init(void)
{
/*
	int i;

	for(i=0; i<8; i++)
	{
		DeviceSecret[i] = DeviceSecret1[i] + DeviceSecret2[i];
	}
*/
//+++MQ
	//getInfo(1);
//+++MQ
	DEBUG("\n\n\ninit ds28e10 driver 2011 Sybay tec %d \n", __LINE__);
	//mac_id_sys_init();
	return platform_driver_register(&ds28e10_driver);	
}

static void __exit ds28e10_exit(void)
{
	DEBUG("exit ds28e10 driver 2011 Sybay tec %d \n", __LINE__);
	platform_driver_unregister(&ds28e10_driver);
}
module_init(ds28e10_init);
module_exit(ds28e10_exit);

MODULE_AUTHOR("jocom@sybaytec.com");
MODULE_DESCRIPTION("ds28e10 Driver");
MODULE_LICENSE("GPL");
