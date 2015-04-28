/*
 * Copyright (c) 2013 Nuand LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "sys/alt_stdio.h"
#include "system.h"
#include "altera_avalon_spi.h"
#include "altera_avalon_spi_regs.h"
#include "fifoed_avalon_uart_regs.h"
#include "altera_avalon_jtag_uart_regs.h"
#include "altera_avalon_pio_regs.h"
// 1wire includes
#include "sockit_owm.h"
#include "sockit_owm_regs.h"
#include "ownet.h"
#include "findtype.h"
// general
#include "priv/alt_busy_sleep.h"
#include "priv/alt_file.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include "sys/alt_dev.h"
#include "sys/alt_irq.h"

#include "fifo.h"



#define LMS_READ            0
#define LMS_WRITE           (1<<7)
#define LMS_VERBOSE         1
#define kHz(x)              (x*1000)
#define MHz(x)              (x*1000000)
#define GHz(x)              (x*1000000000)


//when version id is moved to a qsys port these will be removed
#define FPGA_VERSION_ID         0x7777
#define FPGA_VERSION_MAJOR      0
#define FPGA_VERSION_MINOR      1
#define FPGA_VERSION_PATCH      2
#define FPGA_VERSION            (FPGA_VERSION_MAJOR | (FPGA_VERSION_MINOR << 8) | (FPGA_VERSION_PATCH << 16))

#define TIME_TAMER              TIME_TAMER_0_BASE
// Register offsets from the base
#define I2C                 BLADERF_OC_I2C_MASTER_0_BASE
#define OC_I2C_PRESCALER    0
#define OC_I2C_CTRL         2
#define OC_I2C_DATA         3
#define OC_I2C_CMD_STATUS   4

#define SI5338_I2C          (0xE0)
#define OC_I2C_ENABLE       (1<<7)
#define OC_I2C_STA          (1<<7)
#define OC_I2C_STO          (1<<6)
#define OC_I2C_WR           (1<<4)
#define OC_I2C_RD           (1<<5)
#define OC_I2C_TIP          (1<<1)
#define OC_I2C_RXACK        (1<<7)
#define OC_I2C_NACK         (1<<3)

#define DEFAULT_GAIN_CORRECTION 0x1000
#define GAIN_OFFSET 0
#define DEFAULT_PHASE_CORRECTION 0x0000
#define PHASE_OFFSET 16
#define DEFAULT_CORRECTION ( (DEFAULT_PHASE_CORRECTION << PHASE_OFFSET)|  (DEFAULT_GAIN_CORRECTION << GAIN_OFFSET))


#define _1_WIRE_DS2411_FAMILY_CODE (0x01)


void xb_gps_uart_write( uint8_t val ) ;
uint8_t xb_gps_uart_read(  );
void debug(const char* s);



void si5338_complete_transfer( uint8_t check_rxack ) {
    if( (IORD_8DIRECT(I2C, OC_I2C_CMD_STATUS)&OC_I2C_TIP) == 0 ) {
        while( (IORD_8DIRECT(I2C, OC_I2C_CMD_STATUS)&OC_I2C_TIP) == 0 ) { } ;
    }
    while( IORD_8DIRECT(I2C, OC_I2C_CMD_STATUS)&OC_I2C_TIP ) { } ;
    while(  check_rxack && (IORD_8DIRECT(I2C, OC_I2C_CMD_STATUS) & OC_I2C_RXACK)   ) { } ;
}

void si5338_read( uint8_t addr, uint8_t *data ) {

    // Set the address to the Si5338
    IOWR_8DIRECT(I2C, OC_I2C_DATA, SI5338_I2C ) ;
    IOWR_8DIRECT(I2C, OC_I2C_CMD_STATUS, OC_I2C_STA | OC_I2C_WR ) ;
    si5338_complete_transfer( 1 ) ;

    IOWR_8DIRECT(I2C, OC_I2C_DATA, addr ) ;
    IOWR_8DIRECT(I2C, OC_I2C_CMD_STATUS, OC_I2C_WR | OC_I2C_STO ) ;
    si5338_complete_transfer( 1 ) ;

    // Next transfer is a read operation, so '1' in the read/write bit
    IOWR_8DIRECT(I2C, OC_I2C_DATA, SI5338_I2C | 1 ) ;
    IOWR_8DIRECT(I2C, OC_I2C_CMD_STATUS, OC_I2C_STA | OC_I2C_WR ) ;
    si5338_complete_transfer( 1 ) ;

    IOWR_8DIRECT(I2C, OC_I2C_CMD_STATUS, OC_I2C_RD | OC_I2C_NACK | OC_I2C_STO ) ;
    si5338_complete_transfer( 0 ) ;

    *data = IORD_8DIRECT(I2C, OC_I2C_DATA) ;
    return ;
}

void si5338_write( uint8_t addr, uint8_t data ) {

    // Set the address to the Si5338
    IOWR_8DIRECT(I2C, OC_I2C_DATA, SI5338_I2C) ;
    IOWR_8DIRECT(I2C, OC_I2C_CMD_STATUS, OC_I2C_STA | OC_I2C_WR ) ;
    si5338_complete_transfer( 1 ) ;

    IOWR_8DIRECT(I2C, OC_I2C_DATA, addr) ;
    IOWR_8DIRECT(I2C, OC_I2C_CMD_STATUS, OC_I2C_CMD_STATUS | OC_I2C_WR ) ;
    si5338_complete_transfer( 1 ) ;

    IOWR_8DIRECT(I2C, OC_I2C_DATA, data ) ;
    IOWR_8DIRECT(I2C, OC_I2C_CMD_STATUS, OC_I2C_WR | OC_I2C_STO ) ;
    si5338_complete_transfer( 0 ) ;

    return ;
}

// Trim DAC write
void dac_write( uint16_t val ) {
//    alt_printf( "DAC Writing: %x\n", val ) ;
    uint8_t data[3]; //cmd, data1, data2

    //DAC_SWB
    data[0] = 0x28, data[1] = 0, data[2] = 0 ;
    alt_avalon_spi_command( SPI_1_BASE, 0, 3, data, 0, 0, 0 ) ;

    //DAC_WR (ADR: 0)
    data[0] = 0x08, data[1] = (val>>8)&0xff, data[2] = val&0xff  ;
    alt_avalon_spi_command( SPI_1_BASE, 0, 3, data, 0, 0, 0) ;
    return ;
}


// Transverter write
void adf4351_write( uint32_t val ) {
    union {
        uint32_t val;
        uint8_t byte[4];
    } sval;

    uint8_t t;
    sval.val = val;

    t = sval.byte[0];
    sval.byte[0] = sval.byte[3];
    sval.byte[3] = t;

    t = sval.byte[1];
    sval.byte[1] = sval.byte[2];
    sval.byte[2] = t;

    alt_avalon_spi_command( SPI_1_BASE, 1, 4, (const alt_u8*)&(sval.val), 0, 0, 0 ) ;
    return ;
}




/*  // reference
 * int alt_avalon_spi_command(    // returns: success code
 *		alt_u32 base,
 *		alt_u32 slave,
 * 	    alt_u32 write_length,     const alt_u8 * 	write_data,
 * 	   	alt_u32 read_length, 	  alt_u8 * 			read_data,
 * 	   	alt_u32 flags
 * 	   	);
 */


// XB_SPI_READ
uint8_t xb_gps_spi( uint8_t write ) {
	alt_u32 slave_select = 0;
    uint8_t reply = 0;

    // If sending out A,B,C
    // gps uses full duplex mode... while writing out B, gps will be responding to A.
    // Unfortunately, spi_command discards miso data while it writes out data... thanks Altera.
    // It records a response, after all data has been written to mosi.
    // That misses half of the data, so we use this instead.
    if(write == 0){
        alt_avalon_spi_command( SPI_2_BASE, slave_select, 0, 0, 1, &reply, 0 );
    }
    else {
    	alt_avalon_spi_command( SPI_2_BASE, slave_select, 1, &write, 0, 0, 0 );
    }


	return reply;
}

// 1024 will hold about 5-8 nmea sentences, or about 2 sec of activity on gps
// must be power of 2 for speed improvment
#define XB_UART_FIFO_BUFFER_SIZE (1<<10) //1024
static fifo uf = {0};

void monitor_xb_uart(void* context);
void create_xb_uart_interrupt(){

	alt_ic_isr_register(UART_1_IRQ_INTERRUPT_CONTROLLER_ID,
			UART_1_IRQ,
			&monitor_xb_uart,
			0,
			0);

	// enable read interrupt, disable others
	uint8_t ctl = IORD_FIFOED_AVALON_UART_CONTROL(UART_1_BASE);
	IOWR_FIFOED_AVALON_UART_CONTROL(UART_1_BASE, ctl | (1 << FIFOED_AVALON_UART_CONTROL_RRDY_OFST) );
}


//collects the xb uart rx, when the library doesn't check the read data very often.
void monitor_xb_uart(void* context){
	//quick read if data available, else skip
	uint8_t data = 0;
	while( IORD_FIFOED_AVALON_UART_STATUS(UART_1_BASE) & FIFOED_AVALON_UART_STATUS_RRDY_MSK ){
		data = IORD_FIFOED_AVALON_UART_RXDATA(UART_1_BASE) ;

		fifo_enqueue(&uf, data);
	}


	//service interrupt flag
	//uint8_t sts = IORD_ALTERA_AVALON_UART_STATUS(UART_1_BASE);
	//IOWR_ALTERA_AVALON_UART_STATUS(  UART_1_BASE, 0  );
}

void xb_gps_uart_baud( uint32_t baud ) {
	 uint16_t div = (uint16_t)((float)ALT_CPU_FREQ / (float)baud + 0.5);
     IOWR_FIFOED_AVALON_UART_DIVISOR(UART_1_BASE, div);
}


uint8_t xb_gps_uart_read(  ) {
	return fifo_dequeue(&uf);
}

void xb_gps_uart_write( uint8_t val ) {

	  while (!(IORD_FIFOED_AVALON_UART_STATUS(UART_1_BASE) & FIFOED_AVALON_UART_STATUS_TRDY_MSK));
	  IOWR_FIFOED_AVALON_UART_TXDATA(UART_1_BASE,  val );
}
uint32_t xb_gps_uart_hasdata(  ) {

	//return 1;
	uint32_t s = fifo_size(&uf);
	//alt_printf( "sz: %d\n", s ) ;
	return s;
}


// LMS SPI Read
void lms_spi_read( uint8_t address, uint8_t *val )
{
    uint8_t rv ;
    if( address > 0x7f )
    {
//        alt_printf( "Invalid read address: %x\n", address ) ;
    } else {
    	alt_avalon_spi_command( SPI_0_BASE, 0, 1, &address, 0, 0, ALT_AVALON_SPI_COMMAND_MERGE ) ;
    	rv = alt_avalon_spi_command( SPI_0_BASE, 0, 0, 0, 1, val, 0 ) ;
        if( rv != 1 )
        {
//            alt_putstr( "SPI data read did not work :(\n") ;
        }
    }
    if( LMS_VERBOSE )
    {
//        alt_printf( "r-addr: %x data: %x\n", address, *val ) ;
    }
    return ;
}

// LMS SPI Write
void lms_spi_write( uint8_t address, uint8_t val )
{
    if( LMS_VERBOSE )
    {
//        alt_printf( "w-addr: %x data: %x\n", address, val ) ;
    }
    /*if( address > 0x7f )
    {
        alt_printf( "Invalid write address: %x\n", address ) ;
    } else*/ {
        uint8_t data[2] = { address |= LMS_WRITE, val } ;
        alt_avalon_spi_command( SPI_0_BASE, 0, 2, data, 0, 0, 0 ) ;
    }
    return;
}

// nsamp must be within 0-127.
// Note: Max counter value is 0xFFFFFFFF or 4,294,967,295.
// 80mhz clock will overflow in 53 sec.
void gps_calibration_start(uint32_t nsamp){

	// config register  { 7 6 5 4 3 2 1 0 }
	//                    r n n n n n n n
	// n = nsamp (7 bits - 1 :: max = 126)
	// r = reset flag
	// * = not used

	uint8_t rst = 0x80;
	uint8_t ssize = (uint8_t )(( nsamp & ((1<<7)-1) ));
	uint8_t cfg = rst | ssize;

	IOWR_ALTERA_AVALON_PIO_DATA(PIO_4_BASE, cfg);		// send cfg
	IOWR_ALTERA_AVALON_PIO_DATA(PIO_4_BASE, ssize);		// make sure size is latched
	IOWR_ALTERA_AVALON_PIO_DATA(PIO_4_BASE, 0);			// start counter

}

// Value needs to be cached to maintain returned value over the 4 byte read cycle
static uint32_t gps_calibration_counter_cached = 0;

// Returns 0 when counting, and # > 0 when complete.
uint32_t gps_calibration_read(){
	return IORD_ALTERA_AVALON_PIO_DATA(PIO_3_BASE);
}


static char xb_ROM_data[8] = {"NULLDATA"};
void xb_1wire_read_ROM(){
	// put spaces in the cache
	 sprintf(xb_ROM_data, "        ");

	// attempt to acquire the 1-Wire Net
	if (!owAcquire(0,0))
	{
	    sprintf(xb_ROM_data, "%%AQ_%d", owGetErrorNum());
	    return;
	}

	int found = FindDevices(0, (uint8_t (*)[8])&xb_ROM_data, _1_WIRE_DS2411_FAMILY_CODE, 1);


	if(!found){
	     sprintf(xb_ROM_data, "%%FD_%d", owGetErrorNum());
	     return;
	}

}

// returns the 1wire data that was read at startup and stored in xb_ROM_data
uint8_t read_ROM_byte_at(uint8_t index){
	if(index >= sizeof(xb_ROM_data)){
		return 0;
	}

	return xb_ROM_data[index];
}




#define UART_PKT_MAGIC          'N'
#define UART_EXPRESS_PKT_MAGIC 'X'
#define UART_PKT_MODE_CNT_MASK   0xF
#define UART_PKT_MODE_CNT_SHIFT  0

#define UART_PKT_MODE_DEV_MASK   0x30
#define UART_PKT_MODE_DEV_SHIFT  4
#define UART_PKT_DEV_GPIO        (0<<UART_PKT_MODE_DEV_SHIFT)
#define UART_PKT_DEV_LMS         (1<<UART_PKT_MODE_DEV_SHIFT)
#define UART_PKT_DEV_VCTCXO      (2<<UART_PKT_MODE_DEV_SHIFT)
#define UART_PKT_DEV_SI5338      (3<<UART_PKT_MODE_DEV_SHIFT)

#define UART_PKT_MODE_DIR_MASK   0xC0
#define UART_PKT_MODE_DIR_SHIFT  6
#define UART_PKT_MODE_DIR_READ   (2<<UART_PKT_MODE_DIR_SHIFT)		//read is 8 bit 1
#define UART_PKT_MODE_DIR_WRITE  (1<<UART_PKT_MODE_DIR_SHIFT)		//write is 7 bit 1

struct uart_pkt {
  unsigned char magic;

  //  | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
  //  |  dir  |  dev  |     cnt       |
  unsigned char mode;
};

struct uart_cmd {
  unsigned char addr;
  unsigned char data;
};


//writes chars
void debugs(const char* s){
	return;
	// ----- SKIPPED ON RELEASE ------

	int k = 0;
	while(s[k] != 0){
		xb_gps_uart_write(s[k]);
		k++;
	}
}
// writes line
void debug(const char* s){
	return;
	// ----- SKIPPED ON RELEASE ------

	debugs("DBG: ");
	debugs(s);
	debugs("\r\n");
}








void init_NIOS(){
	  // Set the prescaler for 400kHz with an 80MHz clock (prescaler = clock / (5*desired) - 1)
	  IOWR_16DIRECT(I2C, OC_I2C_PRESCALER, 39 ) ;
	  IOWR_8DIRECT(I2C, OC_I2C_CTRL, OC_I2C_ENABLE ) ;

	  // Set the UART divisor to 19 to get 4000000bps UART (baud rate = clock/(divisor + 1))
	  IOWR_FIFOED_AVALON_UART_DIVISOR(UART_0_BASE, 19) ;

	  // Set the IQ Correction parameters to 0
	  IOWR_ALTERA_AVALON_PIO_DATA(IQ_CORR_RX_PHASE_GAIN_BASE, DEFAULT_CORRECTION);
	  IOWR_ALTERA_AVALON_PIO_DATA(IQ_CORR_TX_PHASE_GAIN_BASE, DEFAULT_CORRECTION);

}


const struct {
  enum {
	  GDEV_UNKNOWN,
	  GDEV_GPIO,
	  GDEV_IQ_CORR_RX_GAIN,
	  GDEV_IQ_CORR_RX_PHASE,
	  GDEV_IQ_CORR_TX_GAIN,
	  GDEV_IQ_CORR_TX_PHASE,
	  GDEV_FPGA_VERSION,
	  GDEV_TIME_TIMER,
	  GDEV_XB_LO,
	  GDEV_VCTXCO,
	  GDEV_EXPANSION,
	  GDEV_EXPANSION_DIR,
	  GDEV_XB_GPS_SPI,
	  GDEV_XB_GPS_UART,
	  GDEV_XB_GPS_UART_BAUD,
	  GDEV_XB_GPS_UART_HASDATA,
	  GDEV_XB_GPS_CALIBRATION_START,
	  GDEV_XB_GPS_CALIBRATION_READ,
	  GDEV_XB_1WIRE_READ_BYTE
  } gdev;

  int start, len;
} gdev_lut[] = {
	  {GDEV_GPIO,           0, 4},		//config gpio
	  {GDEV_IQ_CORR_RX_GAIN,    4, 2},
	  {GDEV_IQ_CORR_RX_PHASE,   6, 2},
	  {GDEV_IQ_CORR_TX_GAIN,    8, 2},
	  {GDEV_IQ_CORR_TX_PHASE,  10, 2},
	  {GDEV_FPGA_VERSION,  12, 4},
	  {GDEV_TIME_TIMER,    16, 16},		//referring to "time tamer"
	  {GDEV_VCTXCO,        34, 2},
	  {GDEV_XB_LO,         36, 4},
	  {GDEV_EXPANSION,     40, 4},		//expansion gpio
	  {GDEV_EXPANSION_DIR, 44, 4},
	  {GDEV_XB_GPS_SPI,    48, 4},		//Custom devices
	  {GDEV_XB_GPS_UART,   52, 4},
	  {GDEV_XB_GPS_UART_BAUD,   56, 4},
	  {GDEV_XB_GPS_UART_HASDATA,60, 4},
	  {GDEV_XB_GPS_CALIBRATION_START,64, 4},
	  {GDEV_XB_GPS_CALIBRATION_READ,68, 4},
	  {GDEV_XB_1WIRE_READ_BYTE, 72, 4}
};


void write_uart(unsigned char val) {
  // spin while not transmit ready
  while (!(IORD_FIFOED_AVALON_UART_STATUS(UART_0_BASE) & FIFOED_AVALON_UART_STATUS_TRDY_MSK))
  {}
  IOWR_FIFOED_AVALON_UART_TXDATA(UART_0_BASE,  val);
}

// Entry point
int main()
{
  //debug("\r\n[NIOS POWER ON]");







  // start comm interface with Cypress USB controller and Lime Chip
  init_NIOS();

  //--read the RF board serial number
  // Get the current serial code/ROM data from the 1wire DS2411
  // Because the cypress uart -> nios is fifo buffered, any commands from the host
  // should just get buffered while the serial loads, then when this returns
  // the host commands should get processed quickly.
  xb_1wire_read_ROM();


  // initialize fifo GPS uart buffer
  uint8_t uart_buffer[XB_UART_FIFO_BUFFER_SIZE] = {0};
  uf = fifo_new(uart_buffer, sizeof(uart_buffer));


  /* Event loop never exits. */
  {
      char state;
      enum {
          LOOKING_FOR_MAGIC,
          READING_MODE,
          READING_CMDS,
          EXECUTE_CMDS
      };

      unsigned short i, cnt;
      unsigned char mode;
      unsigned char buf[14] = {0};
      struct uart_cmd *cmd_ptr;
      uint32_t tmpvar = 0;
      uint32_t iovar = 0;

      uint32_t uart_hasdata_cached = 0;



      state = LOOKING_FOR_MAGIC;
      while(1)
      {
		  monitor_xb_uart(0);


          // Check if anything is in the FSK UART
          if( IORD_FIFOED_AVALON_UART_STATUS(UART_0_BASE) & FIFOED_AVALON_UART_STATUS_RRDY_MSK )
          {
              uint8_t val ;
              int isRead;
              int isWrite;

              val = IORD_FIFOED_AVALON_UART_RXDATA(UART_0_BASE) ;

              // when a "magic is found, a specific behavior - parsing the following data as uart commands - ensues until set back to LOOKING
              switch (state) {
              case LOOKING_FOR_MAGIC:
                  if (val == UART_PKT_MAGIC)
                      state = READING_MODE;
                  break;
              case READING_MODE:
                  mode = val;
                  if ((mode & UART_PKT_MODE_CNT_MASK) > 7) {		//clamps command count to 7
                      mode &= ~UART_PKT_MODE_CNT_MASK;
                      mode |= 7;
                  }
                  i = 0;
                  cnt = (mode & UART_PKT_MODE_CNT_MASK) * sizeof(struct uart_cmd);			//gets number of bytes given number commands
                  state = READING_CMDS;
                  break;
              case READING_CMDS:
                  // cnt here means the number of bytes to read
                  buf[i++] = val;
                  if (!--cnt)
                      state = EXECUTE_CMDS;
                  break;
              default:
                  break;
              }


              isRead = (mode & UART_PKT_MODE_DIR_MASK) == UART_PKT_MODE_DIR_READ;
              isWrite = (mode & UART_PKT_MODE_DIR_MASK) == UART_PKT_MODE_DIR_WRITE;

              if (state == EXECUTE_CMDS) {
                  write_uart(UART_PKT_MAGIC);
                  write_uart(mode);
                  // cnt here means the number of commands
                  cnt = (mode & UART_PKT_MODE_CNT_MASK);
                  cmd_ptr = (struct uart_cmd *)buf;

                  if ((mode & UART_PKT_MODE_DEV_MASK) == UART_PKT_DEV_LMS) {
                      for (i = 0; i < cnt; i++) {
                          if (isRead) {
                              lms_spi_read(cmd_ptr->addr, &cmd_ptr->data);
                          } else if (isWrite) {
                              lms_spi_write(cmd_ptr->addr, cmd_ptr->data);
                              cmd_ptr->data = 0;
                          } else {
                              cmd_ptr->addr = 0;
                              cmd_ptr->data = 0;
                          }
                          cmd_ptr++;
                      }
                  }
                  if ((mode & UART_PKT_MODE_DEV_MASK) == UART_PKT_DEV_SI5338) {
                      for (i = 0; i < cnt; i++) {
                          if (isRead) {
                              uint8_t tmpvar;
                              si5338_read(cmd_ptr->addr, &tmpvar);
                              cmd_ptr->data = tmpvar;
                          } else if (isWrite) {
                              si5338_write(cmd_ptr->addr, cmd_ptr->data);
                              cmd_ptr->data = 0;
                          } else {
                              cmd_ptr->addr = 0;
                              cmd_ptr->data = 0;
                          }
                          cmd_ptr++;
                      }
                  }


#define ARRAY_SZ(x) (sizeof(x)/sizeof(x[0]))

                  	  	  //compress all data bytes into a tmpvar according to address offset
#define COLLECT_BYTES(x)       tmpvar &= ~ ( 0xff << ( 8 * cmd_ptr->addr));   \
                              tmpvar |= cmd_ptr->data << (8 * cmd_ptr->addr); \
                              if (lastByte) { x; tmpvar = 0; } \
                              cmd_ptr->data = 0;
#define SPLIT_WRITE(reg, shift)     ({                                   \
                              iovar = IORD_ALTERA_AVALON_PIO_DATA(reg);  \
                              iovar &= 0xffff << (16 - shift);           \
                              iovar |= (tmpvar) << shift;                \
                              IOWR_ALTERA_AVALON_PIO_DATA(reg, iovar);   \
                         })
                  if ((mode & UART_PKT_MODE_DEV_MASK) == UART_PKT_DEV_GPIO) {
                    uint32_t device;
                    volatile uint32_t lol, lol3;
                    int lut, lastByte;
                    for (i = 0; i < cnt; i++) {
                        device = GDEV_UNKNOWN;
                        lastByte = 0;
                        // trying to match cmd addr with lut index...
                        for (lut = 0; lut < ARRAY_SZ(gdev_lut); lut++) {
                            if (gdev_lut[lut].start <= cmd_ptr->addr && (gdev_lut[lut].start + gdev_lut[lut].len) > cmd_ptr->addr) {
                                cmd_ptr->addr -= gdev_lut[lut].start;	// turn address into offset
                                device = gdev_lut[lut].gdev;
                                lastByte = cmd_ptr->addr == (gdev_lut[lut].len - 1);	//is it the last byte in the word
                                break;
                            }
                        }

                        if (isRead) {
                            if (device == GDEV_FPGA_VERSION){
                                cmd_ptr->data = (FPGA_VERSION >> (cmd_ptr->addr * 8));
                            }
                            else if (device == GDEV_TIME_TIMER){
                                cmd_ptr->data = IORD_8DIRECT(TIME_TAMER, cmd_ptr->addr);
                            }
                            else if (device == GDEV_GPIO){
                                cmd_ptr->data = (IORD_ALTERA_AVALON_PIO_DATA(PIO_0_BASE)) >> (cmd_ptr->addr * 8);
                            }
                            else if (device == GDEV_XB_GPS_SPI){
                            	//sends data to gps, then reads response
                            	// round trip ~ 10us  (5Mhz clock)
                            	if(cmd_ptr->addr == 0){
                            		cmd_ptr->data = xb_gps_spi( cmd_ptr->data );
                            	}
                            }
                            else if (device == GDEV_XB_GPS_UART){
                            	//cmd bytes come in low first
                            		cmd_ptr->data = xb_gps_uart_read( );
                            }
                            else if (device == GDEV_XB_GPS_UART_HASDATA){

                            	// data comes in/out as low bytes first
                            	if(cmd_ptr->addr == 0)
								{
                            		uart_hasdata_cached = xb_gps_uart_hasdata( );
								}
                            	cmd_ptr->data = (uart_hasdata_cached >> (cmd_ptr->addr * 8)) & 0xff;
                            }
                            else if (device == GDEV_EXPANSION)
                                cmd_ptr->data = (IORD_ALTERA_AVALON_PIO_DATA(PIO_1_BASE)) >> (cmd_ptr->addr * 8);
                            else if (device == GDEV_EXPANSION_DIR)
                                cmd_ptr->data = (IORD_ALTERA_AVALON_PIO_DATA(PIO_2_BASE)) >> (cmd_ptr->addr * 8);
                            else if (device == GDEV_IQ_CORR_RX_GAIN)
                            	cmd_ptr->data = (IORD_ALTERA_AVALON_PIO_DATA(IQ_CORR_RX_PHASE_GAIN_BASE)) >> (cmd_ptr->addr * 8);
                            else if (device == GDEV_IQ_CORR_RX_PHASE)
                            	cmd_ptr->data = (IORD_ALTERA_AVALON_PIO_DATA(IQ_CORR_RX_PHASE_GAIN_BASE)) >> ((cmd_ptr->addr + 2) * 8);
                            else if (device == GDEV_IQ_CORR_TX_GAIN)
                            	cmd_ptr->data = (IORD_ALTERA_AVALON_PIO_DATA(IQ_CORR_TX_PHASE_GAIN_BASE)) >> (cmd_ptr->addr * 8);
                            else if (device == GDEV_IQ_CORR_TX_PHASE)
                            	cmd_ptr->data = (IORD_ALTERA_AVALON_PIO_DATA(IQ_CORR_TX_PHASE_GAIN_BASE)) >> ((cmd_ptr->addr + 2) * 8);
                            else if (device == GDEV_XB_GPS_CALIBRATION_READ){
                            	if(cmd_ptr->addr == 0){
                            		// read the latest value and store in gps_calibration_counter
                            		// cache the value that was obtained
                            		gps_calibration_counter_cached = gps_calibration_read();
                            	}
								cmd_ptr->data = (gps_calibration_counter_cached >> (cmd_ptr->addr * 8)) & 0xff;
                            }
                            else if (device == GDEV_XB_GPS_CALIBRATION_START) {
								COLLECT_BYTES(gps_calibration_start(tmpvar));
							}
                            else if (device == GDEV_XB_1WIRE_READ_BYTE){
                            	if(cmd_ptr->addr == 0){
                            		cmd_ptr->data = read_ROM_byte_at( cmd_ptr->data );
                            	}
                            }
                        } else if (isWrite) {
                            if (device == GDEV_TIME_TIMER) {
                                IOWR_8DIRECT(TIME_TAMER, cmd_ptr->addr, 1) ;
                            } else if (device == GDEV_GPIO){
                                COLLECT_BYTES(IOWR_ALTERA_AVALON_PIO_DATA(PIO_0_BASE, tmpvar));
                            } else if (device == GDEV_VCTXCO) {
                                COLLECT_BYTES(dac_write(tmpvar));
                            } else if (device == GDEV_XB_LO) {
                                COLLECT_BYTES(adf4351_write(tmpvar));
                            } else if (device == GDEV_XB_GPS_UART) {
                            	// Only first byte has data
                            	if(cmd_ptr->addr == 0)
								{
									xb_gps_uart_write(cmd_ptr->data);
								}
                            } else if (device == GDEV_XB_GPS_UART_BAUD) {
                            	COLLECT_BYTES( xb_gps_uart_baud( tmpvar) );
                            } else if (device == GDEV_EXPANSION) {
                                COLLECT_BYTES(IOWR_ALTERA_AVALON_PIO_DATA(PIO_1_BASE, tmpvar));
                            } else if (device == GDEV_EXPANSION_DIR) {
                                COLLECT_BYTES(IOWR_ALTERA_AVALON_PIO_DATA(PIO_2_BASE, tmpvar));
                            } else if (device == GDEV_IQ_CORR_RX_GAIN) {
                                COLLECT_BYTES(SPLIT_WRITE(IQ_CORR_RX_PHASE_GAIN_BASE, 0));
                            } else if (device == GDEV_IQ_CORR_RX_PHASE) {
                                COLLECT_BYTES(SPLIT_WRITE(IQ_CORR_RX_PHASE_GAIN_BASE, 16));
                            } else if (device == GDEV_IQ_CORR_TX_GAIN) {
                                COLLECT_BYTES(SPLIT_WRITE(IQ_CORR_TX_PHASE_GAIN_BASE, 0));
                            } else if (device == GDEV_IQ_CORR_TX_PHASE) {
                                COLLECT_BYTES(SPLIT_WRITE(IQ_CORR_TX_PHASE_GAIN_BASE, 16));
                            }

                        } else {
                            cmd_ptr->addr = 0;
                            cmd_ptr->data = 0;
                        }

                        cmd_ptr++;
                     } // end for
                  } // end if : GPIO

                  // after all execution, write back edited buffer data
                  cmd_ptr = (struct uart_cmd *)buf;
                  for (i = 0; i < cnt; i++) {
                      write_uart(cmd_ptr->addr);
                      write_uart(cmd_ptr->data);
                      cmd_ptr++;
                  }

                  // fill any unused space with 0xff
                  for (i = 2 + cnt * sizeof(struct uart_cmd); (i % 16); i++) {
                      write_uart(0xff);
                  }
                  state = LOOKING_FOR_MAGIC;
              }
          }

      }
  }



  return 0;
}

