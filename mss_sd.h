

#ifndef MSS_SD_H_
#define MSS_SD_H_

#include "../mss_spi/mss_spi.h"
#include "../mss_gpio/mss_gpio.h"
#include "../mss_pdma/mss_pdma.h"
#include "string.h"
#include "../../CMSIS/m2sxxx.h"

#ifdef __cplusplus
extern "C" {
#endif 

//define cmd num
#define	SD_CMD0 0x40
#define	SD_CMD8 0x48
#define	SD_CMD9 0x49
#define	SD_CMD13 0x4D
#define	SD_CMD16 0x50
#define	SD_CMD17 0x51
#define SD_CMD24 0x58
#define SD_CMD32 0x60
#define SD_CMD33 0x61
#define SD_CMD38 0x66
#define SD_CMD55 0x77
#define SD_CMD58 0x7A
#define SD_ACMD13 0x4D
#define SD_ACMD41 0x69

#define SD_Sync0 0x94
#define SD_Sync1 0x53	
#define SD_Sync2 0x94

#define SD_POWER_WAIT_TIME_MS 500 //delay 500 ms
#define SD_ENTER_READY_WAIT_MS 100 //time to wait to repeat CMD55+ACMD41 commands during initialization to enter ready state
#define SD_INIT_CMD_DELAY 10 //waiting time between two commands 

/***************************************************************************//**
  This enumeration is used to define the status of SD card.
  
  - ENUM_IDLE:
		Idle status for SD card. In the state, the SD card is power off.
        
  - ENUM_DISABLE:
		Turn off the SD card.
        
  - ENUM_ENABLE:
		Turn on the SD card and set the spi speed to 500 kHz.
        
  - ENUM_DUMMY_CLK:
		Send ten dummy clock to SD card.
        
  - ENUM_SEND_GET_CMD0:
		Send CMD0 (Resets the SD Memory Card)
        
  - ENUM_SEND_GET_CMD8:
		Send CMD8 (Sends SD Memory Card interface condition)

  - ENUM_SEND_GET_CMD58_FIRST:
		Send CMD58 (Reads the OCR register of a card CCS bit is assigned to OCR[30].)

  - ENUM_SPI_CLK_SET_FAST:
		Modify the spi speed to user's need (default is up to 2 MHz)

  - ENUM_SEND_GET_CMD55_ACMD41:
		Send CMD55 and ACMD41 (Sends host capacity support information and activates 
		the card's initialization process.)

  - ENUM_SEND_GET_CMD58_SEC:
		Send CMD58 (Reads the OCR register of a card CCS bit is assigned to OCR[30].)

  - ENUM_INIT_SUCCESS:
		Initializtion is complete and success.

  - ENUM_INIT_ERROR:
		Initialization occurs error. The program runs too long.
        
  - ENUM_WRITE_ERROR:
		Writing block occurs error. The SD card doesn't reply the normal response
		or the data response token is error.

  - ENUM_READ_ERROR:
		Reading block occurs error. The SD card doesn't reply the normal response
		or the data error response occur.

  - ENUM_ADDR_OUT_OF_RANGE:
		The designated address is out of the SD card maximum size. 

  - ENUM_CLOSE_ERROR:
		Close the SD card and store the satellite information occur error.

  - ENUM_GetSat_ERROR:
		Comparing the satellite information sync word or checking the CRC occur error.
 */
typedef enum mss_sd_enum_state{
	ENUM_IDLE = 0,
	ENUM_DISABLE,
	ENUM_ENABLE,
	ENUM_DUMMY_CLK,
	ENUM_SEND_GET_CMD0,
	ENUM_SEND_GET_CMD8,
	ENUM_SEND_GET_CMD58_FIRST,
	ENUM_SPI_CLK_SET_FAST,
	ENUM_SEND_GET_CMD55_ACMD41,
	ENUM_SEND_GET_CMD58_SEC,
	ENUM_INIT_SUCCESS,
	ENUM_INIT_ERROR = -10,
	ENUM_WRITE_ERROR = -15,
	ENUM_READ_ERROR = -20,
	ENUM_ADDR_OUT_OF_RANGE=-25,
	ENUM_CLOSE_ERROR=-30,
	ENUM_GetSat_ERROR=-35
} mss_sd_enum_state_t;

/***************************************************************************//**
  This enumeration is used to define the spi speed.
  
  - SpiClock_2000k:
        
  - SpiClock_1250k:

  - SpiClock_500k:
        
  - SpiClock_200k:
        
 */
typedef enum SpiClockEnum {
	SpiClock_2000k = 26,
	SpiClock_1250k = 40,
	SpiClock_500k = 100,
	SpiClock_200k = 250
}SpiClock_t; //need to set when the main freq is 50MHz

/***************************************************************************//**
  There is one instance of this structure for each of the microcontroller
  subsystem's SDs. Instances of this structure are used to identify a specific
  SD. A pointer to an instance of the mss_sd_state_t structure is passed as
  the first parameter to MSS SD driver functions to identify which SD should
  perform the requested operation.
 */
typedef struct __mss_sd_state_t
{
	mss_gpio_id_t enableGpioPin;
	mss_spi_instance_t * mss_spi;
	mss_spi_slave_t slave;
	SpiClock_t spiClockRate;
	mss_sd_enum_state_t enum_state;
	uint8_t no;
	uint8_t init_count;
	uint8_t cmd_buff[6];
	uint8_t first_check;
	uint8_t fourty_min_delay;
	uint32_t c_size;
	uint8_t write_addr[4];
	uint8_t read_addr[4];

	float ncu_ecef[3];
	float ncu_min_elv;
	float ncu_max_range;
	uint8_t ncu_active;

	float cu_ecef[3];
	float cu_min_elv;
	float cu_max_range;
	uint8_t cu_active;

	float ntu_ecef[3];
	float ntu_min_elv;
	float ntu_max_range;
	uint8_t ntu_active;

	float iist_ecef[3];
	float iist_min_elv;
	float iist_max_range;
	uint8_t iist_active;
}mss_sd_state_t;

extern void delay(uint16_t num);
extern uint16_t crc_16(uint8_t *data, uint16_t len);

/***************************************************************************//**
    SD function
/***************************************************************************//**
/***************************************************************************//**
  The MSS_SD_InitState() function initializes the status of the SD card.
  The MSS_SD_InitState() function must be called before any
  other MSS SD driver functions can be called.
  
  @param sd
    The sd parameter is a pointer to an mss_sd_state_t structure
    identifying the MSS SD hardware block to be initialized. 

  @param EnableGpioPin
  	The EnableGpioPin is a GPIO pin number which can control the 
  	SD card power.

  @param mss_spi
  	The mss_spi parameter is a pointer to an mss_spi_instance_t structure
  	identifing the MSS SD hardware to be used.

  @param slave
  	The slave parameter is an MSS SPI slave. It identify the MSS SD hardware to be used.

  @param SpiClockRate
  	The SpiClockRate parameter is the frequency when the MSS SD card transport in fast mode.
	- SpiClock_2000k:        
  	- SpiClock_1250k:
  	- SpiClock_500k:        
  	- SpiClock_200k:

  @param No
  	The No parameter is the number of the SD card.

  @note 
    Need to configure the MSS_SPI before use this function.
    

  Example:
  @code
  	uint8_t addr[4]={0x00,0x00,0x01,0x00};

  	mss_sd_state_t sd [2];
  	MSS_SD_InitState(&sd[0],MSS_GPIO_0,&g_mss_spi1,MSS_SPI_SLAVE_0,SpiClock_2000k,0);
  	MSS_SD_Init(&sd[0]);
  
  	MSS_SD_WriteBlock( &sd[0] , addr , tx_data);
  	MSS_SD_ReadBlock( &sd[0] , addr , rx_data , 0 );
  	MSS_SD_close(&sd[0]);
  @endcode
*/

void MSS_SD_InitState
(
	mss_sd_state_t * sd,
	mss_gpio_id_t EnableGpioPin,
	mss_spi_instance_t * mss_spi,
	mss_spi_slave_t slave,
	SpiClock_t SpiClockRate,
	uint8_t No
);

/***************************************************************************//**
  The MSS_SD_Init() function runs the SD card initial program. It can automatically 
  initialize the SD card and get the satellite information.

  @param sd
    The sd parameter is a pointer to an mss_sd_state_t structure
    identifying the MSS SD hardware block to be initialized. 

  @note 
    Need to configure the MSS_SPI before use this function.
    
    
  Example:
  @code
  	uint8_t addr[4]={0x00,0x00,0x01,0x00};

  	mss_sd_state_t sd [2];
  	MSS_SD_InitState(&sd[0],MSS_GPIO_0,&g_mss_spi1,MSS_SPI_SLAVE_0,SpiClock_2000k,0);
  	MSS_SD_Init(&sd[0]);
  
  	MSS_SD_WriteBlock( &sd[0] , addr , tx_data);
  	MSS_SD_ReadBlock( &sd[0] , addr , rx_data , 0 );
  	MSS_SD_close(&sd[0]);
  @endcode
*/

void MSS_SD_Init
(
	mss_sd_state_t * sd
);

/***************************************************************************//**
  The MSS_SD_WriteBlock() function writes a block to SD card memory by spi transmission.

  @param sd
    The sd parameter is a pointer to an mss_sd_state_t structure
    identifying the MSS SD hardware block to be initialized. 

  @param addr
  	The addr parameter is an address in SD card memory which the data will be store.

  @param tx_data
  	The tx_data parameter is a pointer to the buffer where the data tranport to the SD card
  	and	store in the SD card. The data need to prepare 512 bytes because one block in SDXC 
  	card is 512 bytes. If tx_data vector is large than 512 bytes, only the first 512 bytes 
  	of data will be stored in the sd card, and the rest will be discard.

  @note 
    Need to configure the MSS_SPI before use this function.


  Example:
  @code
  	uint8_t addr[4]={0x00,0x00,0x01,0x00};

  	mss_sd_state_t sd [2];
  	MSS_SD_InitState(&sd[0],MSS_GPIO_0,&g_mss_spi1,MSS_SPI_SLAVE_0,SpiClock_2000k,0);
  	MSS_SD_Init(&sd[0]);
  
  	MSS_SD_WriteBlock( &sd[0] , addr , tx_data);
  	MSS_SD_ReadBlock( &sd[0] , addr , rx_data , 0 );
  	MSS_SD_close(&sd[0]);
  @endcode
*/

void MSS_SD_WriteBlock
(
	mss_sd_state_t * sd,
	const uint8_t * addr,
	uint8_t * tx_data
);

/***************************************************************************//**
  The MSS_SD_ReadBlock() function reads a block from SD card memory by spi transmission.

  @param sd
    The sd parameter is a pointer to an mss_sd_state_t structure
    identifying the MSS SD hardware block to be initialized. 

  @param addr
  	The addr parameter is an address in SD card memory which the data will be read.

  @param rx_data
  	The rx_data parameter is a pointer to the buffer where the data tranport from the SD card.
  	The buffer needs to prepare 512 bytes because one block in SDXC	card is 512 bytes.
  	If rx_data vector is small than 512 bytes, overwrite may happen.

  @param init_use
  	The init_use parameter is a flag to identify the function is used to get the satellite 
  	information or get the science data. If this parameter sets 1, the address check function
  	will not be used. Default is 0.

  @note 
    Need to configure the MSS_SPI before use this function.
      

  Example:
  @code
  	uint8_t addr[4]={0x00,0x00,0x01,0x00};

  	mss_sd_state_t sd [2];
  	MSS_SD_InitState(&sd[0],MSS_GPIO_0,&g_mss_spi1,MSS_SPI_SLAVE_0,SpiClock_2000k,0);
  	MSS_SD_Init(&sd[0]);
  
  	MSS_SD_WriteBlock( &sd[0] , addr , tx_data);
  	MSS_SD_ReadBlock( &sd[0] , addr , rx_data , 0 );
  	MSS_SD_close(&sd[0]);
  @endcode

*/

void MSS_SD_ReadBlock
(
	mss_sd_state_t * sd,
	const uint8_t * addr,
	uint8_t * rx_data,
	uint8_t init_use
);

/***************************************************************************//**
  The MSS_PDMA_SendToSbandReg() function send data to mss_spi1's tx register and
  transport data to Sband transmitter.

  @param pdma_channel
  	The pdma_channel parameter identifies the PDMA channel used by the function.

  @param tx_data
  	The tx_data parameter is a pointer to the buffer where the data tranport to the Sband 
  	transmitter. The data size has to be 512 bytes. If the tx_data is large than 512 bytes,
  	only the first 512 bytes of data will be transport to Sband, and the rest will be discard.

  @note
  	Need to configure the pdma channel before use this function.

  Example:
  @code
  	MSS_PDMA_SendToSbandReg( PDMA_CHANNEL_0 , tx_data );
  @endcode
*/

uint8_t MSS_PDMA_SendToSbandReg
(
	pdma_channel_id_t pdma_channel,
	uint8_t* tx_data
);

/***************************************************************************//**
  The MSS_SD_Close function close the SD card and update the satellite information.
  
  @param sd
  	The sd parameter is a pointer to an mss_sd_state_t structure
  	identifying the MSS SD hardware block to be initialized. 

  @note 
    Need to configure the MSS_SPI before use this function.
     

  Example:
  @code
  	uint8_t addr[4]={0x00,0x00,0x01,0x00};

  	mss_sd_state_t sd [2];
  	MSS_SD_InitState(&sd[0],MSS_GPIO_0,&g_mss_spi1,MSS_SPI_SLAVE_0,SpiClock_2000k,0);
  	MSS_SD_Init(&sd[0]);
  
  	MSS_SD_WriteBlock( &sd[0] , addr , tx_data);
  	MSS_SD_ReadBlock( &sd[0] , addr , rx_data , 0 );
  	MSS_SD_close(&sd[0]);
  @endcode
*/

void MSS_SD_Close
(
	mss_sd_state_t * sd
);

/***************************************************************************//**
  The MSS_SD_PowerOff funcion turn off the SD card power by designate GPIO port.
  This function is used at emergency. In normal conditions, this function was called
  by the MSS_SD_Close().

  @param sd
  	The sd parameter is a pointer to an mss_sd_state_t structure
  	identifying the MSS SD hardware block to be initialized. 

  @note 
  	Need to configure the MSS_GPIO before use this function

  Example:
  @code
	MSS_SD_PowerOff( &sd[0] );
  @endcode	
*/

void MSS_SD_PowerOff
(
	mss_sd_state_t * sd
);

/***************************************************************************//**
  The MSS_SD_PowerOn funcion turn on the SD card power by designate GPIO port.
  This function is used at emergency. In normal conditions, this function was called
  by the MSS_SD_Init().

  @param sd
  	The sd parameter is a pointer to an mss_sd_state_t structure
  	identifying the MSS SD hardware block to be initialized. 

  @note 
  	Need to configure the MSS_GPIO before use this function

  Example:
  @code
	MSS_SD_PowerOn( &sd[0] );
  @endcode	
*/

void MSS_SD_PowerOn
(
	mss_sd_state_t * sd
);

/***************************************************************************//**
	Ground Use
/***************************************************************************//**
/***************************************************************************//**
* All write all read check
*/

void MSS_SD_First_Check
(
	mss_sd_state_t * sd
);

/***************************************************************************//**
* Store initial satellite information
** Need to Use Before Flight
** Need to Set Before Flight
*/

void MSS_SD_FirstUse
(
	mss_sd_state_t * sd
);

/***************************************************************************//**
* Checking all data is correct when all write all read 
*/

uint8_t MSS_Break_Block_Check
(
	uint8_t* test_data
);

/***************************************************************************//**
* Get the SD card CSD register and get the SD card maximum size from CSD
*/

uint32_t MSS_SD_SizeCheck
(
	mss_sd_state_t * sd
);

#ifdef __cplusplus
}
#endif

#endif /* MSS_SD_H_*/
