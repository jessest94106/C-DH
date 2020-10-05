

#ifndef MSS_SD_C_
#define MSS_SD_C_

#include "mss_sd.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif 

/***************************************************************************//**
  local functions
 */
static void MSS_SD_fsm(mss_sd_state_t * sd);
static void MSS_SD_CreateCMD(mss_sd_state_t * sd, uint8_t sdCmdCode, const uint8_t * arg, uint8_t crc);
static void MSS_SD_SendCmd(mss_sd_state_t * sd,uint8_t * cmd_array, uint16_t cmd_size);
static uint8_t MSS_SD_GetR1Response(mss_sd_state_t * sd, uint8_t check_response);
static void MSS_SD_GetOtherResponse(mss_sd_state_t * sd, uint8_t Response_size);
static uint8_t MSS_SD_GetWriteResponse(mss_sd_state_t * sd);
static uint8_t MSS_SD_GetReadResponse(mss_sd_state_t * sd);
static uint8_t MSS_SD_AddrCheck(uint32_t c_size,const uint8_t* addr,uint8_t mode);
static void MSS_SD_CreateSatInform(mss_sd_state_t * sd, uint8_t* sat_inform);
static uint8_t MSS_SD_GetSatInform(mss_sd_state_t * sd, uint8_t * sat_inform);

/***************************************************************************//**
    SD function
 */
/***************************************************************************//**
* MSS_SD_InitState()
* See "mss_sd.h" for details of how to use this function.
*/
void MSS_SD_InitState(mss_sd_state_t * sd,mss_gpio_id_t EnableGpioPin,mss_spi_instance_t * mss_spi,
		mss_spi_slave_t slave,SpiClock_t SpiClockRate,uint8_t No)
{

	memset(sd,0,sizeof(mss_sd_state_t));
	sd->enableGpioPin=EnableGpioPin;
	sd->mss_spi=mss_spi;
	sd->slave=slave;
	sd->spiClockRate=SpiClockRate;
	sd->enum_state=ENUM_IDLE;
	sd->no=No;
}

/***************************************************************************//**
* MSS_SD_Init()
* See "mss_sd.h" for details of how to use this function.
*/
void MSS_SD_Init(mss_sd_state_t * sd)
{
	sd->enum_state=ENUM_DISABLE;
	for(sd->init_count=0;sd->init_count<200;){
		MSS_SD_fsm(sd);
		if(sd->enum_state==ENUM_INIT_SUCCESS) break;
	}
	if(sd->init_count==200 && sd->enum_state!=ENUM_INIT_SUCCESS){
		sd->enum_state=ENUM_INIT_ERROR;
		return;
	}

	uint8_t addr[4]={0x00,0x00,0x00,0x00};
	uint8_t rx_data[512];
	for(sd->init_count=0;sd->init_count<10;sd->init_count++){
		addr[3]=sd->init_count;
		memset(rx_data,0,512);
		MSS_SD_ReadBlock(sd,addr,rx_data,1);
		if(MSS_SD_GetSatInform(sd,rx_data)) break;//Get the store data in first addr	
	}
	if(sd->init_count==10){
		sd->enum_state=ENUM_INIT_ERROR;
		return;
	}

}

/***************************************************************************//**
* MSS_SD_WriteBlock()
* See "mss_sd.h" for details of how to use this function.
*/
void MSS_SD_WriteBlock(mss_sd_state_t * sd,const uint8_t * addr,uint8_t * tx_data)
{
	if(MSS_SD_AddrCheck(sd->c_size,addr,1)){
		sd->enum_state=ENUM_ADDR_OUT_OF_RANGE;
		return;
	}//assert addr max
  
	MSS_SD_CreateCMD(sd,SD_CMD24,addr,0xFF);
    
	uint8_t temp_rx=0;
	uint8_t SD_write_count;
	uint8_t tx_buff [515];//total data is 515 bytes (FE user_data FF FF)
	tx_buff[0]=0xFE;
	tx_buff[513]=0xFF;
	tx_buff[514]=0xFF;

	memmove(tx_buff+1,tx_data,512);

	for(SD_write_count=0;SD_write_count<10;SD_write_count++){
		MSS_SD_SendCmd(sd,sd->cmd_buff,6);
		if(MSS_SD_GetR1Response(sd,0x00)) break;
	}
	if(SD_write_count==10){
		sd->enum_state=ENUM_WRITE_ERROR;
		return;
  	}//ERROR check
  
	MSS_SD_SendCmd(sd,tx_buff,515);

	if(MSS_SD_GetWriteResponse(sd)){//Data Response token
	do{
		MSS_SPI_transfer_block(
			sd->mss_spi,
			0,
			0,
			&temp_rx,
			1,
			1
		);
	}while(!(temp_rx==0xFF));
	}
	else{
		sd->enum_state=ENUM_WRITE_ERROR;
		return;
	}//write error need to check status CMD13 //if need ,need to add the statement

	if(MSS_SD_AddrCheck(sd->c_size,addr,0)){
		memmove(sd->write_addr,addr,4);//update the write addr
	}
	
	sd->enum_state=ENUM_INIT_SUCCESS;
}

/***************************************************************************//**
* MSS_SD_ReadBlock()
* See "mss_sd.h" for details of how to use this function.
*/
void MSS_SD_ReadBlock(mss_sd_state_t * sd,const uint8_t * addr,uint8_t * rx_data,uint8_t init_use)
{
	if(MSS_SD_AddrCheck(sd->c_size,addr,1) && !init_use){
		sd->enum_state=ENUM_ADDR_OUT_OF_RANGE;
		return;
	}//assert addr max
  
	MSS_SD_CreateCMD(sd,SD_CMD17,addr,0xFF);
	uint8_t SD_read_count=0,ReadResponse=0;

	uint8_t read_CRC [2] ;

	for(SD_read_count=0;SD_read_count<10;SD_read_count++){
		MSS_SD_SendCmd(sd,sd->cmd_buff,6);
		if(MSS_SD_GetR1Response(sd,0x00)) break;
	}
	if(SD_read_count==10){
		sd->enum_state=ENUM_READ_ERROR;
		return;
	}

	for(SD_read_count=0;SD_read_count<100;SD_read_count++){
		ReadResponse=MSS_SD_GetReadResponse(sd);
		if(ReadResponse==1) break;
		else if(ReadResponse==2){
			sd->enum_state=ENUM_ADDR_OUT_OF_RANGE;
			return;
		}//Data Error Response 
	}
	if(SD_read_count==100){
		sd->enum_state=ENUM_READ_ERROR;
		return;
	}

	MSS_SPI_transfer_block(
		sd->mss_spi,
		0,
		0,
		rx_data,
		512,
		1
	);

	MSS_SPI_transfer_block(
		sd->mss_spi,
		0,
		0,
		read_CRC,
		2,
		1
	);

	if(MSS_SD_AddrCheck(sd->c_size,addr,0)){
		memmove(sd->read_addr,addr,4);// update the read addr
	}
	sd->enum_state=ENUM_INIT_SUCCESS;
}

/***************************************************************************//**
* MSS_PDMA_SendToSbandReg()
* See "mss_sd.h" for details of how to use this function.
*/
uint8_t MSS_PDMA_SendToSbandReg(pdma_channel_id_t pdma_channel, uint8_t* tx_data)
{
	uint16_t count=0;

	MSS_SPI_disable( &g_mss_spi1 );
	MSS_SPI_set_transfer_byte_count( &g_mss_spi1, 512 );
	PDMA_start( pdma_channel, (uint32_t)tx_data, PDMA_SPI1_TX_REGISTER, 512 );
	MSS_SPI_enable( &g_mss_spi1 );
  
	while ( !MSS_SPI_tx_done( &g_mss_spi1) && count<10000)
	{
		count++;
	}

	if(count==10000) return 0;
	else return 1;
}

/***************************************************************************//**
* MSS_SD_Close()
* See "mss_sd.h" for details of how to use this function.
** Need to Set Before Flight
*/
void MSS_SD_Close(mss_sd_state_t * sd)
{
	uint8_t arg[4] = {0x00,0x00,0x00,0x00};
	
	uint8_t sat_inform[512];
	memset(sat_inform,0,512);

	MSS_SD_CreateSatInform(sd,sat_inform);
	for(sd->init_count=0;sd->init_count<10;sd->init_count++){
		arg[3]=sd->init_count;
		MSS_SD_WriteBlock(sd,arg,sat_inform);
	}
	if(sd->enum_state<0){
		sd->enum_state=ENUM_CLOSE_ERROR;
		return;
	}//update the satellite information in sd card

	arg[3]=0x00;
	MSS_SD_CreateCMD(sd,SD_ACMD13,arg,0xFF);

	MSS_SD_SendCmd(sd,sd->cmd_buff,6);
	MSS_SD_GetR1Response(sd,0x00);

	MSS_SD_PowerOff(sd);
	sd->mss_spi->slaves_cfg[sd->slave].clk_gen = (uint8_t)(0x13); // ** need to set when main freq is 50MHz
	sd->enum_state=ENUM_IDLE;

}

/***************************************************************************//**
* MSS_SD_PowerOff()
* See "mss_sd.h" for details of how to use this function.
*/
void MSS_SD_PowerOff(mss_sd_state_t * sd)
{
	MSS_GPIO_set_output( sd->enableGpioPin , 0 );
}

/***************************************************************************//**
* MSS_SD_PowerOn()
* See "mss_sd.h" for details of how to use this function.
*/
void MSS_SD_PowerOn(mss_sd_state_t * sd)
{
	MSS_GPIO_set_output( sd->enableGpioPin , 1 );
}


/***************************************************************************//**
  Internal function
/***************************************************************************//**
/***************************************************************************//**
* SD card finite state machine 
* (used for initial the sd card include all cmd and acmd before write or read)
** Need to Set before Flight
*/
static void MSS_SD_fsm(mss_sd_state_t * sd)
{

	switch(sd->enum_state){
		case ENUM_IDLE:
			break;
		case ENUM_DISABLE://PowerOff SD card, whatever happen
			MSS_SD_PowerOff(sd);
			delay(SD_POWER_WAIT_TIME_MS);//delay 500 ms
			sd->enum_state++;
			break;
		case ENUM_ENABLE://PowerOn SD card, and set the SPI speed to 500khz
			MSS_SD_PowerOn(sd);
			delay(SD_POWER_WAIT_TIME_MS);//delay 500 ms
			sd->mss_spi->hw_reg->CLK_GEN = (uint32_t)(0x63); // ** need to set when main freq is 50MHz
			sd->enum_state++;
			break;
		case ENUM_DUMMY_CLK:{
			uint8_t master_dmmy[10]= {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
			MSS_SD_SendCmd(sd,master_dmmy,10);//tx dummy data to sd card
			delay(SD_INIT_CMD_DELAY);//delay 10 ms
			sd->enum_state++;
			break;
		}
		case ENUM_SEND_GET_CMD0:{// send cmd0 (0x40) to sd card
			uint8_t arg[4]={0x00,0x00,0x00,0x00};
			MSS_SD_CreateCMD(sd,SD_CMD0,arg,0x95);//arg={0,0,0,0}
			MSS_SD_SendCmd(sd,sd->cmd_buff,6);
			if(MSS_SD_GetR1Response(sd,0x01)) sd->enum_state++;
			sd->init_count++;
			break;
		} 
		case ENUM_SEND_GET_CMD8:{// send cmd8 (0x48) to sd card
			uint8_t arg[4]={0x00,0x00,0x01,0xAA};
			MSS_SD_CreateCMD(sd,SD_CMD8,arg,0x87);//arg={0,0,01,AA}
			MSS_SD_SendCmd(sd,sd->cmd_buff,6);
			if(MSS_SD_GetR1Response(sd,0x01)) sd->enum_state++;
			MSS_SD_GetOtherResponse(sd,4);
			sd->init_count++;
			break;
		} 
		case ENUM_SEND_GET_CMD58_FIRST:{// send cmd58 (0x7A) to sd card
			uint8_t arg[4]={0x00,0x00,0x00,0x00};
			MSS_SD_CreateCMD(sd,SD_CMD58,arg,0xFF);//arg={0,0,0,0}
			MSS_SD_SendCmd(sd,sd->cmd_buff,6);
			if(MSS_SD_GetR1Response(sd,0x00)) sd->enum_state++;
			MSS_SD_GetOtherResponse(sd,4);
			sd->init_count++;
			break;
		} 
		case ENUM_SPI_CLK_SET_FAST:
			sd->mss_spi->hw_reg->CLK_GEN = (uint32_t)((sd->spiClockRate/2)-1); // ** need to set when main freq is 50MHz
			sd->enum_state++;
			break;
		case ENUM_SEND_GET_CMD55_ACMD41:{
			uint8_t arg[4]={0x00,0x00,0x00,0x00};
			uint8_t arg1 [4]= {0x40,0x00,0x00,0x00};
			MSS_SD_CreateCMD(sd,SD_CMD55,arg,0xFF);//arg={0,0,0,0}
			MSS_SD_SendCmd(sd,sd->cmd_buff,6);
			if(!MSS_SD_GetR1Response(sd,0x01)) break;
			MSS_SD_CreateCMD(sd,SD_ACMD41,arg1,0xFF);
			MSS_SD_SendCmd(sd,sd->cmd_buff,6);
			if(MSS_SD_GetR1Response(sd,0x00)) sd->enum_state++;
			else delay(SD_ENTER_READY_WAIT_MS);//delay 100ms
			sd->init_count++;
			break;
		}
		case ENUM_SEND_GET_CMD58_SEC:{
			uint8_t arg[4]={0x00,0x00,0x00,0x00};
			MSS_SD_CreateCMD(sd,SD_CMD58,arg,0xFF); //arg={0,0,0,0}
			MSS_SD_SendCmd(sd,sd->cmd_buff,6);
			if(MSS_SD_GetR1Response(sd,0x00)) sd->enum_state++;
			MSS_SD_GetOtherResponse(sd,4);
			sd->init_count++;
			break;
		}
		case ENUM_INIT_SUCCESS:
			break;
		case ENUM_INIT_ERROR:
			break;
	}
}//CRC7 may need to use

/***************************************************************************//**
* Create SD card command 
*/
static void MSS_SD_CreateCMD(mss_sd_state_t * sd, uint8_t sdCmdCode, const uint8_t * arg, uint8_t crc)
{
	sd->cmd_buff[0]=sdCmdCode;
	memmove(sd->cmd_buff+1,arg,4);
	(crc==0x00)?memset(sd->cmd_buff+5,0xFF,1):memmove((sd->cmd_buff)+5,&crc,1);
}//check if need the arg need to invert

/***************************************************************************//**
* Use designated spi channel to send SD card command 
*/
static void MSS_SD_SendCmd(mss_sd_state_t * sd,uint8_t * cmd_array, uint16_t cmd_size)
{
	MSS_SPI_transfer_block(
		sd->mss_spi,
		cmd_array,
		cmd_size,
		0,
		0,
		1
	);
}

/***************************************************************************//**
* Get the SD card spi mode R1's response
*/
static uint8_t MSS_SD_GetR1Response(mss_sd_state_t * sd, uint8_t check_response)
{
	uint8_t rx=0xFF,count=0;
	do{
		MSS_SPI_transfer_block(
			sd->mss_spi,
			0,
			0,
			&rx,
			1,
			1
		);
		count++;
	}while(!(rx==check_response) && count<15);

	if(rx==check_response) return 1;
	else return 0;
}

/***************************************************************************//**
* Get the SD card spi mode all response except R1
*/
static void MSS_SD_GetOtherResponse(mss_sd_state_t * sd, uint8_t Response_size)
{
	uint8_t R2rx[10];
	MSS_SPI_transfer_block(
		sd->mss_spi,
		0,
		0,
		R2rx,
		Response_size,
		1
	);
}

/***************************************************************************//**
* Get the SD card spi mode data response 
*/
static uint8_t MSS_SD_GetWriteResponse(mss_sd_state_t * sd)
{
	uint8_t rx=0xFF,count=0;
	do{
		MSS_SPI_transfer_block(
			sd->mss_spi,
			0,
			0,
			&rx,
			1,
			1
		);
		count++;
		if((rx&0x0F)==0x05) return 1;
		else if (((rx&0x0F)==0x0B) || ((rx&0x0F)==0x0D)) return 0;
	}while(count<15);

	return 0;
}//Data Response Token P.273

/***************************************************************************//**
* Get the SD card spi mode data error
*/
static uint8_t MSS_SD_GetReadResponse(mss_sd_state_t * sd)
{
	uint8_t rx=0xFF,count=0;
	do{
		MSS_SPI_transfer_block(
			sd->mss_spi,
			0,
			0,
			&rx,
			1,
			1
		);
		count++;
		if(rx==0xFE) return 1;
		else if(rx==0x09) return 2;
		else if(rx&0xF0==0x00) return 0;
	}while(count<15);

	return 0;
}//Data Error Token P.274

/***************************************************************************//**
* Check the designated address is valid
*/
static uint8_t MSS_SD_AddrCheck(uint32_t c_size,const uint8_t* addr,uint8_t mode)
{
	uint32_t addr32=(addr[0] << 24) + (addr[1] << 16) + (addr[2] << 8) + addr[3];
	switch(mode){
		case 0: // checking the addr is information area
			if(addr32>=100) return 1;
			else return 0;
			break;
		case 1: // checking the addr is valid 
			if(addr32>c_size) return 1;
			else return 0;
			break;
	}
}//assert the addr max

/***************************************************************************//**
* Encode the satellite information to store in the SD card 
*/
static void MSS_SD_CreateSatInform(mss_sd_state_t * sd, uint8_t* sat_inform)
{
	uint16_t crc=0;

	sat_inform[0]=SD_Sync0;
	sat_inform[1]=SD_Sync1;
	sat_inform[2]=SD_Sync2;
	sat_inform[3]=sd->first_check;
	sat_inform[4]=sd->fourty_min_delay;
	sat_inform[5]=(sd->c_size)>>24;
	sat_inform[6]=((sd->c_size)<<8)>>24;
	sat_inform[7]=((sd->c_size)<<16)>>24;
	sat_inform[8]=((sd->c_size)<<24)>>24;

	memmove(sat_inform+9,sd->write_addr,4);
	memmove(sat_inform+13,sd->read_addr,4);

	//ncu
	memmove(sat_inform+17,(uint8_t*)(sd->ncu_ecef),4);
	memmove(sat_inform+21,(uint8_t*)(sd->ncu_ecef+1),4);
	memmove(sat_inform+25,(uint8_t*)(sd->ncu_ecef+2),4);
	memmove(sat_inform+29,(uint8_t*)&(sd->ncu_min_elv),4);
	memmove(sat_inform+33,(uint8_t*)&(sd->ncu_max_range),4);
	sat_inform[37]=sd->ncu_active;

	//cu
	memmove(sat_inform+38,(uint8_t*)(sd->cu_ecef),4);
	memmove(sat_inform+42,(uint8_t*)(sd->cu_ecef+1),4);
	memmove(sat_inform+46,(uint8_t*)(sd->cu_ecef+2),4);
	memmove(sat_inform+50,(uint8_t*)&(sd->cu_min_elv),4);
	memmove(sat_inform+54,(uint8_t*)&(sd->cu_max_range),4);
	sat_inform[58]=sd->cu_active;

	//singapore
	memmove(sat_inform+59,(uint8_t*)(sd->ntu_ecef),4);
	memmove(sat_inform+63,(uint8_t*)(sd->ntu_ecef+1),4);
	memmove(sat_inform+67,(uint8_t*)(sd->ntu_ecef+2),4);
	memmove(sat_inform+71,(uint8_t*)&(sd->ntu_min_elv),4);
	memmove(sat_inform+75,(uint8_t*)&(sd->ntu_max_range),4);
	sat_inform[79]=sd->ntu_active;

	//india
	memmove(sat_inform+80,(uint8_t*)(sd->iist_ecef),4);
	memmove(sat_inform+84,(uint8_t*)(sd->iist_ecef+1),4);
	memmove(sat_inform+88,(uint8_t*)(sd->iist_ecef+2),4);
	memmove(sat_inform+92,(uint8_t*)&(sd->iist_min_elv),4);
	memmove(sat_inform+96,(uint8_t*)&(sd->iist_max_range),4);
	sat_inform[100]=sd->iist_active;

	crc=crc_16(sat_inform,100);
	memmove(sat_inform+101,(uint8_t*)&crc,2);

}

/***************************************************************************//**
* Decode the satellite information from SD card
*/
static uint8_t MSS_SD_GetSatInform(mss_sd_state_t * sd, uint8_t * sat_inform)
{
	if(!(sat_inform[0]==SD_Sync0 && sat_inform[1]==SD_Sync1 && sat_inform[2]==SD_Sync2)){
		sd->enum_state=ENUM_GetSat_ERROR;
		return 0;
	}

	uint16_t crc=0,raw_crc=0;

	sd->first_check=sat_inform[3];
	sd->fourty_min_delay=sat_inform[4];
	sd->c_size=(sat_inform[5]<<24) + (sat_inform[6]<<16) + (sat_inform[7]<<8) + sat_inform[8];

	memmove(sd->write_addr,sat_inform+9,4);
	memmove(sd->read_addr,sat_inform+13,4);

	//ncu
	memmove((uint8_t*)(sd->ncu_ecef),sat_inform+17,4);
	memmove((uint8_t*)(sd->ncu_ecef+1),sat_inform+21,4);
	memmove((uint8_t*)(sd->ncu_ecef+2),sat_inform+25,4);
	memmove((uint8_t*)&(sd->ncu_min_elv),sat_inform+29,4);
	memmove((uint8_t*)&(sd->ncu_max_range),sat_inform+33,4);
	sd->ncu_active=sat_inform[37];

	//cu
	memmove((uint8_t*)(sd->cu_ecef),sat_inform+38,4);
	memmove((uint8_t*)(sd->cu_ecef+1),sat_inform+42,4);
	memmove((uint8_t*)(sd->cu_ecef+2),sat_inform+46,4);
	memmove((uint8_t*)&(sd->cu_min_elv),sat_inform+50,4);
	memmove((uint8_t*)&(sd->cu_max_range),sat_inform+54,4);
	sd->cu_active=sat_inform[58];

	//singapore
	memmove((uint8_t*)(sd->ntu_ecef),sat_inform+59,4);
	memmove((uint8_t*)(sd->ntu_ecef+1),sat_inform+63,4);
	memmove((uint8_t*)(sd->ntu_ecef+2),sat_inform+67,4);
	memmove((uint8_t*)&(sd->ntu_min_elv),sat_inform+71,4);
	memmove((uint8_t*)&(sd->ntu_max_range),sat_inform+75,4);
	sd->ntu_active=sat_inform[79];

	//india
	memmove((uint8_t*)(sd->iist_ecef),sat_inform+80,4);
	memmove((uint8_t*)(sd->iist_ecef+1),sat_inform+84,4);
	memmove((uint8_t*)(sd->iist_ecef+2),sat_inform+88,4);
	memmove((uint8_t*)&(sd->iist_min_elv),sat_inform+92,4);
	memmove((uint8_t*)&(sd->iist_max_range),sat_inform+96,4);
	sd->iist_active=sat_inform[100];

	memmove((uint8_t*)&raw_crc,sat_inform+101,2);

	crc=crc_16(sat_inform,100);
	if(raw_crc==crc){
		sd->enum_state=ENUM_INIT_SUCCESS;
		return 1;
	}
	else{
		sd->enum_state=ENUM_GetSat_ERROR;
		return 0;
	}


	

}

/***************************************************************************//**
	Ground Use
/***************************************************************************//**
/***************************************************************************//**
* All write all read check
*/
void MSS_SD_First_Check(mss_sd_state_t * sd)
{
	uint32_t C_size=0,test_count;
	uint32_t break_block[1000],idx=0;

	MSS_SD_Init(sd);
	if(sd->enum_state==ENUM_INIT_ERROR) return;

	C_size=MSS_SD_SizeCheck(sd);

	uint8_t tx_test[512];
	for(test_count=0;test_count<512;test_count++) tx_test[test_count]=0xFF;

	uint8_t rx_test[512];
	for(test_count=0;test_count<512;test_count++) rx_test[test_count]=0x00;

	for(test_count=0;test_count<1000;test_count++) break_block[test_count]=0;

	uint8_t addr[4];
	// for(test_count=111002394;test_count<(C_size+1)*1000;test_count++){
	// 	addr[0]=test_count>>24;       addr[1]=(test_count<<8)>>24;
	// 	addr[2]=(test_count<<16)>>24;  addr[3]=(test_count<<24)>>24;
	// 	MSS_SD_WriteBlock(sd,addr,tx_test);
	// 	if(sd->enum_state==ENUM_WRITE_ERROR) break;
	// }//test_count=88999812

	//test_count<(C_size+1)*1000
	for(test_count=88999812;test_count<(C_size+1)*1000;test_count++){
		addr[0]=test_count>>24;       addr[1]=(test_count<<8)>>24;
		addr[2]=(test_count<<16)>>24;  addr[3]=(test_count<<24)>>24;
		MSS_SD_ReadBlock(sd,addr,rx_test,0);

		if(!MSS_Break_Block_Check(rx_test)){
		break_block[idx]=test_count;
		idx++;
	}
    if(idx==1000 || sd->enum_state==ENUM_READ_ERROR) break;
	}

	return ;

}

/***************************************************************************//**
* Store initial satellite information
** Need to Use Before Flight
** Need to Set Before Flight
*/
void MSS_SD_FirstUse(mss_sd_state_t * sd)
{
	uint8_t arg1[4]={0x00,0x00,0x00,0x03};// arg1 is start address to earse
	uint8_t arg2[4]={0x00,0x00,0x00,0x04};// arg2 is end address to earse
	uint8_t arg3[4]={0x00,0x00,0x00,0x00};
	uint8_t rx=0x00;

	sd->enum_state=ENUM_DISABLE;
	for(sd->init_count=0;sd->init_count<200;){
		MSS_SD_fsm(sd);
		if(sd->enum_state==ENUM_INIT_SUCCESS) break;
	}
	if(sd->init_count==200 && sd->enum_state!=ENUM_INIT_SUCCESS){
		sd->enum_state=ENUM_INIT_ERROR;
		return;
	}

	MSS_SD_CreateCMD(sd,SD_CMD32,arg1,0xFF); //arg={0,0,0,0}
	MSS_SD_SendCmd(sd,sd->cmd_buff,6);
	MSS_SD_GetR1Response(sd,0x00);

	MSS_SD_CreateCMD(sd,SD_CMD33,arg2,0xFF); //arg={0,0,0,0}
	MSS_SD_SendCmd(sd,sd->cmd_buff,6);
	MSS_SD_GetR1Response(sd,0x00);

	MSS_SD_CreateCMD(sd,SD_CMD38,arg3,0xFF); //arg={0,0,0,0}
	MSS_SD_SendCmd(sd,sd->cmd_buff,6);
	MSS_SD_GetR1Response(sd,0x00);

	
	do{
		MSS_SPI_transfer_block(
			sd->mss_spi,
			0,
			0,
			&rx,
			1,
			1
		);
	}while(rx==0x00);


	uint8_t sat_inform[512];
	memset(sat_inform,0,512);

	sd->c_size=(MSS_SD_SizeCheck(sd)+1)*1000;
	sd->write_addr[2]=0x01;
	sd->read_addr[2]=0x01;

	sd->ncu_ecef[0]=-2996.224469;
	sd->ncu_ecef[1]=4949.303906;
	sd->ncu_ecef[2]=2676.139201;

	sd->ncu_min_elv=0;
	sd->ncu_max_range=0;
	sd->ncu_active=1;

	sd->cu_ecef[0]=-1266.639515;
	sd->cu_ecef[1]=-4727.163024;
	sd->cu_ecef[2]=4079.002292;

	sd->cu_min_elv=5;
	sd->cu_max_range=0;
	sd->cu_active=0;


	sd->ntu_ecef[0]=-1521.017154;
	sd->ntu_ecef[1]=6192.472593;
	sd->ntu_ecef[2]=143.735144;

	sd->ntu_min_elv=10;
	sd->ntu_max_range=1407.2;
	sd->ntu_active=0;


	sd->iist_ecef[0]=1429.821545;
	sd->iist_ecef[1]=6144.292032;
	sd->iist_ecef[2]=936.494637;

	sd->iist_min_elv=5;
	sd->iist_max_range=1694.5;
	sd->iist_active=0;

	MSS_SD_CreateSatInform(sd,sat_inform);
	for(sd->init_count=0;sd->init_count<10;sd->init_count++){
		arg3[3]=sd->init_count;
		MSS_SD_WriteBlock(sd,arg3,sat_inform);
	}
	if(sd->enum_state<0){
		sd->enum_state=ENUM_CLOSE_ERROR;
		return;
	}//update the satellite information in sd card

	return;
}//write the satellite information to two sd card

/***************************************************************************//**
* Checking all data is correct when all write all read 
*/
uint8_t MSS_Break_Block_Check(uint8_t* test_data)
{
	uint16_t test_data_count;
	for(test_data_count=0;test_data_count<512;test_data_count++){
		if(test_data[test_data_count]!=0xFF) break;
	}

	return (test_data_count==512);
}

/***************************************************************************//**
* Get the SD card CSD register and get the SD card maximum size from CSD
*/
uint32_t MSS_SD_SizeCheck(mss_sd_state_t * sd)
{
	uint8_t SD_SizeCheck_Count;
	uint8_t arg[4]={0x00,0x00,0x00,0x00};
	uint8_t read_CSD[16],read_CRC [2] ;

	MSS_SD_CreateCMD(&sd[0],SD_CMD9,arg,0xFF);
    
	for(SD_SizeCheck_Count=0;SD_SizeCheck_Count<10;SD_SizeCheck_Count++){
		MSS_SD_SendCmd(&sd[0],sd->cmd_buff,6);
		if(MSS_SD_GetR1Response(sd,0x00)) break;
	}
	for(SD_SizeCheck_Count=0;SD_SizeCheck_Count<100;SD_SizeCheck_Count++){
		if(MSS_SD_GetR1Response(sd,0xFE)) break;
	}

	MSS_SPI_transfer_block(
		sd->mss_spi,
		0,
		0,
		read_CSD,
		16,
		1
	);

	MSS_SPI_transfer_block(
		sd->mss_spi,
		0,
		0,
		read_CRC,
		2,
		1
	);


	return ((read_CSD[7] & 0x3F) << 16) + (read_CSD[8] << 8) + read_CSD[9];

}

#ifdef __cplusplus
}
#endif

#endif /* MSS_SD_C_*/
