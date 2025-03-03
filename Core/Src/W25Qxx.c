/*
 * W25Qxx.c
 *
 *  Created on: Jan 07, 2025
 *      Author: illi Eisenberg
 */


#include "main.h"
#include "W25Qxx.h"
#include "math.h"

extern SPI_HandleTypeDef hspi1;
#define W25Q_SPI hspi1

W25Qx_Parameter W25Q_Param;



/************** HAL based functions **************/

/*************************************************
 * The following function are based on the HAL
 * commands.
 * You can implement them by your own and continue
 * using the rest of the code.
 *
 *************************************************/


/*************************************************
 * functions for CS:
 *
 *************************************************/
void csLOW(void)
{
	HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}


void csHIGH(void)
{
	HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}


/*************************************************
 * Read and write bytes to SPI.
 *
 * @param data: the byte or array to handle
 * @param len: number of bytes to handle
 *
 *************************************************/
void SPI_Write (uint8_t *data, uint8_t len)
{
	HAL_SPI_Transmit(&W25Q_SPI, data, len, 2000);
}


void SPI_Read (uint8_t *data, uint32_t len)
{
	HAL_SPI_Receive(&W25Q_SPI, data, len, 5000);
}


/*************************************************
 * Debug functions to turn an LED on and off
 * Here it is inGPIO-C, Pin-13
 *
 *************************************************/
void LED_ON (void)
{
	HAL_GPIO_WritePin(GPIOC, LED_Pin, RESET);
}


void LED_OFF (void)
{
	HAL_GPIO_WritePin(GPIOC, LED_Pin, SET);
}


/*************************************************
 * A Delay function
 *
 * @param msec: the delay in mili-seconds
 *
 *************************************************/
void W25Q_Delay (uint32_t msec)
{
	HAL_Delay(msec);
}


/*************************************************
 * Calculate the rest of the bytes to write,
 * since the data can be spread over a few pages
 * in the memory.
 *
 *
 *************************************************/
// TODO this function can be uint8_t size
uint32_t bytesToWrite(uint32_t size , uint16_t offset)
{
	if ( (size + offset) < 256)
		return size;
	else
		return 256 - offset;
}


/*************************************************
 * Read ticks
 *
 *************************************************/
uint32_t get_tick(void)
{
	return HAL_GetTick();
}


/*************************************************
 * Data Watchpoint Trigger initialization
 *
 * This init is executed in the chip initialization:
 * W25Q_Init(void)  function.
 *
 *************************************************/
void DWT_Delay_Init(void)
{
    // Enable DWT counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable access to DWT registers
    DWT->CYCCNT = 0;                                 // Reset the counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Enable the cycle counter
}


/*************************************************
 * Create a delay in µs using DWT register.
 * (Data Watchpoint Trigger)
 *
 * See: Debug support (DBG) in the Reference Manual
 *
 * Number of clocks in a second, divided in 1,000,000
 * gives the number of clocks in 1µs.
 * Multiply it in the number of desired µs to get
 * the desired delay.
 *
 *************************************************/
void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}


/*************************************************
 * find the remaining bytes in the current sector:
 *
 * This is a support function for func W25Q_Write().
 *
 *************************************************/
uint32_t bytesToModify (uint32_t size , uint16_t offset)
{
	if ( (size + offset) < 4096)
		return size;
	else
		return 4096 - offset;
}


/*************************************************
 * Initialization:
 *
 * Reset the device and get the device configuration
 *
 *************************************************/
uint8_t W25Q_Init(void)
{
	uint8_t state = 0;

	W25Q_Reset();

	W25Q_Delay(5);

	state = W25Q_Get_Param(&W25Q_Param);

	DWT_Delay_Init();

	return state;
}


/*************************************************
 * Resets the device.
 * Need to send 2 commands to avoid accidental reset
 *
 *************************************************/
void W25Q_Reset (void)
{
	uint8_t tData[2];
	tData[0] = W25Q_Enable_Reset;
	tData[1] = W25Q_RReset;

	csLOW();
	SPI_Write (tData, 2);
	csHIGH();
	W25Q_Delay(100);
}


/*************************************************
 * Get the device ID.
 *
 *
 *************************************************/
uint8_t W25Q_ReadID (void)
{

	uint8_t tData = JEDEC_ID;
	uint8_t rData[3] = {0};

	csLOW();

	SPI_Write(&tData, 1);
	SPI_Read(rData, 3);

	csHIGH();

	if (rData[0] == WINBONS_SER_FLASH)
		return rData[2];	// Device ID
	else
		return (W25Qx_ERROR);
}


/*************************************************
 * Get chip parameters:
 *
 * The chip size is calculated here for general
 * work with the firmware.
 *
 * When using the chip as External Loader, please
 * change the "MEMORY_FLASH_SIZE" definition in
 * "Dev_Info.c" to your desired value.
 *
 *************************************************/
uint8_t W25Q_Get_Param(W25Qx_Parameter *Param)
{
	uint8_t id =0;

	id = W25Q_ReadID ();
	if (id == W25Qx_ERROR)
		return W25Qx_ERROR;
	else
	{
		Param->FLASH_Id = id;	// see W25Qxx.h

		Param->FLASH_Size = pow(2 , (id - 0x14)) * 1024 * 1024;	// size in bytes

		Param->PAGE_SIZE = MEMORY_PAGE_SIZE;							//  256 bytes
		Param->SECTOR_SIZE = MEMORY_SECTOR_SIZE;						// 4096 bytes
		Param->BLOCK_SIZE = MEMORY_BLOCK_SIZE;							//  64K Bytes

		Param->PAGE_QUANTITY = Param->FLASH_Size / Param->PAGE_SIZE;	// 32768 (W25Q64)
		Param->SECTOR_QUANTITY = Param->PAGE_QUANTITY / 16;				//  2048 (W25Q64)
		Param->BLOCK_QUANTITY = Param->SECTOR_QUANTITY / 16;			//   128 (W25Q64)

		return W25Qx_OK;
	}
}


/*************************************************
 * Read status register 1:
 *
 * Currently returns BUSY or OK
 *
 * S0: when executing program, Erase, write to SR1, R/W to Security register
 * S1:0 when executing a Write Enable Instruction
 * S3: provide Write Protection control and status
 * S4: provide Write Protection control and status
 * S5: provide Write Protection control and status
 *
 *************************************************/
uint8_t W25Q_Read_Status_Register_1(void)
{
	uint8_t tData = STATUS_REG_1_RD;
	uint8_t status = 0;

	csLOW();

	SPI_Write(&tData, 1);	// send query
	SPI_Read(&status, 1);	// read the register

	csHIGH();

	if((status & W25Qxx_FSR_BUSY) != 0)
	{
		return W25Qx_BUSY;
	}
	else
	{
		return W25Qx_OK;
	}
}


/*************************************************
 * Read data from the device.
 *
 * 24נbit address = 16,777,215 bytes = 134,777,728 bits = 128Mb
 * for W25Q 512Kb...128Mb: address width is 24bits
 * for W25Q 256Kb.....2Gb: address width is 32bits
 *
 * @param startPage: page number where the data starts
 * @param offset   : offset on the page where the data starts
 * @param size     : amount of bytes to be read
 * @param *rData   : pointer to the data to be read
 *
 *************************************************/
void W25Q_Read (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[5];
	// calculate address:
	uint32_t memAddr = (startPage * 256) + offset;

	// TODO
	// you cannot read more bytes than available
	// if capacity - memAddr < size then return an error.....

	if (W25Q_Param.BLOCK_QUANTITY < 512)		// chip size < 256Mb
	{
		tData[0] = READ_DATA_3Badd;
		tData[1] = (memAddr >> 16) & 0xFF;	// MSB
		tData[2] = (memAddr >>  8) & 0xFF;
		tData[3] = (memAddr      ) & 0xFF;	// LSB
	}
	else
	{
		tData[0] = READ_DATA_4Badd;
		tData[1] = (memAddr >> 24) & 0xFF;	// MSB
		tData[2] = (memAddr >> 16) & 0xFF;
		tData[3] = (memAddr >>  8) & 0xFF;
		tData[4] = (memAddr      ) & 0xFF;	// LSB
	}


	csLOW();

	SPI_Write(tData , W25Q_Param.BLOCK_QUANTITY < 512 ? 4 : 5);

	SPI_Read(rData , size);

	csHIGH();
}


/*************************************************
 * Read data from the device.
 *
 * 24נbit address = 16,777,215 bytes = 134,777,728 bits = 128Mb
 * for W25Q 512Kb...128Mb: address width is 24bits
 * for W25Q 256Kb.....2Gb: address width is 32bits
 *
 *************************************************/
void W25Q_Read_Fast (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	// calculate address:
	uint32_t memAddr = (startPage * 256) + offset;

	// TODO
	// you cannot read more bytes than available
	// if capacity - memAddr < size then return an error.....

	if (W25Q_Param.BLOCK_QUANTITY < 512)		// chip size < 256Mb
	{
		tData[0] = READ_DATA_FAST_3Badd;
		tData[1] = (memAddr >> 16) & 0xFF;	// MSB
		tData[2] = (memAddr >>  8) & 0xFF;
		tData[3] = (memAddr      ) & 0xFF;	// LSB
		tData[4] = 0;	                    // dummy clock
	}
	else
	{
		tData[0] = READ_DATA_FAST_4Badd;
		tData[1] = (memAddr >> 24) & 0xFF;	// MSB
		tData[2] = (memAddr >> 16) & 0xFF;
		tData[3] = (memAddr >>  8) & 0xFF;
		tData[4] = (memAddr      ) & 0xFF;	// LSB
		tData[5] = 0;	                    // dummy clock
	}


	csLOW();

	SPI_Write(tData , W25Q_Param.BLOCK_QUANTITY < 512 ? 5 : 6);

	SPI_Read(rData , size);

	csHIGH();
}


/*************************************************
 * Wait for Flash finish writing.
 *
 * WATCH IT: this function shall run while CS is activated!!!
 *
 * (CS-> LOW) --> (Operate) --> (W25Q_Wait_For_Write) --> (CS->HIGH)
 *
 *************************************************/
uint8_t W25Q_Wait_For_Write()
{
	uint8_t tData = STATUS_REG_1_RD;
	uint32_t tickstart = get_tick();

	csLOW();
	delay_us(2);
	SPI_Write(&tData , 1);

	// Check for OK or Timeout
	do
	{
		delay_us(5);
		SPI_Read(&tData, 1);
#ifdef DEBUG_ON
		LED_ON();
#endif

	}
	while ((tData & W25Qx_BUSY) && ((get_tick() - tickstart) < W25Qxx_TIMEOUT));

#ifdef DEBUG_ON
		LED_OFF();
#endif
	csHIGH();

	if ((tData & W25Qxx_FSR_BUSY) != 0)
	{
		return W25Qx_BUSY;
	}
	else
	{
		return W25Qx_OK;
	}
}


/*************************************************
 * Write enable:
 *
 * Enables the device to write data into it.
 *
 *************************************************/
uint8_t W25Q_Write_Enable (void)
{
	uint8_t tData = WRITE_ENABLE;
	uint32_t tickstart = get_tick();

	csLOW();		// chip select - LOW
	SPI_Write(&tData, 1);
	csHIGH();		// chip select - HIGH

	// Wait for Flash finish writing
	while(W25Q_Read_Status_Register_1() == W25Qx_BUSY)
	{
		// Check for Timeout
		if((get_tick() - tickstart) > W25Qxx_TIMEOUT)
		{
				return W25Qx_TIMEOUT;
		}
		W25Q_Delay(1);
	}
	return W25Qx_OK;
}


/*************************************************
 * Write disable:
 *
 * Disables the device from write data into it.
 *
 *************************************************/
void W25Q_Write_Disable (void)
{
	uint8_t tData = WRITE_DISABLE;

	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	W25Q_Delay(1);
}


/*************************************************
 * Erase a sector:
 * the minimum segment that can be erased.
 *
 * each sector contains 16 pages
 * each page contains 256 bytes
 *
 *************************************************/
void W25Q_Erase_Sector (uint16_t numSector)
{
	uint8_t tData[5] = {0};
	uint32_t memAddr = numSector * 16 * 256;	 // each sector contains 16 pages × 256 bytes

	W25Q_Write_Enable();

	if (W25Q_Param.BLOCK_QUANTITY < 512)		// chip size < 256Mb
	{
		tData[0] = SECTOR_ERASE_3Badd;
		tData[1] = (memAddr >> 16) & 0xFF;	// MSB
		tData[2] = (memAddr >>  8) & 0xFF;
		tData[3] = (memAddr      ) & 0xFF;	// LSB
	}
	else
	{
		tData[0] = SECTOR_ERASE_4Badd;
		tData[1] = (memAddr >> 24) & 0xFF;	// MSB
		tData[2] = (memAddr >> 16) & 0xFF;
		tData[3] = (memAddr >>  8) & 0xFF;
		tData[4] = (memAddr      ) & 0xFF;	// LSB
	}

	csLOW();
	SPI_Write(tData, W25Q_Param.BLOCK_QUANTITY < 512 ? 4 : 5);
	csHIGH();

	// wait for sector erase
	W25Q_Wait_For_Write();

//	W25Q_Delay(400);	// Sector erase time = 45ms (MAX 400ms)
	W25Q_Write_Disable();

}


/*************************************************
 * Erase a 64KB block:
 *
 *
 *************************************************/
void W25Q_Erase_Block_64KB (uint32_t numBlock)
{
	uint8_t tData[5] = {0};
	uint32_t memAddr = numBlock * 16 * 16 * 256;	 // each sector contains 16 pages × 256 bytes

	W25Q_Write_Enable();

	if (W25Q_Param.BLOCK_QUANTITY < 512)		// chip size < 256Mb
	{
		tData[0] = BLOCK_ERASE_3Badd;
		tData[1] = (memAddr >> 16) & 0xFF;	// MSB
		tData[2] = (memAddr >>  8) & 0xFF;
		tData[3] = (memAddr      ) & 0xFF;	// LSB
	}
	else
	{
		tData[0] = BLOCK_ERASE_4Badd;		// 256Mb or more
		tData[1] = (memAddr >> 24) & 0xFF;	// MSB
		tData[2] = (memAddr >> 16) & 0xFF;
		tData[3] = (memAddr >>  8) & 0xFF;
		tData[4] = (memAddr      ) & 0xFF;	// LSB
	}

	csLOW();
	SPI_Write(tData, W25Q_Param.BLOCK_QUANTITY < 512 ? 4 : 5);
	csHIGH();

	// wait for sector erase
	W25Q_Wait_For_Write();

//	W25Q_Delay(400);	// Sector erase time = 45ms (MAX 400ms)
	W25Q_Write_Disable();

}


/*************************************************
 * Erase the whole chip:
 *
 *
 *************************************************/
void W25Q_Erase_Chip (void)
{
	uint8_t tData = CHIP_ERASE;

	W25Q_Write_Enable();

	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();

	// wait for chip erase
	W25Q_Wait_For_Write();

	W25Q_Write_Disable();
}


/*************************************************
 * Program a page:
 * This function calculates the number of needed sectors
 * for writing data depending page address and offset,
 * erase them, and write new data.
 *
 * @param page : the page we want to write
 * @param offset : the offset on the page
 * @param size : the size of the data
 * @param *data ; pointer to the data itself
 *
 * Ex: startPage = 1 , offset = 250 , size = 8
 *     (8+250-1)v=v257 --> 257/256 = 1
 *     endPage = 1 + 1 = 2
 *
 *************************************************/

void W25Q_Write_Page_Clean (uint32_t page , uint16_t offset , uint32_t size , uint8_t *data)
{
	uint8_t tData[256+4+1];		// 266
	uint32_t startPage = page;
	uint32_t endPage   = startPage + ( (size + offset - 1) / 256 );
	uint32_t numPages = endPage - startPage + 1;

	// must erase the page first, but whole sector must be erased
	// erase all the needed sectors:
	uint16_t startSector = startPage / 16;	// 16 pages per sector
	uint16_t endSector = endPage / 16;
	uint16_t numSectors = endSector - startSector + 1;

	for (uint16_t i=0 ; i < numSectors ; i++)
	{
		W25Q_Erase_Sector(startSector++);
	}

	// prepare the data to be send:
	uint32_t dataPosition = 0;

	// write the data
	for (uint32_t i = 0 ; i < numPages ; i++)
	{
		uint32_t memAddr = (startPage * 256) + offset;
		uint16_t bytesRemaning = bytesToWrite (size , offset);
		uint32_t indx = 0;

		W25Q_Write_Enable();

		if (W25Q_Param.BLOCK_QUANTITY < 512)		// chip size < 256Mb
		{
			tData[0] = PAGE_PROGRAM_3Badd;
			tData[1] = (memAddr >> 16) & 0xFF;	// MSB
			tData[2] = (memAddr >>  8) & 0xFF;
			tData[3] = (memAddr      ) & 0xFF;	// LSB

			indx = 4;
		}
		else
		{
			tData[0] = PAGE_PROGRAM_4Badd;
			tData[1] = (memAddr >> 24) & 0xFF;	// MSB
			tData[2] = (memAddr >> 16) & 0xFF;
			tData[3] = (memAddr >>  8) & 0xFF;
			tData[4] = (memAddr      ) & 0xFF;	// LSB

			indx = 5;
		}

		uint16_t bytesToSend = bytesRemaning + indx;

		// prepare up to 1 page of data
		for (uint16_t i = 0 ; i < bytesRemaning ; i++)		// "bytesremaining" is a variable smaller then 257
		{
			tData[indx++] = data[i + dataPosition];
		}

		if (bytesToSend > 250)
		{
			csLOW();
			SPI_Write(tData, 100);
			SPI_Write(tData+100, bytesToSend-100);
			csHIGH();
		}
		else
		{
			csLOW();
			SPI_Write(tData, bytesToSend);
			csHIGH();
		}


		startPage++;
		offset = 0;
		size = size - bytesRemaning;
		dataPosition = dataPosition + bytesRemaning;

//		W25Q_Delay(5);
		W25Q_Wait_For_Write();
		W25Q_Write_Disable();

	}
}


/*************************************************
 * Update data in an address:
 *
 * This function ONLY update some data in a page
 * without erasing the other data in the sector.
 *
 * Steps:
 * 1. Copy the data from the sector and store it in the RAM
 * 2. Modify the data as per the requirement
 * 3. Erase the sector
 * 4. Write the modified sector data back to the sector
 *
 *
 * @param page : the page we want to write, where the write will start from
 * @param offset : the  offset on the first page. This can vary between 0 to 255
 * @param size : the size of the data
 * @param *data ; pointer to the data itself
 *
 * Ex: startPage = 1 , offset = 250 , size = 8
 *     (8+250-1)v=v257 --> 257/256 = 1
 *     endPage = 1 + 1 = 2
 *
 *************************************************/

void W25Q_Write (uint32_t page , uint16_t offset , uint32_t size , uint8_t *data)
{
	// focusing sectors rather then pages...
	uint16_t startSector = page / 16;	// 16 pages per sector
	uint16_t endSector = (page + ( (offset + size - 1) / 256) ) / 16;
	uint16_t numSectors = endSector - startSector + 1;

	uint8_t previousData[4096];		// store the sector needed to be updated
	uint32_t sectorOffset = ((page % 16) * 256) + offset;		// start byte address inside the start sector
	uint32_t dataindx = 0;		// monitor the position in the data array


	for (uint16_t i =0 ; i < numSectors ; i++)
	{
		// copy the sector we want to modify:
		uint32_t startPage = startSector * 16;
		W25Q_Read_Fast(startPage, 0, 4096, previousData);

		// Modify the data:
		// keep how many bytes we can modify in the current sector:
		uint16_t bytesRemaining = bytesToModify(size , sectorOffset);

		for (uint16_t i =0 ; i < bytesRemaining ; i++)
		{
			previousData[i + sectorOffset] = data[i + dataindx];
		}

		// Erase the sector AND:
		// Write the modified sector data:
		W25Q_Write_Page_Clean(startPage, 0, 4096, previousData);

		startSector++;
		sectorOffset = 0;
		dataindx = dataindx + bytesRemaining;
		size = size - bytesRemaining;

	}
}


/*************************************************
 * Read a single byte of data.
 *
 * 24נbit address = 16,777,215 bytes = 134,777,728 bits = 128Mb
 * for W25Q 512Kb...128Mb: address width is 24bits
 * for W25Q 256Kb.....2Gb: address width is 32bits
 *
 *************************************************/
uint8_t W25Q_Read_Byte (uint32_t Addr)
{
	uint8_t tData[5];
	uint8_t rData = 0;

	// TODO
	// you cannot read more bytes than available
	// if capacity - memAddr < size then return an error.....

	if (W25Q_Param.BLOCK_QUANTITY < 512)		// chip size < 256Mb
	{
		tData[0] = READ_DATA_3Badd;
		tData[1] = (Addr >> 16) & 0xFF;	// MSB
		tData[2] = (Addr >>  8) & 0xFF;
		tData[3] = (Addr      ) & 0xFF;	// LSB
	}
	else
	{
		tData[0] = READ_DATA_4Badd;
		tData[1] = (Addr >> 24) & 0xFF;	// MSB
		tData[2] = (Addr >> 16) & 0xFF;
		tData[3] = (Addr >>  8) & 0xFF;
		tData[4] = (Addr      ) & 0xFF;	// LSB
	}


	csLOW();

	SPI_Write(tData , W25Q_Param.BLOCK_QUANTITY < 512 ? 4 : 5);

	SPI_Read(&rData , 1);	// read the byte

	csHIGH();

	return rData;
}



/*************************************************
 * Write a single byte of data.
 *
 * 24נbit address = 16,777,215 bytes = 134,777,728 bits = 128Mb
 * for W25Q 512Kb...128Mb: address width is 24bits
 * for W25Q 256Kb.....2Gb: address width is 32bits
 *
 *************************************************/
void W25Q_Write_Byte (uint32_t Addr , uint8_t data)
{
	uint8_t tData[6];

	// TODO
	// you cannot read more bytes than available
	// if capacity - memAddr < size then return an error.....

	if (W25Q_Param.BLOCK_QUANTITY < 512)		// chip size < 256Mb
	{
		tData[0] = PAGE_PROGRAM_3Badd;
		tData[1] = (Addr >> 16) & 0xFF;	// MSB
		tData[2] = (Addr >>  8) & 0xFF;
		tData[3] = (Addr      ) & 0xFF;	// LSB
		tData[4] = data;
	}
	else
	{
		tData[0] = PAGE_PROGRAM_4Badd;
		tData[1] = (Addr >> 24) & 0xFF;	// MSB
		tData[2] = (Addr >> 16) & 0xFF;
		tData[3] = (Addr >>  8) & 0xFF;
		tData[4] = (Addr      ) & 0xFF;	// LSB
		tData[5] = data;
	}

	if (W25Q_Read_Byte(Addr) == 0xFF)
	{
		W25Q_Write_Enable();

		csLOW();
		SPI_Write(tData , W25Q_Param.BLOCK_QUANTITY < 512 ? 5 : 6);
		csHIGH();

//		W25Q_Delay(5);
		W25Q_Wait_For_Write();
		W25Q_Write_Disable();
	}
}


/************** Write/Read Integers, floats and 32 bit data **************/

uint8_t tempBytes[4] = {0};

/*************************************************
 * Convert float / integer number to a 4-bytes array
 * using union operator.
 *
 * @param *ftoa_bytes_arr: pointer to the array to store the float number as bytes.
 * @param float_variable: float number to be converted.
 *
 * @retval: None
 *
 *************************************************/
void float2Bytes (uint8_t * ftoa_bytes_arr , float float_variable)
{
	union
	{
		float a;
		uint8_t bytes[4];
	}thing;

	// load the field with the float number
	thing.a = float_variable;

	// split it to 4 bytes
	for (uint8_t i = 0 ; i < sizeof(int) ; i++)
	{
		ftoa_bytes_arr[i] = thing.bytes[i];
	}
}


/*************************************************
 * Convert a 4-bytes array to a float / integer number
 * using union operator.
 *
 * @param *ftoa_bytes_arr: pointer to the array storing the float number as bytes.
 *
 * @retval: converted float number
 *
 *************************************************/
float Bytes2float (uint8_t * ftoa_bytes_arr)
{
	union
	{
		float a;
		uint8_t bytes[4];
	}thing;

	// load the field with the 4-bytes array
	for (uint8_t i = 0 ; i < sizeof(int) ; i++)
	{
		thing.bytes[i] = ftoa_bytes_arr[i];
	}

	// return the array as a float number (4-bytes)
	float float_variable = thing.a;
	return float_variable;
}


/*************************************************
 * Write a number (int or float) to the memory.
 *
 * @param page :   page number to write to
 * @param offset : offset on the page
 * @param size :   the size of the data
 * @param *data :  data to write
 *
 * @retval: None
 *
 *************************************************/
void W25Q_Write_NUM (uint32_t page , uint16_t offset , float data)
{
	float2Bytes (tempBytes , data);

	// using Sector Update function
	W25Q_Write(page, offset, 4, tempBytes);
}


/*************************************************
 * Read a number (int or float) from the memory.
 *
 * @param page :   page number to read from
 * @param offset : offset on the page
 *
 * @retval: float number converted from bytes.
 *
 *************************************************/
float W25Q_Read_NUM (uint32_t page , uint16_t offset)
{
	uint8_t rData[4];
	W25Q_Read(page, offset, 4, rData);

	return Bytes2float(rData);
}


/*************************************************
 * Write a 32bit array data to the memory.
 *
 * @param page :   page number to write to
 * @param offset : offset on the page
 * @param size :   the size of the data
 * @param *data :  data to write
 *
 * @retval: None
 *
 *************************************************/
void W25Q_Write_32B_Array (uint32_t page , uint16_t offset , uint32_t size , uint32_t * data)
{
	uint8_t data8[size * 4];
	uint32_t indx = 0;

	for (uint32_t i = 0 ; i < size ; i++)
	{
		data8[indx++] = (data[i]      ) & 0xFF;		// LSB
		data8[indx++] = (data[i] >>  8) & 0xFF;
		data8[indx++] = (data[i] >> 16) & 0xFF;
		data8[indx++] = (data[i] >> 24) & 0xFF;
	}

	W25Q_Write(page, offset, indx, data8);
}


/*************************************************
 * Read a 32bit array data from the memory.
 *
 * @param page :   page number to read from
 * @param offset : offset on the page
 *
 * @param size :   the size of the data
 * @param *data :  data to read
 *
 * @retval: None --> pass by reference
 *
 *************************************************/
void W25Q_Read_32B_Array (uint32_t page , uint16_t offset , uint32_t size , uint32_t * data)
{
	uint8_t data8[size * 4];
	uint32_t indx = 0;

	W25Q_Read(page, offset, size * 4, data8);

	for (uint32_t i = 0 ; i < size ; i++)
	{
		uint8_t b0 = data8[indx++];
		uint8_t b1 = data8[indx++];
		uint8_t b2 = data8[indx++];
		uint8_t b3 = data8[indx++];
		data[i] = (uint32_t)b0 | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);
	}
}



/******** Functions for External Flash Loader ********/


void flash_WriteMemory(uint8_t* buffer, uint32_t address, uint32_t buffer_size)
{
	uint32_t page = address/256;
	uint16_t offset = address%256;
	uint32_t size = buffer_size;
	uint8_t tData[266];
	uint32_t startPage = page;
	uint32_t endPage  = startPage + ((size+offset-1)/256);
	uint32_t numPages = endPage-startPage+1;

	uint32_t dataPosition = 0;

	// write the data
	for (uint32_t i = 0 ; i < numPages ; i++)
	{
		uint32_t memAddr = (startPage * 256) + offset;
		uint16_t bytesRemaning  = bytesToWrite(size, offset);
		uint32_t indx = 0;

		W25Q_Write_Enable();

		if (W25Q_Param.BLOCK_QUANTITY < 512)		// chip size < 256Mb
		{
			tData[0] = PAGE_PROGRAM_3Badd;
			tData[1] = (memAddr >> 16) & 0xFF;	// MSB
			tData[2] = (memAddr >>  8) & 0xFF;
			tData[3] = (memAddr      ) & 0xFF;	// LSB

			indx = 4;
		}
		else
		{
			tData[0] = PAGE_PROGRAM_4Badd;
			tData[1] = (memAddr >> 24) & 0xFF;	// MSB
			tData[2] = (memAddr >> 16) & 0xFF;
			tData[3] = (memAddr >>  8) & 0xFF;
			tData[4] = (memAddr      ) & 0xFF;	// LSB

			indx = 5;
		}

		uint16_t bytesToSend  = bytesRemaning + indx;

		// prepare up to 1 page of data
		for (uint16_t i = 0 ; i < bytesRemaning ; i++)		// "bytesremaining" is a variable smaller then 257
		{
			tData[indx++] = buffer[i + dataPosition];
		}

		if (bytesToSend > 250)
		{
			csLOW();
			SPI_Write(tData, 100);
			SPI_Write(tData+100, bytesToSend-100);
			csHIGH();
		}
		else
		{
			csLOW();
			SPI_Write(tData, bytesToSend);
			csHIGH();
		}


		startPage++;
		offset = 0;
		size = size-bytesRemaning;
		dataPosition = dataPosition + bytesRemaning;

		W25Q_Wait_For_Write();
		W25Q_Write_Disable();

	}

}

void flash_ReadMemory (uint32_t Addr, uint32_t Size, uint8_t* buffer)
{
	uint32_t page = Addr/256;
	uint16_t offset = Addr%256;

	W25Q_Read(page, offset, Size, buffer);
}

void flash_SectorErase(uint32_t EraseStartAddress, uint32_t EraseEndAddress)
{
	uint16_t startSector  = EraseStartAddress/4096;
	uint16_t endSector  = EraseEndAddress/4096;
	uint16_t numSectors = endSector-startSector+1;
	for (uint16_t i=0; i<numSectors; i++)
	{
		W25Q_Erase_Sector(startSector++);
	}
}

void flash_ChipErase (void)
{
	W25Q_Erase_Chip();
}

void flash_Reset (void)
{
	W25Q_Reset();
}
