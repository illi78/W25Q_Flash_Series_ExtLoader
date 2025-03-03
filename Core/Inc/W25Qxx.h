/*
 * W25Qxx.h
 *
 *  Created on: Jan 07, 2025
 *      Author: illi Eisenberg
 */

#ifndef INC_W25QXX_H_
#define INC_W25QXX_H_

// debug option (comment this line to disable the debug option
#define DEBUG_ON

// Standard definitions for the W25Qxx series (512k ~ 2G)
#define MEMORY_BLOCK_SIZE				0x10000   /*  blocks of 64KBytes */
#define MEMORY_SECTOR_SIZE				0x1000    /* 4kBytes */
#define MEMORY_PAGE_SIZE				0x100     /* 256 bytes */

// Device status:
#define W25Qx_OK             0x00
#define W25Qx_ERROR          0x01
#define W25Qx_BUSY           0x02
#define W25Qx_TIMEOUT        0x03


// Device ID:
#define WINBONS_SER_FLASH    0xEF	//
#define W25Q05               0x10	// 512k		    256 p	16
#define W25Q10               0x11	//   1M		    512 p	17
#define W25Q20               0x12	//   2M		   1024 p	18
#define W25Q40               0x13	//   4M		   2048 p	19
#define W25Q80               0x14	//   8M		   4096 p	20
#define W25Q16               0x15	//  16M		   8192 p	21
#define W25Q32               0x16	//  32M		  16384 p	22
#define W25Q64               0x17	//  64M		  32768 p	23
#define W25Q128              0x18	// 128M		  65536 p	24
#define W25Q256              0x19	// 256M		 131072 p	25
#define W25Q512              0x20	// 512M		 262144 p	26
#define W25Q01               0x21	//   1G		 524288 p	27
#define W25Q02               0x22	//   2G		1048576 p	28

// sector     = 16 pages = 4096 bytes
// numPAGES   = 2 ^ (DEV_SIZE_ID - 8)
// numSECTORS = numPAGES / 16
// numBLOCK   = numSECTORS / 16

// there are 128 pages block, and 256 pages block

typedef struct
{
	uint8_t FLASH_Id;			// 17
	uint32_t FLASH_Size;		// in bytes
	uint32_t BLOCK_SIZE;		// 0x10000 (65536)
	uint32_t SECTOR_SIZE;		// 0x1000  (4096)
	uint32_t PAGE_SIZE;			// 0x100   (256)


	uint32_t PAGE_QUANTITY	;	//
	uint32_t SECTOR_QUANTITY;	//
	uint32_t BLOCK_QUANTITY;	//
}W25Qx_Parameter;

extern W25Qx_Parameter W25Qx_Para;


#define STATUS_REG_1_RD      0x05
#define STATUS_REG_2_RD      0x35
#define STATUS_REG_3_RD      0x15

#define W25Q_WIP             0x01	// Write In Progress

// Reset Operations
#define W25Q_Enable_Reset    0x66
#define W25Q_RReset          0x99
#define W25Q_POWER_DOWN      0xB9	// to implement

// Identification Operations
#define WINBOND_SERIAL_FLASH 0xEF

#define READ_DEVICE_ID       0x90	// EF17
#define JEDEC_ID             0x9F
#define MEM_TYPE             0x40
#define MEM_CAPACITY         0x17	// 64Mb

// Write Operations
#define WRITE_ENABLE         0x06
#define WRITE_DISABLE        0x04

// Read Operations
#define READ_DATA_3Badd      0x03	// read using a 3-bytes address
#define READ_DATA_4Badd      0x13	// read using a 4-bytes address
#define READ_DATA_FAST_3Badd 0x0B	// fast read using a 3-bytes address
#define READ_DATA_FAST_4Badd 0x0C	// fast read using a 4-bytes address

// Program Operations
#define PAGE_PROGRAM_3Badd   0x02	// write using a 3-bytes address
#define PAGE_PROGRAM_4Badd   0x12	// write using a 4-bytes address

// Erase Operations
#define SECTOR_ERASE_3Badd   0x20	// erase using a 3-bytes address
#define SECTOR_ERASE_4Badd   0x21	// erase using a 4-bytes address

#define BLOCK_ERASE_3Badd    0xD8	// erase using a 3-bytes address
#define BLOCK_ERASE_4Badd    0xDC	// erase using a 4-bytes address

#define CHIP_ERASE           0xC7	// erase using a 4-bytes address

// Flag Status Register
#define W25Qxx_FSR_BUSY      0x01    // !< busy
#define W25Qxx_FSR_WREN      0x02    // !< write enable
#define W25Qxx_FSR_QE        0x02    // !< quad enable

#define W25Qxx_TIMEOUT       1000





// Functions Definitions:

uint8_t W25Q_Init(void);
void W25Q_Reset (void);
uint8_t W25Q_ReadID (void);
uint8_t W25Q_Get_Param(W25Qx_Parameter *Param);

uint8_t W25Q_Read_Status_Register_1(void);

void W25Q_Read (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
void W25Q_Read_Fast (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);

void W25Q_Erase_Sector (uint16_t numSector);
void W25Q_Erase_Block_64KB (uint32_t numBlock);
void W25Q_Erase_Chip (void);

void W25Q_Write_Page_Clean (uint32_t page , uint16_t offset , uint32_t size , uint8_t *data);
void W25Q_Write (uint32_t page , uint16_t offset , uint32_t size , uint8_t *data);

uint8_t W25Q_Read_Byte (uint32_t Addr);
void W25Q_Write_Byte (uint32_t Addr , uint8_t data);

void W25Q_Write_NUM (uint32_t page , uint16_t offset , float data);
float W25Q_Read_NUM (uint32_t page , uint16_t offset);

void W25Q_Write_32B_Array (uint32_t page , uint16_t offset , uint32_t size , uint32_t * data);
void W25Q_Read_32B_Array (uint32_t page , uint16_t offset , uint32_t size , uint32_t * data);


// Auxiliary functions
void csLOW(void);
void csHIGH(void);

void SPI_Write (uint8_t *data, uint8_t len);
void SPI_Read (uint8_t *data, uint32_t len);

void W25Q_Delay (uint32_t msec);

uint32_t bytesToWrite(uint32_t size , uint16_t offset);
uint32_t get_tick(void);
void DWT_Delay_Init(void);
void delay_us(uint32_t us);
uint32_t bytesToModify (uint32_t size , uint16_t offset);

uint8_t W25Q_Wait_For_Write();

uint8_t W25Q_Write_Enable (void);
void W25Q_Write_Disable (void);


// Although we already have the functions to Read, Write, erase, etc,
// the loader uses a different set of parameters for them.

// Functions for use by the External Flash Loader source file
void flash_WriteMemory(uint8_t* buffer, uint32_t address, uint32_t buffer_size);
void flash_ReadMemory (uint32_t Addr, uint32_t Size, uint8_t* buffer);
void flash_SectorErase(uint32_t EraseStartAddress, uint32_t EraseEndAddress);
void flash_ChipErase (void);
void flash_Reset (void);


#endif /* INC_W25QXX_H_ */


/*
16Mb version:
-------------
1 page = 256bytes
8192 pages total
1 sector = 16 pages = 4096 bytes (4KB)
1 block = 128 pages -->  64 blocks
1 block = 256 pages -->  32 blocks    <==

1 block = 128 pages =  8 sectors = 32768 bytes (32KB)
1 block = 256 pages = 16 sectors = 65536 bytes (64KB)

512 sectors
32 blocks
32 blocks × 16 sectors × 16 pages × 256 bytes × 8bits = 16,777,216 bits
32 blocks × 65536 bytes = 2,097,152 bytes = 2MB


64Mb version:
-------------
1 page = 256bytes
32768 pages total
1 sector = 16 pages = 4096 bytes (4KB)
1 block = 128 pages --> 256 blocks
1 block = 256 pages --> 128 blocks    <==

1 block = 128 pages =  8 sectors = 32768 bytes (32KB)
1 block = 256 pages = 16 sectors = 65536 bytes (64KB)

2048 sectors
128 blocks
128 blocks × 16 sectors × 16 pages × 256 bytes × 8bits = 67,108,864 bits
128 blocks × 65536 bytes = 8,388,608 bytes = 8MB


128Mb version:
-------------
1 page = 256bytes
65536 pages total
1 sector = 16 pages = 4096 bytes (4KB)
1 block = 128 pages --> 512 blocks
1 block = 256 pages --> 256 blocks    <==

1 block = 128 pages =  8 sectors = 32768 bytes (32KB)
1 block = 256 pages = 16 sectors = 65536 bytes (64KB)

4096 sectors
256 blocks

256 blocks × 16 sectors × 16 pages × 256 bytes × 8bits = 134,217,728 bits
256 blocks × 65536 bytes = 16,777,216 bytes = 16MB

*/
