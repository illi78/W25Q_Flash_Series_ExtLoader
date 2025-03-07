/*
 * Loader_Src.c
 *
 *  Created on: Jan 07, 2025
 *      Author: illi Eisenberg
 */

#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "W25Qxx.h"
//#include "Loader_SrcX.h"


/*************************************************
 * This file needs to receive the following functions
 * in these specific format (the same names, types,
 * parameters and return values). These functions
 * will be used by STM32CubeProgrammer to manage the
 * external memory and you should place your memory
 * driver inside that, according to the function
 * purpose. These are the functions will need to
 * populate:
 *
 *************************************************/

extern void SystemClock_Config(void);
extern void MX_SPI2_Init(void);
extern void MX_GPIO_Init(void);

#define LOADER_OK	0x1
#define LOADER_FAIL	0x0

//int Write(uint32_t Address, uint32_t Size, uint8_t *buffer);	//	address in BYTES
//int Read(uint32_t Address, uint32_t Size, uint8_t *Buffer);	//	address in BYTES
//int SectorErase (uint32_t EraseStartAddress ,uint32_t EraseEndAddress);	//	address in BYTES
//int MassErase(void);
//
//uint32_t CheckSum(uint32_t StartAddress, uint32_t Size, uint32_t InitVal);
//uint64_t Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size, uint32_t missalignement);


int Init(void)
{
	SystemInit();

	SCB->VTOR = 0x20000000 | 0x200;					// REMOVED THIS

//	HAL_DeInit();									// REMOVED THIS
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();

	__set_PRIMASK(0);	//enable interrupts
	W25Q_Init();	 	//External memory function initialization
	__set_PRIMASK(1);	//disable interrupts

	return LOADER_OK;
}


/*************************************************
 * Description :
 * Write function for the External Loader.
 *
 * @brief   Program memory.
 *
 * @param   Address: Byte address
 * @param   Size   : size of data
 * @param   buffer : pointer to data buffer
 *
 * @retval  LOADER_OK = 1       : Operation succeeded
 * @retval  LOADER_FAIL = 0 : Operation failed
 *************************************************/
int Write(uint32_t Address, uint32_t Size, uint8_t* buffer) {

    __set_PRIMASK(0); //enable interrupts

    // Address -= 0x90000000;
    flash_WriteMemory(buffer, Address, Size);

    __set_PRIMASK(1); //disable interrupts
    return LOADER_OK;
}


/*************************************************
 * Description :
 * Read function for the External Loader.
 *
 * @brief   Read from memory.
 *
 * @param   Address: Byte address
 * @param   Size   : size of data
 * @param   buffer : pointer to data buffer
 *
 * @retval  LOADER_OK = 1       : Operation succeeded
 * @retval  LOADER_FAIL = 0 : Operation failed
 *************************************************/
int Read (uint32_t Address, uint32_t Size, uint8_t* buffer){
    __set_PRIMASK(0); //enable interrupts

	flash_ReadMemory(Address, Size, buffer);

    __set_PRIMASK(1); //disable interrupts
	return 1;
}


/*************************************************
 * Description :
 * Erasing sector function for the External Loader.
 *
 * @brief   Sector erase.
 *
 * @param   EraseStartAddress :  erase start address
 * @param   EraseEndAddress   :  erase end address
 *
 * @retval  LOADER_OK = 1       : Operation succeeded
 * @retval  LOADER_FAIL = 0 : Operation failed
 *************************************************/
int SectorErase(uint32_t EraseStartAddress, uint32_t EraseEndAddress) {

    __set_PRIMASK(0); //enable interrupts

    flash_SectorErase(EraseStartAddress, EraseEndAddress);

    __set_PRIMASK(1); //disable interrupts
    return LOADER_OK;
}


/*************************************************
 * Description :
 * Mass erase of external flash area
 *
 * Optional command - delete in case usage of mass erase is not planed
 * Inputs    :
 *      none
 * outputs   :
 *     none
 * Note: Optional for all types of device
 *************************************************/
int MassErase(void) {

    __set_PRIMASK(0); //enable interrupts

    flash_ChipErase();

    __set_PRIMASK(1); //disable interrupts
    return LOADER_OK;
}


/*************************************************
 * Description :
 * Calculates checksum value of the memory zone
 * Inputs    :
 *      StartAddress  : Flash start address
 *      Size          : Size (in WORD)
 *      InitVal       : Initial CRC value
 * outputs   :
 *     R0             : Checksum value
 * Note: Optional for all types of device
 *************************************************/
uint32_t CheckSum(uint32_t StartAddress, uint32_t Size, uint32_t InitVal) {
    uint8_t missalignementAddress = StartAddress % 4;
    uint8_t missalignementSize = Size;
    int cnt;
    uint32_t Val;

    StartAddress -= StartAddress % 4;
    Size += (Size % 4 == 0) ? 0 : 4 - (Size % 4);

    for (cnt = 0; cnt < Size; cnt += 4) {
        Val = *(uint32_t*) StartAddress;
        if (missalignementAddress) {
            switch (missalignementAddress) {
                case 1:
                    InitVal += (uint8_t) (Val >> 8 & 0xff);
                    InitVal += (uint8_t) (Val >> 16 & 0xff);
                    InitVal += (uint8_t) (Val >> 24 & 0xff);
                    missalignementAddress -= 1;
                    break;
                case 2:
                    InitVal += (uint8_t) (Val >> 16 & 0xff);
                    InitVal += (uint8_t) (Val >> 24 & 0xff);
                    missalignementAddress -= 2;
                    break;
                case 3:
                    InitVal += (uint8_t) (Val >> 24 & 0xff);
                    missalignementAddress -= 3;
                    break;
            }
        } else if ((Size - missalignementSize) % 4 && (Size - cnt) <= 4) {
            switch (Size - missalignementSize) {
                case 1:
                    InitVal += (uint8_t) Val;
                    InitVal += (uint8_t) (Val >> 8 & 0xff);
                    InitVal += (uint8_t) (Val >> 16 & 0xff);
                    missalignementSize -= 1;
                    break;
                case 2:
                    InitVal += (uint8_t) Val;
                    InitVal += (uint8_t) (Val >> 8 & 0xff);
                    missalignementSize -= 2;
                    break;
                case 3:
                    InitVal += (uint8_t) Val;
                    missalignementSize -= 3;
                    break;
            }
        } else {
            InitVal += (uint8_t) Val;
            InitVal += (uint8_t) (Val >> 8 & 0xff);
            InitVal += (uint8_t) (Val >> 16 & 0xff);
            InitVal += (uint8_t) (Val >> 24 & 0xff);
        }
        StartAddress += 4;
    }

    return (InitVal);
}

/*************************************************
 * Description :
 * Verify flash memory with RAM buffer and calculates checksum value of
 * the programmed memory
 *
 * Inputs    :
 *      FlashAddr     : Flash address
 *      RAMBufferAddr : RAM buffer address
 *      Size          : Size (in WORD)
 *      InitVal       : Initial CRC value
 * outputs   :
 *     R0             : Operation failed (address of failure)
 *     R1             : Checksum value
 *
 * Note: Optional for all types of device
 *************************************************/
uint64_t Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size, uint32_t missalignement){
	__set_PRIMASK(0); //enable interrupts

	uint32_t VerifiedData = 0, InitVal = 0;
    uint64_t checksum;
    Size *= 4;

	uint8_t Buffer[2];
	uint32_t posBuf;

	checksum = CheckSum((uint32_t)MemoryAddr + (missalignement & 0xf), Size - ((missalignement >> 16) & 0xF), InitVal);

	while (Size>VerifiedData)
	{
		flash_ReadMemory(MemoryAddr+VerifiedData, 2, Buffer);

		posBuf=0;
		while ((Size>VerifiedData) && (posBuf<1024)) {
			if (Buffer[posBuf] != *((uint8_t*)RAMBufferAddr+VerifiedData))
			{
				__set_PRIMASK(1); //disable interrupts
				return ((checksum<<32) + MemoryAddr+VerifiedData);
			}
			posBuf++;
			VerifiedData++;
		}
	}
	__set_PRIMASK(1); //disable interrupts
	return (checksum<<32);
}
