/*
 * Dev_Info.c
 *
 *  Created on: Jan 07, 2025
 *      Author: illi Eisenberg
 */

#include "Dev_Inf.h"
#include "main.h"
#include "W25Qxx.h"

/*************************************************
 *
 * List of Device Sizes in Bytes:
 * {Winbond W25Q}
 *
 *  512 kBit	( 64 kByte)	    65536	0x10000
 *    1 MBit	(128 kByte)	   131072	0x20000
 *    2 MBit	(256 kByte)	   262144	0x40000
 *    4 MBit	(512 kByte)	   524288	0x80000
 *    8 MBit	(  1 MByte)	  1048576	0x100000
 *   16 MBit	(  2 MByte)	  2097152	0x200000
 *   32 MBit	(  4 MByte)	  4194304	0x400000
 *   64 MBit	(  8 MByte)	  8388608	0x800000	**
 *  128 MBit	( 16 MByte)	 16777216	0x1000000
 *  256 MBit	( 32 MByte)	 33554432	0x2000000
 *  512 MBit	( 64 MByte)	 67108864	0x4000000
 *    1 GBit	(128 MByte)	134217728	0x8000000
 *    2 GBit	(256 MByte)	268435456	0x10000000

*************************************************/

/***** NOTE: this file is for W25Q64 device *****/
#define MEMORY_FLASH_SIZE 0x800000

/* This structure contains information used by ST-LINK Utility to program and erase the device */
#if defined (__ICCARM__)
__root struct StorageInfo const StorageInfo  =  {
#else
struct StorageInfo __attribute__((section(".Dev_info"))) /*const*/ StorageInfo  = {
#endif
		"W25Q64_Blue-Pill-A_SPI1", 	 	         // Device Name + version number
		SPI_FLASH,                  		 // Device Type
		0x90000000,                			 // Device Start Address
		MEMORY_FLASH_SIZE,                 	 // Device Size in Bytes
		MEMORY_PAGE_SIZE,                    // Programming Page Size
		0xFF,                                // Initial Content of Erased Memory

		// Specify Size and Address of Sectors (view example below)
		{
				{ 0x800,  			// Sector Numbers,
				  0x1000 },        	//Sector Size

				{ 0x00000000, 0x00000000 }
		}
};
