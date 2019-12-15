/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "flash.h"

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	
	for(;count>0;count--)
	{
		Flash_Read(sector * FLASH_SECTOR_SIZE + USER_FLASH_BASE, buff, FLASH_SECTOR_SIZE);
		sector++;
		buff+=FLASH_SECTOR_SIZE;
	}
	return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
#if FF_FS_READONLY == 0
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	for(;count>0;count--)
	{
		Flash_Write(sector * FLASH_SECTOR_SIZE + USER_FLASH_BASE, buff, FLASH_SECTOR_SIZE);
		sector++;
		buff+=FLASH_SECTOR_SIZE;
	}
	return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	switch(cmd)
	{
		case CTRL_SYNC:
			res = RES_OK; 
			break;	 
		case GET_SECTOR_SIZE:
			*(DWORD*)buff = FLASH_SECTOR_SIZE; 
			res = RES_OK;
			break;	 
		case GET_BLOCK_SIZE:
		*(WORD*)buff = FLASH_SECTOR_SIZE;
			res = RES_OK;
			break;	 
		case GET_SECTOR_COUNT:
			*(DWORD*)buff = USER_SECTOR_COUNT;
			res = RES_OK;
			break;
		default:
			res = RES_PARERR;
			break;
	}
	return res;
}

// User defined function to give a current time to fatfs module
// 31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31)                                                                                                                                                                                                                                   
// 15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2)
DWORD get_fattime (void)
{
	return 0;
}

#endif

