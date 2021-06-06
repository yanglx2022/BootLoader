#ifndef __flash_H
#define __flash_H

#include "stm32f10x.h"

// FLASH容量(K)
#define FLASH_SIZE			64
// 扇区大小(byte)
#if FLASH_SIZE < 256
	#define FLASH_SECTOR_SIZE 	1024
#else 
	#define FLASH_SECTOR_SIZE		2048
#endif	

// 用户区设置(非代码区域)
#define USER_SECTOR_COUNT			35
#define USER_FLASH_BASE				(0x08000000 + (FLASH_SIZE - USER_SECTOR_COUNT) * 1024)		// 起始地址


// 从指定地址开始读数据
void Flash_Read(uint32_t address, uint8_t* buffer, uint32_t length);
// 从指定地址开始写数据(address与length必须为2的倍数)
void Flash_Write(uint32_t address, const uint8_t* buffer, uint32_t length);


#endif

