/**
  ******************************************************************************
  * @file    main.c
  * @author  yanglx2022
  * @version V0.1
  * @date    2019/10/15
  * @brief   U盘BootLoader(升级模式下连接电脑被识别为U盘,向U盘拷贝固件实现升级)
  ******************************************************************************
  */

#include "usb_hw.h"
#include "ff.h" 
#include "flash.h"
#include "usart.h"
#include "delay.h"

// 升级引脚
#define GPIO_UPDATE				GPIOB
#define GPIO_PIN_UPDATE		GPIO_Pin_0

// 升级指示灯
#define GPIO_LED					GPIOA
#define GPIO_PIN_LED			GPIO_Pin_4

// 应用程序地址
#define APPLICATION_ADDRESS		0x08004800

// U盘只读状态(拷贝升级文件时丢弃所有写操作防止主机修改文件)
extern uint8_t udisk_read_only;

// 文件系统
FATFS fs;
FIL file;

// 固件缓存
uint8_t firmware[FLASH_SECTOR_SIZE];

uint8_t delay_cnt = 0;

// 升级指示LED初始化
void Led_Init()
{
	// 使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	// 设置引脚
  GPIO_InitTypeDef  GPIO_InitStructure;  
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_LED;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIO_LED, &GPIO_InitStructure);
}

// 判断是否启动升级
uint8_t IsUpdate()
{
	// 使能GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	// 设置引脚
  GPIO_InitTypeDef  GPIO_InitStructure;  
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_UPDATE;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIO_UPDATE, &GPIO_InitStructure);
	// 判断引脚状态
	if (GPIO_ReadInputDataBit(GPIO_UPDATE, GPIO_PIN_UPDATE) == 0)
	{
		Delay_Ms(100);
		if (GPIO_ReadInputDataBit(GPIO_UPDATE, GPIO_PIN_UPDATE) == 0)
		{
			Delay_Ms(100);
			if (GPIO_ReadInputDataBit(GPIO_UPDATE, GPIO_PIN_UPDATE) == 0)
			{
				return 1;		// 升级键按下需要升级
			}
		}
	}
	return 0;
}

// 跳转到应用程序
void Jump_to_Application(void)
{
	// 关SysTick(不关闭的话如果App中没有SysTick_Handler会造成设置中断向量表时程序异常卡住)
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	// 关闭总中断
	__disable_irq();
	#ifdef DEBUG
	printf("sp=%x\n",*(__IO uint32_t*)APPLICATION_ADDRESS);
	#endif
	// 判断堆栈地址合法(RAM为20K)
	if (((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFFA000) == 0x20000000)
	{
		// 设置APP堆栈指针
		__set_MSP(*(__IO uint32_t*)APPLICATION_ADDRESS);
		#ifdef DEBUG
		printf("jump %x\n", *(__IO uint32_t*)(APPLICATION_ADDRESS + 4));
		#endif
		// 跳转到APP
		((void(*)())*(__IO uint32_t*)(APPLICATION_ADDRESS + 4))();
	}
}

int main(void)
{
	#ifdef DEBUG
	Debug_Init();
	printf("Bootloader init\n");
	#endif
	
	Delay_Init();
	
	if (IsUpdate()) // 升级键按下
	{
		// 升级U盘模式
		USB_HW_Init();
		USB_Init();
		#ifdef DEBUG
		printf("usb wait\n");
		#endif
		while (bDeviceState != CONFIGURED);
		#ifdef DEBUG
		printf("usb ok\n");
		#endif
		Led_Init();
		
		// 查询固件拷贝
		while(1)
		{
			Delay_Ms(100);
			delay_cnt++;
			if (delay_cnt % 10 == 0)	// 每1s查询一次
			{
				delay_cnt = 0;
				f_mount(&fs, "", 1);
				FRESULT fr = f_open(&file, "Keyboard.bin", FA_READ);
				if (fr == FR_OK)
				{
					udisk_read_only = 1;
					uint32_t length = f_size(&file);
					if (length % 2 > 0)
					{
						length++;
					}
					uint32_t bytes_read;
					uint32_t bytes_to_write = length;
					uint32_t address = APPLICATION_ADDRESS;
					while(length > 0)
					{
						if (bytes_to_write >= FLASH_SECTOR_SIZE)
						{
							bytes_to_write = FLASH_SECTOR_SIZE;
						}
						f_read(&file, firmware, bytes_to_write, &bytes_read);
						#ifdef DEBUG
						printf("bytes_to_write = %d, bytes_read = %d\n", bytes_to_write, bytes_read);
						#endif
						Flash_Write(address, firmware, bytes_to_write);
						length -= bytes_to_write;
						address += bytes_to_write;
						bytes_to_write = length;
					}
					f_close(&file);
					f_unlink("Keyboard.bin");
					f_mount(0, "", 1);
					udisk_read_only = 0;
					break;
				}
				f_mount(0, "", 1);
			}
			// LED闪烁
			if (delay_cnt % 2 == 0)
			{
				GPIO_SetBits(GPIO_LED, GPIO_PIN_LED);
			}
			else
			{
				GPIO_ResetBits(GPIO_LED, GPIO_PIN_LED);
			}
		}
		GPIO_ResetBits(GPIO_LED, GPIO_PIN_LED);
	}
	#ifdef DEBUG
	printf("ready to jump\n");
	#endif
	USB_Connect(FALSE);
	Jump_to_Application();
	
  while(1);
}
