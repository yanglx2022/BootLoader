[说明]
U盘升级Bootloader

STM32F103C8T6

按着升级按键启动被电脑识别为U盘，向U盘内拷入固件实现升级

LED指示：USB加载过程中两LED同时闪烁，成功识别为U盘后两LED常亮，固件升级过程LED流水灯

升级完成后自动跳转到新固件运行

Flash空间分配：Bootloader + Application + file system + firmware file
(17k + 12k + 23k + 12k = 64k, 应用程序固件不能大于12k)








