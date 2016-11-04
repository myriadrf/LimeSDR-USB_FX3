/**
 * ----------------------------------------------------------------------------
 * FILE:	lime_sdr-usb_brd.h
 * DESCRIPTION:	LimeSDR-USB config
 * DATE:	2016.10.21
 * AUTHOR(s):	Lime Microsystems
 * REVISION: v0r3
 * ----------------------------------------------------------------------------
 */

#ifndef _LIMESDR_USB_BRD_H_
#define _LIMESDR_USB_BRD_H_

#include "LMS64C_protocol.h"

//get info
#define DEV_TYPE			LMS_DEV_LIMESDR_USB
#define HW_VER				4 //LimeSDR-USB 1.4
#define EXP_BOARD			EXP_BOARD_UNSUPPORTED

//I2C devices
#define I2C_ADDR_SI5351		0xC0
#define I2C_ADDR_MAX7322	0xDA
#define I2C_ADDR_LM75		0x90
#define I2C_ADDR_SC18IS602B	0x50
#define I2C_ADDR_EEPROM		0xA0

//SC18IS602B SPI SS
#define BRDG_SPI_FPGA_SS	0x01
#define BRDG_SPI_DUMMY_SS	0x08

//FX3 GPIO
#define FX3_MCU_BUSY		25
#define FX3_SPI_FLASH_SS	54
#define FX3_SPI_AS_SS		57
#define BRDG_INT			45

//BRD_SPI map
#define FPGA_SPI_REG_LMS1_LMS2_CTRL  0x13

#define LMS1_SS			0
#define LMS1_RESET		1

#define FPGA_SPI_REG_SS_CTRL  0x12

#define TCXO_ADF_SS		0 //SS0
#define TCXO_DAC_SS		1 //SS1

//FX3 Firmware Flash (M25P40, 4Mbit)
#define FX3_SIZE 					(512*1024)
#define FX3_FLASH_PAGE_SIZE 		0x100 //256 bytes, SPI Page size to be used for transfers
#define FX3_FLASH_SECTOR_SIZE 		0x10000 //256 pages * 256 page size = 65536 bytes
#define FX3_FLASH_CMD_SECTOR_ERASE 	0xD8

//FPGA bitstream Flash (M25P16, 16Mbit)
#define FPGA_SIZE 			1191788 //FPGA Cyclone IV (EP4CE40F23C7N) bitstream (RBF file) size (max) in bytes
#define FLASH_PAGE_SIZE 	0x100 //256 bytes, SPI Page size to be used for transfers
#define FLASH_SECTOR_SIZE 	0x10000 //256 pages * 256 page size = 65536 bytes
//#define FLASH_BLOCK_SIZE	(FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE) //in pages
#define FLASH_CMD_SECTOR_ERASE 0xD8 //depends on flash: 0xD8 or 0x20

#endif
