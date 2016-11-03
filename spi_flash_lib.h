/**
 * ----------------------------------------------------------------------------
 * FILE:	spi_flash_lib.h
 * DESCRIPTION:	SPI flash library
 * DATE:	2016.10.27
 * AUTHOR(s):	Lime Microsystems
 * REVISION: v0r1
 * ----------------------------------------------------------------------------
 */
#ifndef _SPI_FLASH_LIB_H_
#define _SPI_FLASH_LIB_H_

void CyFxSpiFastRead(CyBool_t v);
CyU3PReturnStatus_t FlashSpiInit();
CyU3PReturnStatus_t FlashSpiDeInit();
CyU3PReturnStatus_t FlashSpiTransfer(uint16_t pageAddress, uint16_t byteCount, uint8_t *buffer, CyBool_t isRead);
CyU3PReturnStatus_t FlashSpiEraseSector(CyBool_t isErase, uint8_t sector);

CyU3PReturnStatus_t FX3_FlashSpiWaitForStatus(void);
CyU3PReturnStatus_t FX3_FlashSpiTransfer(uint16_t pageAddress, uint16_t byteCount, uint8_t *buffer, CyBool_t isRead);
CyU3PReturnStatus_t FX3_FlashSpiEraseSector(CyBool_t isErase, uint8_t sector);

#endif
