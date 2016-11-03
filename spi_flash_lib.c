/**
 * ----------------------------------------------------------------------------
 * FILE:	spi_flash_lib.c
 * DESCRIPTION:	SPI flash library
 * DATE:	2016.10.27
 * AUTHOR(s):	Lime Microsystems
 * REVISION: v0r5
 * ----------------------------------------------------------------------------
 */

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3uart.h"
#include "cyu3gpif.h"
#include "cyu3pib.h"
#include "cyu3gpio.h"
#include "pib_regs.h"
#include "cyu3spi.h"
#include "spi_flash_lib.h"
#include "lime_sdr-usb_brd.h"
#include "cyu3i2c.h"

#define CHANGE_SPI_TO_LSB
#define FLASH_STATUS_TIMEOUT 3000 //3000 ms

 uint8_t   I2C_Addr, data, cmd_errors, sc_brdg_data[255];
 CyU3PI2cPreamble_t preamble;

/* SPI initialization for application. */
CyU3PReturnStatus_t FlashSpiInit(void)
{
    CyU3PSpiConfig_t spiConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    // Start the SPI master block.
    CyU3PMemSet((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
    spiConfig.isLsbFirst = CyFalse;
    spiConfig.cpol       = CyFalse;
    spiConfig.ssnPol     = CyFalse;
    spiConfig.cpha       = CyFalse;
    spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
    spiConfig.clock      = 33 *1000000; //33 MHz
    spiConfig.wordLen    = 8;

    status = CyU3PSpiSetConfig(&spiConfig, NULL);

    return status;
}

void Reconfigure_SPI_for_FPGA_PS(void)
{
    CyU3PSpiConfig_t spiConfig;

    CyU3PMemSet ((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
    spiConfig.isLsbFirst = CyTrue;
    spiConfig.cpol       = CyFalse;
    spiConfig.ssnPol     = CyFalse;
    spiConfig.cpha       = CyFalse;
    spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
    spiConfig.clock      = 33 *1000000; //33 MHz
    spiConfig.wordLen    = 8;

    CyU3PSpiSetConfig (&spiConfig, NULL);
}

CyU3PReturnStatus_t FlashSpiDeInit() {
	return CyU3PSpiDeInit();
}

/* Wait for the status response from the SPI flash. */
CyU3PReturnStatus_t CyFxSpiWaitForStatus(void)
{
    uint8_t buf[2], rd_buf[2];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    uint32_t start_ticks = CyU3PGetTime();

    /* Wait for status response from SPI flash device. */
    do {
		if ((CyU3PGetTime()- start_ticks) > FLASH_STATUS_TIMEOUT)
		{
			status = CY_U3P_ERROR_TIMEOUT;
			return status;
		}

    	buf[0] = 0x06;  /* Write enable command. */

    	CyU3PGpioSetValue (FX3_SPI_AS_SS, CyFalse);
        status = CyU3PSpiTransmitWords (buf, 1);
        CyU3PGpioSetValue (FX3_SPI_AS_SS, CyTrue);
        if (status != CY_U3P_SUCCESS) { //SPI WR_ENABLE command failed
            return status;
        }

        buf[0] = 0x05;  /* Read status command */

        CyU3PGpioSetValue (FX3_SPI_AS_SS, CyFalse);
        status = CyU3PSpiTransmitWords(buf, 1);
        if (status != CY_U3P_SUCCESS) { //SPI READ_STATUS command failed
            CyU3PGpioSetValue (FX3_SPI_AS_SS, CyTrue);
            return status;
        }

        status = CyU3PSpiReceiveWords(rd_buf, 2);
        CyU3PGpioSetValue (FX3_SPI_AS_SS, CyTrue);

        if(status != CY_U3P_SUCCESS) { //SPI status read failed
            return status;
        }

    } while ((rd_buf[0] & 1) || (!(rd_buf[0] & 0x2)));

    return CY_U3P_SUCCESS;
}

CyBool_t spiFastRead = CyFalse;
void FlashSpiFastRead(CyBool_t v) {
    spiFastRead = v;
}

/* SPI read / write for programmer application. */
CyU3PReturnStatus_t FlashSpiTransfer(uint16_t pageAddress, uint16_t byteCount, uint8_t *buffer, CyBool_t isRead)
{
    uint8_t location[4];
    uint32_t byteAddress = 0;
    uint16_t pageCount = (byteCount / FLASH_PAGE_SIZE);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    spiFastRead = CyTrue;// force fast read

    if (byteCount == 0) {
        return CY_U3P_SUCCESS;
    }

    if ((byteCount % FLASH_PAGE_SIZE) != 0) {
        return CY_U3P_ERROR_NOT_SUPPORTED;
    }

    byteAddress = pageAddress * FLASH_PAGE_SIZE;

    while (pageCount != 0) {
        location[1] = (byteAddress >> 16) & 0xFF;       /* MS byte */
        location[2] = (byteAddress >> 8) & 0xFF;
        location[3] = byteAddress & 0xFF;               /* LS byte */

        if (isRead) {
            location[0] = 0x03; /* Read command. */

            if (!spiFastRead) {
                status = CyFxSpiWaitForStatus();
                if (status != CY_U3P_SUCCESS)
                    return status;
            }

            CyU3PGpioSetValue (FX3_SPI_AS_SS, CyFalse);
            status = CyU3PSpiTransmitWords(location, 4);

            if (status != CY_U3P_SUCCESS) { //SPI READ command failed
                CyU3PGpioSetValue (FX3_SPI_AS_SS, CyTrue);
                return status;
            }

            status = CyU3PSpiReceiveWords(buffer, FLASH_PAGE_SIZE);
            if (status != CY_U3P_SUCCESS) {
            	CyU3PGpioSetValue (FX3_SPI_AS_SS, CyTrue);
                return status;
            }

            CyU3PGpioSetValue (FX3_SPI_AS_SS, CyTrue);

        } else { /* Write */
            location[0] = 0x02; /* Write command */

            status = CyFxSpiWaitForStatus();
            if (status != CY_U3P_SUCCESS)
                return status;

            CyU3PGpioSetValue (FX3_SPI_AS_SS, CyFalse);

            status = CyU3PSpiTransmitWords(location, 4);
            if (status != CY_U3P_SUCCESS) { //SPI WRITE command failed
                CyU3PGpioSetValue (FX3_SPI_AS_SS, CyTrue);
                return status;
            }

			#ifdef CHANGE_SPI_TO_LSB
            Reconfigure_SPI_for_FPGA_PS ();
			#endif

            status = CyU3PSpiTransmitWords(buffer, FLASH_PAGE_SIZE);

			#ifdef CHANGE_SPI_TO_LSB
            FlashSpiInit ();
			#endif

            if (status != CY_U3P_SUCCESS) {
            	CyU3PGpioSetValue (FX3_SPI_AS_SS, CyTrue);
                return status;
            }

            CyU3PGpioSetValue (FX3_SPI_AS_SS, CyTrue);
        }

        byteAddress += FLASH_PAGE_SIZE;
        buffer += FLASH_PAGE_SIZE;
        pageCount--;

        if (!spiFastRead)
            CyU3PThreadSleep(15);
    }

    return CY_U3P_SUCCESS;
}

/* Function to erase SPI flash sectors. */
CyU3PReturnStatus_t FlashSpiEraseSector(CyBool_t isErase, uint8_t sector)
{
    uint32_t temp = 0;
    uint8_t  location[4], rdBuf[2];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    location[0] = 0x06;  /* Write enable. */

    CyU3PGpioSetValue (FX3_SPI_AS_SS, CyFalse);
    status = CyU3PSpiTransmitWords (location, 1);
    CyU3PGpioSetValue (FX3_SPI_AS_SS, CyTrue);

    if (status != CY_U3P_SUCCESS)
        return status;

    if (isErase)
    {
        location[0] = FLASH_CMD_SECTOR_ERASE; /* Sector erase. */
        temp        = sector * FLASH_SECTOR_SIZE;
        location[1] = (temp >> 16) & 0xFF;
        location[2] = (temp >> 8) & 0xFF;
        location[3] = temp & 0xFF;

        CyU3PGpioSetValue (FX3_SPI_AS_SS, CyFalse);
        status = CyU3PSpiTransmitWords (location, 4);
        CyU3PGpioSetValue (FX3_SPI_AS_SS, CyTrue);
    }

    uint32_t start_ticks = CyU3PGetTime();

    location[0] = 0x05; /* RDSTATUS */
    do {
		if ((CyU3PGetTime()- start_ticks) > FLASH_STATUS_TIMEOUT)
		{
			status = CY_U3P_ERROR_TIMEOUT;
			return status;
		}

    	CyU3PGpioSetValue (FX3_SPI_AS_SS, CyFalse);
        status = CyU3PSpiTransmitWords (location, 1);
        status = CyU3PSpiReceiveWords(rdBuf, 1);
        CyU3PGpioSetValue (FX3_SPI_AS_SS, CyTrue);

    } while(rdBuf[0] & 1);

    return status;
}

// Functions for fx3 flash

/* Wait for the status response from the FX3 SPI flash. */
CyU3PReturnStatus_t FX3_FlashSpiWaitForStatus(void)
{
    uint8_t buf[2], rd_buf[2];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    uint32_t start_ticks = CyU3PGetTime();

    /* Wait for status response from SPI flash device. */
    do {
		if ((CyU3PGetTime()- start_ticks) > FLASH_STATUS_TIMEOUT)
		{
			status = CY_U3P_ERROR_TIMEOUT;
			return status;
		}

    	buf[0] = 0x06;  /* Write enable command. */

        CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyFalse);
        status = CyU3PSpiTransmitWords (buf, 1);
        CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyTrue);
        if (status != CY_U3P_SUCCESS) { //SPI WR_ENABLE command failed
            return status;
        }

        buf[0] = 0x05;  /* Read status command */

        CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyFalse);
        status = CyU3PSpiTransmitWords(buf, 1);
        if (status != CY_U3P_SUCCESS) { //SPI READ_STATUS command failed
            CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyTrue);
            return status;
        }

        status = CyU3PSpiReceiveWords(rd_buf, 2);
        CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyTrue);
        if(status != CY_U3P_SUCCESS) { //SPI status read failed
            return status;
        }

    } while ((rd_buf[0] & 1) || (!(rd_buf[0] & 0x2)));

    return CY_U3P_SUCCESS;
}

/* FX3 SPI flash read / write for programmer application. */
CyU3PReturnStatus_t FX3_FlashSpiTransfer(uint16_t pageAddress, uint16_t byteCount, uint8_t *buffer, CyBool_t isRead)
{
    uint8_t location[4];
    uint32_t byteAddress = 0;
    uint16_t pageCount = (byteCount / FLASH_PAGE_SIZE);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    spiFastRead = CyTrue;// force fast read

    if (byteCount == 0) {
        return CY_U3P_SUCCESS;
    }

    if ((byteCount % FLASH_PAGE_SIZE) != 0) {
        return CY_U3P_ERROR_NOT_SUPPORTED;
    }

    byteAddress = pageAddress * FLASH_PAGE_SIZE;

    while (pageCount != 0) {
        location[1] = (byteAddress >> 16) & 0xFF;       /* MS byte */
        location[2] = (byteAddress >> 8) & 0xFF;
        location[3] = byteAddress & 0xFF;               /* LS byte */

        if (isRead) {
            location[0] = 0x03; /* Read command. */

            if (!spiFastRead) {
                status = FX3_FlashSpiWaitForStatus();
                if (status != CY_U3P_SUCCESS)
                    return status;
            }

            CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyFalse);
            status = CyU3PSpiTransmitWords(location, 4);

            if (status != CY_U3P_SUCCESS) { //SPI READ command failed
                CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyTrue);
                return status;
            }

            status = CyU3PSpiReceiveWords(buffer, FLASH_PAGE_SIZE);
            if (status != CY_U3P_SUCCESS) {
            	CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyTrue);
                return status;
            }

            CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyTrue);
        } else { /* Write */
            location[0] = 0x02; /* Write command */

            status = FX3_FlashSpiWaitForStatus();
            if (status != CY_U3P_SUCCESS)
                return status;

            CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyFalse);
            status = CyU3PSpiTransmitWords(location, 4);
            if (status != CY_U3P_SUCCESS) { //SPI WRITE command failed
                CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyTrue);
                return status;
            }

            status = CyU3PSpiTransmitWords(buffer, FLASH_PAGE_SIZE);

            if (status != CY_U3P_SUCCESS) {
            	CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyTrue);
                return status;
            }

            CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyTrue);
        }

        byteAddress += FLASH_PAGE_SIZE;
        buffer += FLASH_PAGE_SIZE;
        pageCount--;

        if (!spiFastRead)
            CyU3PThreadSleep(15);
    }

    return CY_U3P_SUCCESS;
}

/* Function to erase FX3 SPI flash sectors. */
CyU3PReturnStatus_t FX3_FlashSpiEraseSector(CyBool_t isErase, uint8_t sector)
{
    uint32_t temp = 0;
    uint8_t  location[4], rdBuf[2];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    location[0] = 0x06;  /* Write enable. */

    CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyFalse);
    status = CyU3PSpiTransmitWords (location, 1);
    CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyTrue);
    if (status != CY_U3P_SUCCESS)
        return status;

    if (isErase)
    {
        location[0] = FX3_FLASH_CMD_SECTOR_ERASE; /* Sector erase. */
        temp        = sector * FLASH_SECTOR_SIZE;
        location[1] = (temp >> 16) & 0xFF;
        location[2] = (temp >> 8) & 0xFF;
        location[3] = temp & 0xFF;

        CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyFalse);
        status = CyU3PSpiTransmitWords (location, 4);
        CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyTrue);
    }

    uint32_t start_ticks = CyU3PGetTime();

    location[0] = 0x05; /* RDSTATUS */
    do {
		if ((CyU3PGetTime()- start_ticks) > FLASH_STATUS_TIMEOUT)
		{
			status = CY_U3P_ERROR_TIMEOUT;
			return status;
		}

    	CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyFalse);
        status = CyU3PSpiTransmitWords (location, 1);
        status = CyU3PSpiReceiveWords(rdBuf, 1);
        CyU3PGpioSetValue (FX3_SPI_FLASH_SS, CyTrue);
    } while(rdBuf[0] & 1);

    return status;
}
