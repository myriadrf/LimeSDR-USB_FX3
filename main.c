/**
 * ----------------------------------------------------------------------------
 * FILE:	main.c
 * DESCRIPTION:	LimeSDR-USB firmware main file
 * DATE:	2017.01.17
 * AUTHOR(s):	Lime Microsystems
 * REVISION: v0r4
 * ----------------------------------------------------------------------------
 */

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "usb_config.h"
#include "cyu3gpif.h"
#include "cyu3pib.h"
#include "cyu3gpio.h"
#include "cyu3i2c.h"
#include "cyu3spi.h"
#include "pib_regs.h"
#include <cyu3gpio.h>
#include "stdint.h"
#include <stdio.h>
#include "cyu3utils.h"

/* This file should be included only once as it contains
 * structure definitions. Including it in multiple places
 * can result in linker error. */
#include "LimeSDR-GPIF_32bit.cydsn/cyfxgpif2config.h" //32 bit GPIF configuration

#include "LMS64C_protocol.h"
#include "lime_sdr-usb_brd.h"
#include "spi_flash_lib.h"
#include "Si5351_config_map.h"

//GET_INFO FW_VER
#define FW_VER				4

#define sbi(p,n) ((p) |= (1UL << (n)))
#define cbi(p,n) ((p) &= ~(1 << (n)))

#define TRUE			CyTrue
#define FALSE			CyFalse

//CMD_PROG_MCU
#define PROG_EEPROM 1
#define PROG_SRAM	2
#define BOOT_MCU	3

#define MCU_CONTROL_REG	0x02
#define MCU_STATUS_REG	0x03
#define MCU_FIFO_WR_REG	0x04

#define MAX_MCU_RETRIES	30

//USB serial number from FX3 die id
static const char hex_digit[16] = "0123456789ABCDEF";
static uint32_t *EFUSE_DIE_ID = ((uint32_t *)0xE0055010);
uint32_t die_id[2];

uint8_t test, block, cmd_errors, glEp0Buffer[64], glEp0Buffer_Rx[64], glEp0Buffer_Tx[64] __attribute__ ((aligned (32))); //4096

tLMS_Ctrl_Packet *LMS_Ctrl_Packet_Tx = (tLMS_Ctrl_Packet*)glEp0Buffer_Tx;
tLMS_Ctrl_Packet *LMS_Ctrl_Packet_Rx = (tLMS_Ctrl_Packet*)glEp0Buffer_Rx;

int flash_page = 0, flash_page_data_cnt = 0, flash_data_cnt_free = 0, flash_data_counter_to_copy = 0;
unsigned char flash_page_data[FLASH_PAGE_SIZE], dac_val, need_fx3_reset = CyFalse;
uint8_t temp_status, MCU_retries;

//FPGA configuration
unsigned long int last_portion, current_portion, fpga_data, fpga_byte;
unsigned char data_cnt, sc_brdg_data[255];

CyU3PThread slFifoAppThread;	        /* Slave FIFO application thread structure */
CyU3PDmaChannel USB_BULK_STREAM_DMA_UtoP_Handle, USB_BULK_STREAM_DMA_PtoU_Handle; //USB_BULK_STREAM DMA Channel handles
CyU3PDmaChannel USB_BULK_CONTROL_DMA_UtoP_Handle, USB_BULK_CONTROL_DMA_PtoU_Handle;  //USB_BULK_CONTROL DMA Channel handles

CyBool_t glIsApplnActive = CyFalse;      /* Whether the loopback application is active or not. */

extern CyU3PReturnStatus_t CyU3PUsbSetTxSwing (uint32_t swing);

//functions prototypes
void Modify_BRDSPI16_Reg_bits (unsigned short int SPI_reg_addr, unsigned char MSB_bit, unsigned char LSB_bit, unsigned short int new_bits_data);
void Delay_us (unsigned int us);
unsigned char Check_many_blocks (unsigned char block_size);
void Configure_FPGA_from_flash (void);
void Configure_Si5351 (void);
void Configure_LM75 (void);
void Control_TCXO_DAC (unsigned char oe, unsigned char *data);
void Control_TCXO_ADF (unsigned char oe, unsigned char *data);
CyU3PReturnStatus_t Reconfigure_GPIF_16b (void);
void Wait_till_SC18B20_busy (void);
void Reconfigure_SPI_for_LMS (void);
void Reconfigure_SPI_for_Flash (void);
void GPIO_configuration (void);

/* Application Error Handler */
void CyFxAppErrorHandler (CyU3PReturnStatus_t apiRetStatus )   /* API return status */
{
	/* Application failed with the error code apiRetStatus */
	/* Add custom debug or recovery actions here */

	/* Loop Indefinitely */
	for (;;)
	{
		/* Thread sleep : 100 ms */
		CyU3PThreadSleep (100);
	}
}

/* I2c initialization . */
CyU3PReturnStatus_t CyFxI2cInit ()
{
	CyU3PI2cConfig_t i2cConfig;
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	/* Initialize and configure the I2C master module. */
	status = CyU3PI2cInit ();
	if (status != CY_U3P_SUCCESS)
	{
		return status;
	}

	// Start the I2C master block.
	CyU3PMemSet ((uint8_t *)&i2cConfig, 0, sizeof(i2cConfig));
	i2cConfig.bitRate    = 400 *1000;//bit rate 400 kHz
	i2cConfig.busTimeout = 0xFFFFFFFF;
	i2cConfig.dmaTimeout = 0xFFFF;
	i2cConfig.isDma      = CyFalse;

	status = CyU3PI2cSetConfig (&i2cConfig, NULL);
	return status;
}

/* This function starts the slave FIFO loop application. This is called
 * when a SET_CONF event is received from the USB host. The endpoints
 * are configured and the DMA pipe is setup in this function. */
void CyFxSlFifoApplnStart (void)
{
	uint16_t size = 0;
	CyU3PEpConfig_t epCfg;
	CyU3PDmaChannelConfig_t dmaCfg;
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

	// First identify the USB speed. Once that is identified, create a DMA channel and start the transfer on this.

	// Based on the Bus Speed configure the endpoint packet size
	switch (usbSpeed)
	{
		case CY_U3P_FULL_SPEED:
			size = 64;
			break;

		case CY_U3P_HIGH_SPEED:
			size = 512;
			break;

		case  CY_U3P_SUPER_SPEED:
			size = 1024;
			break;

		default:
			CyFxAppErrorHandler (CY_U3P_ERROR_FAILURE);
			break;
	}

	CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));

    //USB_BULK_STREAM endpoints (0x01, 0x81) configuration

    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.streams = 0;
    epCfg.pcktSize = size;
    if(usbSpeed == CY_U3P_SUPER_SPEED) epCfg.burstLen = USB_BULK_STREAM_BURST_LEN;
    	else  epCfg.burstLen = 1;

 	apiRetStatus = CyU3PSetEpConfig(USB_BULK_STREAM_EP_PROD, &epCfg); // Producer endpoint configuration
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler (apiRetStatus);
	}

	apiRetStatus = CyU3PSetEpConfig(USB_BULK_STREAM_EP_CONS, &epCfg); // Consumer endpoint configuration
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler (apiRetStatus);
	}

    //USB_BULK_CONTROL endpoints (0x0F, 0x8F) configuration

    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = 1;
    epCfg.streams = 0;
    epCfg.pcktSize = size;

    apiRetStatus = CyU3PSetEpConfig(USB_BULK_CONTROL_EP_PROD, &epCfg); // Producer endpoint configuration
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler (apiRetStatus);
	}

	apiRetStatus = CyU3PSetEpConfig(USB_BULK_CONTROL_EP_CONS, &epCfg); // Consumer endpoint configuration
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler (apiRetStatus);
	}

	//USB_BULK_STREAM DMA configuration

	dmaCfg.size  = USB_BULK_STREAM_DMA_BUF_SIZE*size; //DMA size is set based on the USB speed.
    dmaCfg.count = USB_BULK_CONTROL_DMA_BUF_COUNT;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;

    // Create a DMA AUTO channel for U2P transfer.

	dmaCfg.count = USB_BULK_STREAM_DMA_BUF_COUNT_U_2_P;

	dmaCfg.prodSckId = USB_BULK_STREAM_PROD_USB_SOCKET;
    dmaCfg.consSckId = USB_BULK_STREAM_CONS_PPORT_SOCKET;

    apiRetStatus = CyU3PDmaChannelCreate (&USB_BULK_STREAM_DMA_UtoP_Handle, CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    apiRetStatus = CyU3PDmaChannelSetXfer (&USB_BULK_STREAM_DMA_UtoP_Handle, DMA_TX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    // Create a DMA AUTO channel for P2U transfer.

	dmaCfg.count = USB_BULK_STREAM_DMA_BUF_COUNT_P_2_U;

    dmaCfg.prodSckId = USB_BULK_STREAM_PROD_PPORT_SOCKET;
    dmaCfg.consSckId = USB_BULK_STREAM_CONS_USB_SOCKET;
    apiRetStatus = CyU3PDmaChannelCreate (&USB_BULK_STREAM_DMA_PtoU_Handle, CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    apiRetStatus = CyU3PDmaChannelSetXfer (&USB_BULK_STREAM_DMA_PtoU_Handle, DMA_RX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    CyU3PUsbFlushEp(USB_BULK_STREAM_EP_PROD);
    CyU3PUsbFlushEp(USB_BULK_STREAM_EP_CONS);

    //USB_BULK_CONTROL DMA configuration

    dmaCfg.size  = size;
    dmaCfg.count = USB_BULK_CONTROL_DMA_BUF_COUNT;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;

    // Create a DMA AUTO channel for U2P transfer.

	dmaCfg.prodSckId = USB_BULK_CONTROL_PROD_USB_SOCKET;
    dmaCfg.consSckId = USB_BULK_CONTROL_CONS_PPORT_SOCKET;

    apiRetStatus = CyU3PDmaChannelCreate (&USB_BULK_CONTROL_DMA_UtoP_Handle, CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    apiRetStatus = CyU3PDmaChannelSetXfer (&USB_BULK_CONTROL_DMA_UtoP_Handle, DMA_TX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    // Create a DMA AUTO channel for P2U transfer.

    dmaCfg.prodSckId = USB_BULK_CONTROL_PROD_PPORT_SOCKET;
    dmaCfg.consSckId = USB_BULK_CONTROL_CONS_USB_SOCKET;
    apiRetStatus = CyU3PDmaChannelCreate (&USB_BULK_CONTROL_DMA_PtoU_Handle, CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    apiRetStatus = CyU3PDmaChannelSetXfer (&USB_BULK_CONTROL_DMA_PtoU_Handle, DMA_RX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    CyU3PUsbFlushEp(USB_BULK_CONTROL_EP_PROD);
    CyU3PUsbFlushEp(USB_BULK_CONTROL_EP_CONS);

	/* Update the status flag. */
	glIsApplnActive = CyTrue;
}

/* This function stops the slave FIFO loop application. This shall be called
 * whenever a RESET or DISCONNECT event is received from the USB host. The
 * endpoints are disabled and the DMA pipe is destroyed by this function. */
void CyFxSlFifoApplnStop (void)
{
	CyU3PEpConfig_t epCfg;
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	/* Update the flag. */
	glIsApplnActive = CyFalse;

	/* Flush the endpoint memory */
	CyU3PUsbFlushEp(USB_BULK_STREAM_EP_PROD);
	CyU3PUsbFlushEp(USB_BULK_STREAM_EP_CONS);
	CyU3PUsbFlushEp(USB_BULK_CONTROL_EP_PROD);
	CyU3PUsbFlushEp(USB_BULK_CONTROL_EP_CONS);

	CyU3PDmaChannelDestroy (&USB_BULK_STREAM_DMA_UtoP_Handle);
	CyU3PDmaChannelDestroy (&USB_BULK_STREAM_DMA_PtoU_Handle);
	CyU3PDmaChannelDestroy (&USB_BULK_CONTROL_DMA_UtoP_Handle);
	CyU3PDmaChannelDestroy (&USB_BULK_CONTROL_DMA_PtoU_Handle);

    /* Disable endpoints. */
    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyFalse;

	/* Producer endpoint configuration. */
	apiRetStatus = CyU3PSetEpConfig(USB_BULK_STREAM_EP_PROD, &epCfg);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* Consumer endpoint configuration. */
	apiRetStatus = CyU3PSetEpConfig(USB_BULK_STREAM_EP_CONS, &epCfg);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* Producer endpoint configuration. */
	apiRetStatus = CyU3PSetEpConfig(USB_BULK_CONTROL_EP_PROD, &epCfg);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* Consumer endpoint configuration. */
	apiRetStatus = CyU3PSetEpConfig(USB_BULK_CONTROL_EP_CONS, &epCfg);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler (apiRetStatus);
	}
}

/* Callback to handle the USB setup requests. */
CyBool_t CyFxSlFifoApplnUSBSetupCB (uint32_t setupdat0, uint32_t setupdat1)
{
	/* Fast enumeration is used. Only requests addressed to the interface, class,
	 * vendor and unknown control requests are received by this function.
	 * This application does not support any class or vendor requests. */

	uint8_t  bRequest, bReqType;
	uint8_t  bType, bTarget;
	uint16_t wValue;
	CyBool_t isHandled = CyFalse;

	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	/* Decode the fields from the setup request. */
	bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
	bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
	bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
	bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
	wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);

	if (bType == CY_U3P_USB_STANDARD_RQT)
	{
		/* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
		 * requests here. It should be allowed to pass if the device is in configured
		 * state and failed otherwise. */
		if ((bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE)
		        || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
		{
			if (glIsApplnActive)
				CyU3PUsbAckSetup ();
			else
				CyU3PUsbStall (0, CyTrue, CyFalse);

			isHandled = CyTrue;
		}
	}

	/* Handle supported vendor requests. */
	if (bType == CY_U3P_USB_VENDOR_RQT)
	{
		isHandled = CyTrue;
		uint8_t   I2C_Addr;
		CyU3PI2cPreamble_t preamble;

		switch (bRequest)
		{

			case 0xC0: //read
				CyU3PGpioSimpleSetValue (FX3_MCU_BUSY, CyTrue); //FX3 is busy
				CyU3PGpioSimpleSetValue (FX3_MCU_BUSY, CyFalse); //FX3 is not busy
				CyU3PUsbSendEP0Data (64, glEp0Buffer_Tx);
				if(need_fx3_reset) CyU3PDeviceReset(CyFalse); //hard fx3 reset
				break;

			case 0xC1: //write

				CyU3PGpioSimpleSetValue (FX3_MCU_BUSY, CyTrue); //indicate busy
				CyU3PUsbGetEP0Data (64, glEp0Buffer_Rx, NULL);

				// LMS64C protocol
				memset (glEp0Buffer_Tx, 0, sizeof(glEp0Buffer_Tx)); //fill whole tx buffer with zeros
				cmd_errors = 0;

				LMS_Ctrl_Packet_Tx->Header.Command = LMS_Ctrl_Packet_Rx->Header.Command;
				LMS_Ctrl_Packet_Tx->Header.Data_blocks = LMS_Ctrl_Packet_Rx->Header.Data_blocks;
				LMS_Ctrl_Packet_Tx->Header.Periph_ID = LMS_Ctrl_Packet_Rx->Header.Periph_ID;
				LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BUSY_CMD;

				//Executing commands. Some commands may be executed in FPGA (NIOS)
				switch(LMS_Ctrl_Packet_Rx->Header.Command)
				{
					case CMD_GET_INFO:

						//LSB_bytes
						LMS_Ctrl_Packet_Tx->Data_field[0] = FW_VER;
						LMS_Ctrl_Packet_Tx->Data_field[1] = DEV_TYPE;
						LMS_Ctrl_Packet_Tx->Data_field[2] = LMS_PROTOCOL_VER;
						LMS_Ctrl_Packet_Tx->Data_field[3] = HW_VER;
						LMS_Ctrl_Packet_Tx->Data_field[4] = EXP_BOARD;

						//BSN - board serial number
						LMS_Ctrl_Packet_Tx->Data_field[10] = (die_id[1] >> 24) & 0xFF;
						LMS_Ctrl_Packet_Tx->Data_field[11] = (die_id[1] >> 16) & 0xFF;
						LMS_Ctrl_Packet_Tx->Data_field[12] = (die_id[1] >>  8) & 0xFF;
						LMS_Ctrl_Packet_Tx->Data_field[13] = (die_id[1] >>  0) & 0xFF;

						LMS_Ctrl_Packet_Tx->Data_field[14] = (die_id[0] >> 24) & 0xFF;
						LMS_Ctrl_Packet_Tx->Data_field[15] = (die_id[0] >> 16) & 0xFF;
						LMS_Ctrl_Packet_Tx->Data_field[16] = (die_id[0] >>  8) & 0xFF;
						LMS_Ctrl_Packet_Tx->Data_field[17] = (die_id[0] >>  0) & 0xFF;

						LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
						break;

					case CMD_LMS_RST:

						switch (LMS_Ctrl_Packet_Rx->Data_field[0])
						{
							case LMS_RST_DEACTIVATE:
								Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 1); //high level
								break;

							case LMS_RST_ACTIVATE:
								Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 0); //low level
								break;

							case LMS_RST_PULSE:
								Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 0); //low level
								Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 1); //high level
								break;

							default:
								cmd_errors++;
								break;
						}

						LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
						break;

					case CMD_LMS7002_WR:
						if(Check_many_blocks (4)) break;
						Reconfigure_SPI_for_LMS ();

						I2C_Addr = I2C_ADDR_SC18IS602B;

						Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 0); //Enable LMS's SPI

						//write byte
						preamble.length    = 1;
						preamble.buffer[0] = I2C_Addr;
						preamble.buffer[1] = 0xF1; //Clear Interrupt
						preamble.ctrlMask  = 0x0000;

						sc_brdg_data[0] = 0xF1; //Clear Interrupt

						CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

						for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
						{
							//write reg addr
							sbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); //set write bit
						}

						//write byte
						preamble.length    = 2;
						preamble.buffer[0] = I2C_Addr;
						preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
						preamble.ctrlMask  = 0x0000;

						if( CyU3PI2cTransmitBytes (&preamble, &LMS_Ctrl_Packet_Rx->Data_field[0], 4*LMS_Ctrl_Packet_Rx->Header.Data_blocks, 0) != CY_U3P_SUCCESS)  cmd_errors++;

						Wait_till_SC18B20_busy ();

						Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 1); //Disable LMS's SPI

						if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
						else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
						break;

					case CMD_LMS7002_RD:
						if(Check_many_blocks (4)) break;
						Reconfigure_SPI_for_LMS ();

						I2C_Addr = I2C_ADDR_SC18IS602B;

						Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 0); //Enable LMS's SPI

						//write byte
						preamble.length    = 1;
						preamble.buffer[0] = I2C_Addr;
						preamble.buffer[1] = 0xF1; //Clear Interrupt
						preamble.ctrlMask  = 0x0000;
						sc_brdg_data[0] = 0xF1; //Clear Interrupt
						CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

						//write byte
						preamble.length    = 2;
						preamble.buffer[0] = I2C_Addr;
						preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
						preamble.ctrlMask  = 0x0000;

						for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
						{
							//write reg addr
							cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7);  //clear write bit

							sc_brdg_data[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)]; //reg addr MSB
							sc_brdg_data[1 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)]; //reg addr LSB

							sc_brdg_data[2 + (block * 4)] = 0x00; //dummy byte for spi reading
							sc_brdg_data[3 + (block * 4)] = 0x00; //dummy byte for spi reading
						}

						if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4*LMS_Ctrl_Packet_Rx->Header.Data_blocks, 0) != CY_U3P_SUCCESS)  cmd_errors++;

						Wait_till_SC18B20_busy ();

						//read bytes from I2C buffer
						I2C_Addr |= 1 << 0;	//read addr

						preamble.length    = 1;
						preamble.buffer[0] = I2C_Addr;
						preamble.ctrlMask  = 0x0000;


						if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 4*LMS_Ctrl_Packet_Rx->Header.Data_blocks, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

						for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
						{
							LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)]; //reg addr
							LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)]; //reg addr

							//read reg data
							LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = sc_brdg_data[(block * 4) + 2]; //reg data MSB
							LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = sc_brdg_data[(block * 4) + 3]; //reg data LSB
						}

						CyU3PThreadSleep (1); //need some time?
						//Wait_till_SC18B20_busy ();

						Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 1); //Disable LMS's SPI

						if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
						else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
						break;

					case CMD_BRDSPI16_WR:
						if(Check_many_blocks (4)) break;

						Reconfigure_SPI_for_LMS ();

						I2C_Addr = I2C_ADDR_SC18IS602B;

						//write byte
						preamble.length    = 1;
						preamble.buffer[0] = I2C_Addr;
						preamble.buffer[1] = 0xF1; //Clear Interrupt
						preamble.ctrlMask  = 0x0000;

						sc_brdg_data[0] = 0xF1; //Clear Interrupt

						CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

						for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
						{
							//write reg addr
							sbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); //set write bit
						}

						I2C_Addr = I2C_ADDR_SC18IS602B;

						//write byte
						preamble.length    = 2;
						preamble.buffer[0] = I2C_Addr;
						preamble.buffer[1] = BRDG_SPI_FPGA_SS; //FPGA SS
						preamble.ctrlMask  = 0x0000;

						if( CyU3PI2cTransmitBytes (&preamble, &LMS_Ctrl_Packet_Rx->Data_field[0], 4*LMS_Ctrl_Packet_Rx->Header.Data_blocks, 0) != CY_U3P_SUCCESS)  cmd_errors++;

						Wait_till_SC18B20_busy ();

						if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
						else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

						break;

					case CMD_BRDSPI16_RD:

						if(Check_many_blocks (4)) break;

						Reconfigure_SPI_for_LMS ();

						I2C_Addr = I2C_ADDR_SC18IS602B;

						//write byte
						preamble.length    = 1;
						preamble.buffer[0] = I2C_Addr;
						preamble.buffer[1] = 0xF1; //Clear Interrupt
						preamble.ctrlMask  = 0x0000;

						sc_brdg_data[0] = 0xF1; //Clear Interrupt

						CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);


						I2C_Addr = I2C_ADDR_SC18IS602B;

						//write byte
						preamble.length    = 2;
						preamble.buffer[0] = I2C_Addr;
						preamble.buffer[1] = BRDG_SPI_FPGA_SS; //FPGA SS
						preamble.ctrlMask  = 0x0000;

						for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
						{
							//write reg addr
							cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7);  //clear write bit

							sc_brdg_data[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)]; //reg addr MSB
							sc_brdg_data[1 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)]; //reg addr LSB

							sc_brdg_data[2 + (block * 4)] = 0x00; //dummy byte for spi reading
							sc_brdg_data[3 + (block * 4)] = 0x00; //dummy byte for spi reading
						}

						if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4*LMS_Ctrl_Packet_Rx->Header.Data_blocks, 0) != CY_U3P_SUCCESS)  cmd_errors++;

						Wait_till_SC18B20_busy ();

						//read byte

						I2C_Addr |= 1 << 0;	//read addr

						preamble.length    = 1;
						preamble.buffer[0] = I2C_Addr;
						preamble.ctrlMask  = 0x0000;

						if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 4*LMS_Ctrl_Packet_Rx->Header.Data_blocks, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

						for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
						{
							LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)]; //reg addr
							LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)]; //reg addr

							//read reg data
							LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = sc_brdg_data[(block * 4) + 2]; //reg data MSB
							LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = sc_brdg_data[(block * 4) + 3]; //reg data LSB
						}

						if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
						else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
						break;

					case CMD_ADF4002_WR:
						if(Check_many_blocks (3)) break;

						Control_TCXO_DAC (0, NULL); //set DAC out to three-state

						for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
						{
							Control_TCXO_ADF (1, &LMS_Ctrl_Packet_Rx->Data_field[0 + (block*3)]); //write data to ADF
						}

						if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_INVALID_PERIPH_ID_CMD;
						else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
						break;

					case CMD_SI5351_WR:
						if(Check_many_blocks (2)) break;

						I2C_Addr = I2C_ADDR_SI5351;

						for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
						{
							//write byte
							preamble.length    = 2;
							preamble.buffer[0] = I2C_Addr;
							preamble.buffer[1] = LMS_Ctrl_Packet_Rx->Data_field[block * 2]; //reg to write
							preamble.ctrlMask  = 0x0000;

							if( CyU3PI2cTransmitBytes (&preamble, &LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;
						}

						if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
						else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
						break;

					case CMD_SI5351_RD:
						if(Check_many_blocks (2)) break;

						I2C_Addr = I2C_ADDR_SI5351;

						for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
						{
							//read byte
							preamble.length = 3;

							I2C_Addr &= ~(1 << 0);//write addr
							preamble.buffer[0] = I2C_Addr;

							preamble.buffer[1] = LMS_Ctrl_Packet_Rx->Data_field[block]; //reg to read

							I2C_Addr |= 1 << 0;	//read addr

							preamble.buffer[2] = I2C_Addr;//0xE1; //read h70
							preamble.ctrlMask  = 0x0002;

							if( CyU3PI2cReceiveBytes (&preamble, &LMS_Ctrl_Packet_Tx->Data_field[1 + block * 2], 1, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

							LMS_Ctrl_Packet_Tx->Data_field[block * 2] = LMS_Ctrl_Packet_Rx->Data_field[block];
						}

						if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
						else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
						break;

					case CMD_SSTREAM_RST: //fifo_rst

						/*
						 * Data_field[0] value:
						 * 0 - USB_BULK_STREAM reset
						 * 1 - USB_BULK_CONTROL reset
						 */
						switch(LMS_Ctrl_Packet_Rx->Data_field[0])
						{
							case 0: //USB_BULK_STREAM reset

								CyU3PDmaChannelReset (&USB_BULK_STREAM_DMA_UtoP_Handle);
								CyU3PUsbFlushEp(USB_BULK_STREAM_EP_PROD);
								CyU3PUsbResetEp (USB_BULK_STREAM_EP_PROD);
								CyU3PDmaChannelSetXfer (&USB_BULK_STREAM_DMA_UtoP_Handle, DMA_TX_SIZE);

								CyU3PDmaChannelReset (&USB_BULK_STREAM_DMA_PtoU_Handle);
								CyU3PUsbFlushEp(USB_BULK_STREAM_EP_CONS);
								CyU3PUsbResetEp (USB_BULK_STREAM_EP_CONS);
								CyU3PDmaChannelSetXfer (&USB_BULK_STREAM_DMA_PtoU_Handle, DMA_RX_SIZE);

								LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
								break;

							case 1:

								CyU3PDmaChannelReset (&USB_BULK_CONTROL_DMA_UtoP_Handle);
								CyU3PUsbFlushEp(USB_BULK_CONTROL_EP_PROD);
								CyU3PUsbResetEp (USB_BULK_CONTROL_EP_PROD);
								CyU3PDmaChannelSetXfer (&USB_BULK_CONTROL_DMA_UtoP_Handle, DMA_TX_SIZE);

								CyU3PDmaChannelReset (&USB_BULK_CONTROL_DMA_PtoU_Handle);
								CyU3PUsbFlushEp(USB_BULK_CONTROL_EP_CONS);
								CyU3PUsbResetEp (USB_BULK_CONTROL_EP_CONS);
								CyU3PDmaChannelSetXfer (&USB_BULK_CONTROL_DMA_PtoU_Handle, DMA_RX_SIZE);

								LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
								break;

							default:
								LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
								break;
						}

						break;

					case CMD_MEMORY_WR:
						current_portion = (LMS_Ctrl_Packet_Rx->Data_field[1] << 24) | (LMS_Ctrl_Packet_Rx->Data_field[2] << 16) | (LMS_Ctrl_Packet_Rx->Data_field[3] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[4]);
						data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

						if((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 3)) //TARGET = 3 (EEPROM)
						{
							if(LMS_Ctrl_Packet_Rx->Data_field[0] == 0) //write data to EEPROM #1
							{
								I2C_Addr = I2C_ADDR_EEPROM;

								I2C_Addr &= ~(1 << 0);//write addr
								preamble.buffer[0] = I2C_Addr;

								preamble.buffer[1] = LMS_Ctrl_Packet_Rx->Data_field[8]; //addr hi
								preamble.buffer[2] = LMS_Ctrl_Packet_Rx->Data_field[9]; //addr lo

								preamble.length = 3;

								preamble.ctrlMask  = 0x0000;

								if( CyU3PI2cTransmitBytes (&preamble, &LMS_Ctrl_Packet_Rx->Data_field[24], data_cnt, 0) != CY_U3P_SUCCESS)  cmd_errors++;

								CyU3PThreadSleep (5); //wait till EEPROM finish writing

								if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
								else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
							}
							else
								LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
						}
						else

							if((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 1)) // TARGET = 1 (FX3)
							{
								switch (LMS_Ctrl_Packet_Rx->Data_field[0]) //PROG_MODE
								{
									case 0:
										need_fx3_reset = CyTrue;
										LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
										break;

									case 1: //PROG_MODE = 1 (destroy current firmware in flash)

										if(current_portion == 0)//beginning
										{
											Reconfigure_GPIF_16b ();
											Reconfigure_SPI_for_Flash ();

											FX3_FlashSpiEraseSector(CyTrue, 0);
											LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
										}
										else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
										break;

									case 2: //PROG_MODE = 2 (write FW to flash)

										if(current_portion == 0)//beginning
										{
											//check CY signature
											if (!(LMS_Ctrl_Packet_Rx->Data_field[24] == 'C' && LMS_Ctrl_Packet_Rx->Data_field[25] == 'Y')) //CY Signature ok?
											{
												LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
												break;
											}

											flash_page = 0;
											flash_page_data_cnt = 0;
											flash_data_counter_to_copy = 0;
											fpga_byte = 0;

											Reconfigure_GPIF_16b ();
											Reconfigure_SPI_for_Flash ();
										}

										flash_data_cnt_free = FX3_FLASH_PAGE_SIZE - flash_page_data_cnt;

										if (flash_data_cnt_free > 0)
										{
											if (flash_data_cnt_free > data_cnt)
												flash_data_counter_to_copy = data_cnt; //copy all data if fits to free page space
											else
												flash_data_counter_to_copy = flash_data_cnt_free; //copy only amount of data that fits in to free page size

											memcpy(&flash_page_data[flash_page_data_cnt], &LMS_Ctrl_Packet_Rx->Data_field[24], flash_data_counter_to_copy);

											flash_page_data_cnt = flash_page_data_cnt + flash_data_counter_to_copy;
											flash_data_cnt_free = FX3_FLASH_PAGE_SIZE - flash_page_data_cnt;

											if (data_cnt == 0)//all bytes transmitted, end of programming
											{
												if (flash_page_data_cnt > 0)
													flash_page_data_cnt = FX3_FLASH_PAGE_SIZE; //finish page
											}

											flash_data_cnt_free = FX3_FLASH_PAGE_SIZE - flash_page_data_cnt;
										}

										if (flash_page_data_cnt >= FX3_FLASH_PAGE_SIZE) //write data to flash
										{
											if ((flash_page % (FX3_FLASH_SECTOR_SIZE/FX3_FLASH_PAGE_SIZE)) == 0) //need to erase sector? reached number of pages in block?
												if( FX3_FlashSpiEraseSector(CyTrue, flash_page/(FX3_FLASH_SECTOR_SIZE/FX3_FLASH_PAGE_SIZE)) != CY_U3P_SUCCESS) cmd_errors++;

											if(!cmd_errors)
												if( FX3_FlashSpiTransfer(flash_page, FX3_FLASH_PAGE_SIZE, flash_page_data, CyFalse) != CY_U3P_SUCCESS)  cmd_errors++;//write to flash

											flash_page++;
											flash_page_data_cnt = 0;
											flash_data_cnt_free = FX3_FLASH_PAGE_SIZE - flash_page_data_cnt;
										}

										//if not all bytes written to flash page
										if (data_cnt > flash_data_counter_to_copy)
										{
											flash_data_counter_to_copy = data_cnt - flash_data_counter_to_copy;

											memcpy(&flash_page_data[flash_page_data_cnt], &LMS_Ctrl_Packet_Rx->Data_field[24], data_cnt);

											flash_page_data_cnt = flash_page_data_cnt + flash_data_counter_to_copy;
											flash_data_cnt_free = FX3_FLASH_PAGE_SIZE - flash_page_data_cnt;
										}

										fpga_byte = fpga_byte + data_cnt;

										if (fpga_byte <= FX3_SIZE) //correct firmware size?
										{
											if (data_cnt == 0)//end of programming
											{
											}

											if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
											else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
										}
										else //not correct firmware size
											LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;

										break;

									default:
										LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
										break;
								}
							}
							else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;

						break;

					case CMD_MEMORY_RD:
						current_portion = (LMS_Ctrl_Packet_Rx->Data_field[1] << 24) | (LMS_Ctrl_Packet_Rx->Data_field[2] << 16) | (LMS_Ctrl_Packet_Rx->Data_field[3] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[4]);
						data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

						if((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 3)) ///TARGET = 3 (EEPROM)
						{
							if(LMS_Ctrl_Packet_Rx->Data_field[0] == 0) //read data from EEPROM #1
							{
								I2C_Addr = I2C_ADDR_EEPROM;

								//read byte
								preamble.length = 4;

								I2C_Addr &= ~(1 << 0);//write addr
								preamble.buffer[0] = I2C_Addr;

								preamble.buffer[1] = LMS_Ctrl_Packet_Rx->Data_field[8];; //addr hi
								preamble.buffer[2] = LMS_Ctrl_Packet_Rx->Data_field[9]; //addr lo

								I2C_Addr |= 1 << 0;	//read addr

								preamble.buffer[3] = I2C_Addr;
								preamble.ctrlMask  = 0x0004;

								if( CyU3PI2cReceiveBytes (&preamble, &LMS_Ctrl_Packet_Tx->Data_field[24], data_cnt, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

								if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
								else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
							}
							else
								LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
						}
						else
							LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;

						break;

					case CMD_ALTERA_FPGA_GW_WR: //FPGA active serial

						current_portion = (LMS_Ctrl_Packet_Rx->Data_field[1] << 24) | (LMS_Ctrl_Packet_Rx->Data_field[2] << 16) | (LMS_Ctrl_Packet_Rx->Data_field[3] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[4]);
						data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

						switch(LMS_Ctrl_Packet_Rx->Data_field[0])//prog_mode
						{
							/*
							Programming mode:

							0 - Bitstream to FPGA
							1 - Bitstream to Flash
							2 - Bitstream from FLASH
							*/

							case 1: //write data to Flash from PC
								//Flash_ID();
								/*Reconfigure_SPI_for_Flash ();*/

								if(current_portion == 0)//beginning
								{
									flash_page = 0;
									flash_page_data_cnt = 0;
									flash_data_counter_to_copy = 0;
									fpga_byte = 0;

									Reconfigure_GPIF_16b ();
									Reconfigure_SPI_for_Flash ();

									//write byte
									preamble.length    = 1;
									preamble.buffer[0] = I2C_ADDR_MAX7322;
									preamble.ctrlMask  = 0x0000;
									sc_brdg_data[0] = 0x41; //AS_SW = 0, NCONFIG = 1 .. AS_SS = 1
									if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

									CyU3PThreadSleep (1);
								}

								flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;

								if (flash_data_cnt_free > 0)
								{
									if (flash_data_cnt_free > data_cnt)
										flash_data_counter_to_copy = data_cnt; //copy all data if fits to free page space
									else
										flash_data_counter_to_copy = flash_data_cnt_free; //copy only amount of data that fits in to free page size

									memcpy(&flash_page_data[flash_page_data_cnt], &LMS_Ctrl_Packet_Rx->Data_field[24], flash_data_counter_to_copy);

									flash_page_data_cnt = flash_page_data_cnt + flash_data_counter_to_copy;
									flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;

									if (data_cnt == 0)//all bytes transmitted, end of programming
									{
										if (flash_page_data_cnt > 0)
											flash_page_data_cnt = FLASH_PAGE_SIZE; //finish page
									}

									flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;
								}

								if (flash_page_data_cnt >= FLASH_PAGE_SIZE)
								{
									if ((flash_page % (FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE)) == 0) //need to erase sector? reached number of pages in block?
										if( FlashSpiEraseSector(CyTrue, flash_page/(FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE)) != CY_U3P_SUCCESS) cmd_errors++;

									if(!cmd_errors)
										if( FlashSpiTransfer(flash_page, FLASH_PAGE_SIZE, flash_page_data, CyFalse) != CY_U3P_SUCCESS)  cmd_errors++;//write to flash

									flash_page++;
									flash_page_data_cnt = 0;
									flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;
								}

								//if not all bytes written to flash page
								if (data_cnt > flash_data_counter_to_copy)
								{
									flash_data_counter_to_copy = data_cnt - flash_data_counter_to_copy;

									memcpy(&flash_page_data[flash_page_data_cnt], &LMS_Ctrl_Packet_Rx->Data_field[24], data_cnt);

									flash_page_data_cnt = flash_page_data_cnt + flash_data_counter_to_copy;
									flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;
								}

								fpga_byte = fpga_byte + data_cnt;

								if (fpga_byte <= FPGA_SIZE) //correct bitstream size?
								{
									if (data_cnt == 0)//end of programming
									{
										//write byte
										preamble.length    = 1;
										preamble.buffer[0] = I2C_ADDR_MAX7322;
										preamble.buffer[1] = 0xF7; //reg to write = GPIO Configuration
										preamble.ctrlMask  = 0x0000;
										sc_brdg_data[0] = 0xC1; //AS_SW = 1, NCONFIG = 1 .. AS_SS = 1
										if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;
									}

									if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
									else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
								}
								else //not correct bitsream size
									LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;

								break;

							case 2: //configure FPGA from FLASH

								Configure_FPGA_from_flash ();

								if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
								else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

								break;

							default:
								LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
								break;
						}

						break;

					case CMD_ANALOG_VAL_RD:

						for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
						{
							signed short int converted_val;

							switch (LMS_Ctrl_Packet_Rx->Data_field[0 + (block)])//ch
							{
								case 0://TCXO DAC val

									LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[block]; //ch
									LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = 0x00; //RAW //unit, power

									LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = 0; //signed val, MSB byte
									LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = dac_val; //signed val, LSB byte
									break;

								case 1: //temperature
									I2C_Addr = I2C_ADDR_LM75;

									//read byte
									preamble.length = 3;

									I2C_Addr &= ~(1 << 0);//write addr
									preamble.buffer[0] = I2C_Addr;

									preamble.buffer[1] = 0x00; //temperature

									I2C_Addr |= 1 << 0;	//read addr

									preamble.buffer[2] = I2C_Addr;
									preamble.ctrlMask  = 0x0002;

									if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 2, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

									converted_val = (((signed short int)sc_brdg_data[0]) << 8) + 0;//sc_brdg_data[1];
									converted_val = (converted_val/256)*10;

									if(sc_brdg_data[1]&0x80) converted_val = converted_val + 5;

									LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[block]; //ch
									LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = 0x50; //mC //unit, power

									LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = (converted_val >> 8); //signed val, MSB byte
									LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = converted_val; //signed val, LSB byte

									break;

								default:
									cmd_errors++;
									break;
							}
						}

						if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
						else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

						break;

					case CMD_ANALOG_VAL_WR:
						if(Check_many_blocks (4)) break;

						for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
						{
							switch (LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)]) //do something according to channel
							{
								case 0: //TCXO DAC
									if (LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 4)] == 0) //RAW units?
									{
										if(LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)] == 0) //MSB byte empty?
										{
											Control_TCXO_ADF (0, NULL); //set ADF4002 CP to three-state

											//write data to DAC
											dac_val = LMS_Ctrl_Packet_Rx->Data_field[3 + (block * 4)];
											Control_TCXO_DAC (1, &dac_val); //enable DAC output, set new val
										}
										else cmd_errors++;
									}
									else cmd_errors++;

									break;
								default:
									cmd_errors++;
									break;
							}
						}

						if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
						else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
						break;

					case CMD_LMS_MCU_FW_WR:

						current_portion = LMS_Ctrl_Packet_Rx->Data_field[1];

						//check if portions are send in correct order
						if(current_portion != 0) //not first portion?
						{
							if(last_portion != (current_portion - 1)) //portion number increments?
							{
								LMS_Ctrl_Packet_Tx->Header.Status = STATUS_WRONG_ORDER_CMD;
								break;
							}
						}

						I2C_Addr = I2C_ADDR_SC18IS602B;

						Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 0); //Enable LMS's SPI

						if (current_portion == 0) //PORTION_NR = first fifo
						{

							//write byte
							preamble.length    = 1;
							preamble.buffer[0] = I2C_Addr;
							preamble.buffer[1] = 0xF1; //Clear Interrupt
							preamble.ctrlMask  = 0x0000;
							sc_brdg_data[0] = 0xF1; //Clear Interrupt
							CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

							//write byte
							preamble.length    = 2;
							preamble.buffer[0] = I2C_Addr;
							preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
							preamble.ctrlMask  = 0x0000;

							//reset mcu
							sc_brdg_data[0] = (0x80); //reg addr MSB with write bit
							sc_brdg_data[1] = (MCU_CONTROL_REG); //reg addr LSB

							sc_brdg_data[2] = (0x00); //reg data MSB
							sc_brdg_data[3] = (0x00); //reg data LSB //8

							if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

							Wait_till_SC18B20_busy ();

							//write byte
							preamble.length    = 1;
							preamble.buffer[0] = I2C_Addr;
							preamble.buffer[1] = 0xF1; //Clear Interrupt
							preamble.ctrlMask  = 0x0000;
							sc_brdg_data[0] = 0xF1; //Clear Interrupt
							CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

							//write byte
							preamble.length    = 2;
							preamble.buffer[0] = I2C_Addr;
							preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
							preamble.ctrlMask  = 0x0000;

							//set mode
							//write reg addr - mSPI_REG2 (Controls MCU input pins)
							sc_brdg_data[0] = (0x80); //reg addr MSB with write bit
							sc_brdg_data[1] = (MCU_CONTROL_REG); //reg addr LSB

							sc_brdg_data[2] = (0x00); //reg data MSB

							//reg data LSB
							switch (LMS_Ctrl_Packet_Rx->Data_field[0]) //PROG_MODE
							{
								case PROG_EEPROM:
									sc_brdg_data[3] = (0x01); //Programming both EEPROM and SRAM  //8
									if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;
									Wait_till_SC18B20_busy ();
									break;

								case PROG_SRAM:
									sc_brdg_data[3] =(0x02); //Programming only SRAM  //8
									if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;
									Wait_till_SC18B20_busy ();
									break;

								case BOOT_MCU:
									sc_brdg_data[3] = (0x03); //Programming both EEPROM and SRAM  //8
									if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;
									Wait_till_SC18B20_busy ();

									//write byte
									preamble.length    = 1;
									preamble.buffer[0] = I2C_Addr;
									preamble.buffer[1] = 0xF1; //Clear Interrupt
									preamble.ctrlMask  = 0x0000;
									sc_brdg_data[0] = 0xF1; //Clear Interrupt
									CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);


									//write byte
									preamble.length    = 2;
									preamble.buffer[0] = I2C_Addr;
									preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
									preamble.ctrlMask  = 0x0000;

									sc_brdg_data[0] = (0x00); //reg addr MSB
									sc_brdg_data[1] = (MCU_STATUS_REG); //reg addr LSB
									sc_brdg_data[2] = 0x00;  //dummy byte for spi reading
									sc_brdg_data[3] = 0x00;  //dummy byte for spi reading

									if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

									Wait_till_SC18B20_busy ();


									//read bytes from I2C buffer
									I2C_Addr |= 1 << 0;	//read addr

									preamble.length    = 1;
									preamble.buffer[0] = I2C_Addr;
									preamble.ctrlMask  = 0x0000;

									if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 4, 0)  != CY_U3P_SUCCESS)  cmd_errors++;
									temp_status = sc_brdg_data[3]; //reg data LSB
									CyU3PThreadSleep (1); ///????

									goto BOOTING;

									break;
							}
						}

						MCU_retries = 0;

						//wait till EMPTY_WRITE_BUFF = 1
						while (MCU_retries < MAX_MCU_RETRIES)
						{
							//read status reg

							//spi read
							//write reg addr

							I2C_Addr = I2C_ADDR_SC18IS602B;

							//write byte
							preamble.length    = 1;
							preamble.buffer[0] = I2C_Addr;
							preamble.buffer[1] = 0xF1; //Clear Interrupt
							preamble.ctrlMask  = 0x0000;
							sc_brdg_data[0] = 0xF1; //Clear Interrupt
							CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

							//write byte
							preamble.length    = 2;
							preamble.buffer[0] = I2C_Addr;
							preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
							preamble.ctrlMask  = 0x0000;

							sc_brdg_data[0] = (0x00); //reg addr MSB
							sc_brdg_data[1] = (MCU_STATUS_REG); //reg addr LSB
							sc_brdg_data[2] = 0x00;  //dummy byte for spi reading
							sc_brdg_data[3] = 0x00;  //dummy byte for spi reading

							if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

							Wait_till_SC18B20_busy ();

							//read bytes from I2C buffer
							I2C_Addr |= 1 << 0;	//read addr

							preamble.length    = 1;
							preamble.buffer[0] = I2C_Addr;
							preamble.ctrlMask  = 0x0000;

							if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 4, 0)  != CY_U3P_SUCCESS)  cmd_errors++;
							temp_status = sc_brdg_data[3]; //reg data LSB
							CyU3PThreadSleep (1); ///wait


							if (temp_status &0x01) break; //EMPTY_WRITE_BUFF = 1

							MCU_retries++;
							Delay_us (30);
						}

						//write 32 bytes to FIFO
						for(block = 0; block < 32; block++)
						{
							I2C_Addr = I2C_ADDR_SC18IS602B;

							//write byte
							preamble.length    = 1;
							preamble.buffer[0] = I2C_Addr;
							preamble.buffer[1] = 0xF1; //Clear Interrupt
							preamble.ctrlMask  = 0x0000;
							sc_brdg_data[0] = 0xF1; //Clear Interrupt
							CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

							//write byte
							preamble.length    = 2;
							preamble.buffer[0] = I2C_Addr;
							preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
							preamble.ctrlMask  = 0x0000;

							//reset mcu
							sc_brdg_data[0] = (0x80); //reg addr MSB with write bit
							sc_brdg_data[1] = (MCU_FIFO_WR_REG); //reg addr LSB

							sc_brdg_data[2] = (0x00); //reg data MSB
							sc_brdg_data[3] = (LMS_Ctrl_Packet_Rx->Data_field[2 + block]); //reg data LSB //8

							if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

							Wait_till_SC18B20_busy ();

							temp_status=0;
							MCU_retries = 0;
						}

						MCU_retries = 0;

						//wait till EMPTY_WRITE_BUFF = 1
						while (MCU_retries < 500)
						{
							//read status reg

							//spi read
							//write reg addr

							I2C_Addr = I2C_ADDR_SC18IS602B;

							//write byte
							preamble.length    = 1;
							preamble.buffer[0] = I2C_Addr;
							preamble.buffer[1] = 0xF1; //Clear Interrupt
							preamble.ctrlMask  = 0x0000;
							sc_brdg_data[0] = 0xF1; //Clear Interrupt
							CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);


							//write byte
							preamble.length    = 2;
							preamble.buffer[0] = I2C_Addr;
							preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
							preamble.ctrlMask  = 0x0000;

							sc_brdg_data[0] = (0x00); //reg addr MSB
							sc_brdg_data[1] = (MCU_STATUS_REG); //reg addr LSB
							sc_brdg_data[2] = 0x00;  //dummy byte for spi reading
							sc_brdg_data[3] = 0x00;  //dummy byte for spi reading

							if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

							Wait_till_SC18B20_busy ();


							//read bytes from I2C buffer
							I2C_Addr |= 1 << 0;	//read addr

							preamble.length    = 1;
							preamble.buffer[0] = I2C_Addr;
							preamble.ctrlMask  = 0x0000;

							if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 4, 0)  != CY_U3P_SUCCESS)  cmd_errors++;
							temp_status = sc_brdg_data[3]; //reg data LSB
							CyU3PThreadSleep (1); //wait

							if (temp_status &0x01) break;  //EMPTY_WRITE_BUFF = 1

							MCU_retries++;
							Delay_us (30);
						}


						if (current_portion  == 255) //PORTION_NR = last FIFO
						{
							//check programmed bit

							MCU_retries = 0;

							//wait till PROGRAMMED = 1
							while (MCU_retries < MAX_MCU_RETRIES)
							{
								//read status reg

								//spi read
								//write reg addr

								I2C_Addr = I2C_ADDR_SC18IS602B;

								//write byte
								preamble.length    = 1;
								preamble.buffer[0] = I2C_Addr;
								preamble.buffer[1] = 0xF1; //Clear Interrupt
								preamble.ctrlMask  = 0x0000;
								sc_brdg_data[0] = 0xF1; //Clear Interrupt
								CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);


								//write byte
								preamble.length    = 2;
								preamble.buffer[0] = I2C_Addr;
								preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
								preamble.ctrlMask  = 0x0000;

								sc_brdg_data[0] = (0x00); //reg addr MSB
								sc_brdg_data[1] = (MCU_STATUS_REG); //reg addr LSB
								sc_brdg_data[2] = 0x00;  //dummy byte for spi reading
								sc_brdg_data[3] = 0x00;  //dummy byte for spi reading

								if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

								Wait_till_SC18B20_busy ();


								//read bytes from I2C buffer
								I2C_Addr |= 1 << 0;	//read addr

								preamble.length    = 1;
								preamble.buffer[0] = I2C_Addr;
								preamble.ctrlMask  = 0x0000;

								if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 4, 0)  != CY_U3P_SUCCESS)  cmd_errors++;
								temp_status = sc_brdg_data[3]; //reg data LSB
								CyU3PThreadSleep (1); //wait

								if (temp_status &0x40) break; //PROGRAMMED = 1

								MCU_retries++;
								Delay_us (30);
							}

							if (MCU_retries == MAX_MCU_RETRIES) cmd_errors++;
						}

						last_portion = current_portion; //save last portion number

BOOTING:

						if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
						else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

						Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 1); //Disable LMS's SPI

						break;


					default:

						LMS_Ctrl_Packet_Tx->Header.Status = STATUS_UNKNOWN_CMD;
						break;
				}

				CyU3PGpioSimpleSetValue (FX3_MCU_BUSY, CyFalse); //FX3 is not busy
				break;

			default:
				/* This is unknown request. */
				isHandled = CyFalse;
				break;
		}

		/* If there was any error, return not handled so that the library will
		 * stall the request. Alternatively EP0 can be stalled here and return
		 * CyTrue. */
		if (status != CY_U3P_SUCCESS)
		{
			isHandled = CyFalse;
		}
	}

	return isHandled;
}

/* This is the callback function to handle the USB events. */
void CyFxSlFifoApplnUSBEventCB (CyU3PUsbEventType_t evtype, uint16_t evdata)
{
	switch (evtype)
	{
		case CY_U3P_USB_EVENT_SETCONF:
			/* Stop the application before re-starting. */
			if (glIsApplnActive)
			{
				CyFxSlFifoApplnStop ();
			}
			CyU3PUsbLPMDisable();
			/* Start the loop back function. */
			CyFxSlFifoApplnStart ();
			break;

		case CY_U3P_USB_EVENT_RESET:
		case CY_U3P_USB_EVENT_DISCONNECT:
			/* Stop the loop back function. */
			if (glIsApplnActive)
			{
				CyFxSlFifoApplnStop ();
			}
			break;

		default:
			break;
	}
	CyU3PGpioSimpleSetValue (FX3_MCU_BUSY, CyTrue); //FX3 is busy
	CyU3PGpioSimpleSetValue (FX3_MCU_BUSY, CyFalse); //FX3 is not busy
}

/* Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
   whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function, the
   FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately tries
   to trigger an exit back to U0.

   This application does not have any state in which we should not allow U1/U2 transitions; and therefore
   the function always return CyTrue.
 */
CyBool_t CyFxApplnLPMRqtCB (CyU3PUsbLinkPowerMode link_mode)
{
	return CyTrue;
}

/* This function initializes the GPIF interface and initializes the USB interface. */
void CyFxSlFifoApplnInit (void)
{
	int i;

	CyU3PPibClock_t pibClock;
	CyU3PGpioClock_t gpioClock;
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	/* Initialize the p-port block. */
	pibClock.clkDiv = 4;
	pibClock.clkSrc = CY_U3P_SYS_CLK;
	pibClock.isHalfDiv = CyFalse;
	/* Disable DLL for sync GPIF */
	pibClock.isDllEnable = CyFalse;
	apiRetStatus = CyU3PPibInit(CyTrue, &pibClock);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* Load the GPIF configuration for Slave FIFO sync mode. */
	apiRetStatus = CyU3PGpifLoad (&CyFxGpifConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{

		CyFxAppErrorHandler(apiRetStatus);
	}

    //GPIF Watermark (FLAG B) configuration
    CyU3PGpifSocketConfigure (0, CY_U3P_PIB_SOCKET_0, 6, CyFalse, 1); //watermark = 6, The number of data words available for reading = watermark x (32/bus width) – 1
    CyU3PGpifSocketConfigure (1, CY_U3P_PIB_SOCKET_1, 6, CyFalse, 1); //watermark = 6, The number of data words available for reading = watermark x (32/bus width) – 1
    CyU3PGpifSocketConfigure (2, CY_U3P_PIB_SOCKET_2, 6, CyFalse, 1); //watermark = 6, The number of data words that may be written  = watermark x (32/bus width) – 4
    CyU3PGpifSocketConfigure (3, CY_U3P_PIB_SOCKET_3, 6, CyFalse, 1); //watermark = 6, The number of data words that may be written  = watermark x (32/bus width) – 4

	/* Start the state machine. */
	apiRetStatus = CyU3PGpifSMStart (RESET,ALPHA_RESET);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* Init the GPIO module */
	gpioClock.fastClkDiv = 2;
	gpioClock.slowClkDiv = 0;
	gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
	gpioClock.clkSrc = CY_U3P_SYS_CLK;
	gpioClock.halfDiv = 0;

	apiRetStatus = CyU3PGpioInit(&gpioClock, NULL);
	if (apiRetStatus != 0)
	{
		/* Error Handling */
		CyFxAppErrorHandler(apiRetStatus);
	}

	GPIO_configuration ();

	/* Start the USB functionality. */
	apiRetStatus = CyU3PUsbStart();
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	CyU3PUsbSetTxSwing(127); //TX Amplitude swing of the electrical signalling on the USB superspeed lines in 10 mV units. Should be less than 1.28V.

	/* The fast enumeration is the easiest way to setup a USB connection,
	 * where all enumeration phase is handled by the library. Only the
	 * class / vendor requests need to be handled by the application. */
	CyU3PUsbRegisterSetupCallback(CyFxSlFifoApplnUSBSetupCB, CyTrue);

	/* Setup the callback to handle the USB events. */
	CyU3PUsbRegisterEventCallback(CyFxSlFifoApplnUSBEventCB);

	/* Register a callback to handle LPM requests from the USB 3.0 host. */
	CyU3PUsbRegisterLPMRequestCallback(CyFxApplnLPMRqtCB);

	/* Set the USB Enumeration descriptors */

	/* Super speed device descriptor. */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* High speed device descriptor. */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* BOS descriptor */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* Device qualifier descriptor */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* Super speed configuration descriptor */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* High speed configuration descriptor */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* Full speed configuration descriptor */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* String descriptor 0 */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* String descriptor 1 */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* String descriptor 2 */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	//read FX3 die ID and set USB serial number
	CyU3PReadDeviceRegisters(EFUSE_DIE_ID, 2, die_id);
	for (i = 0; i < 2; i++)
	{
		CyFxUSBSerialNumDesc[i*16+ 2] = hex_digit[(die_id[1-i] >> 28) & 0xF];
		CyFxUSBSerialNumDesc[i*16+ 4] = hex_digit[(die_id[1-i] >> 24) & 0xF];
		CyFxUSBSerialNumDesc[i*16+ 6] = hex_digit[(die_id[1-i] >> 20) & 0xF];
		CyFxUSBSerialNumDesc[i*16+ 8] = hex_digit[(die_id[1-i] >> 16) & 0xF];
		CyFxUSBSerialNumDesc[i*16+10] = hex_digit[(die_id[1-i] >> 12) & 0xF];
		CyFxUSBSerialNumDesc[i*16+12] = hex_digit[(die_id[1-i] >>  8) & 0xF];
		CyFxUSBSerialNumDesc[i*16+14] = hex_digit[(die_id[1-i] >>  4) & 0xF];
		CyFxUSBSerialNumDesc[i*16+16] = hex_digit[(die_id[1-i] >>  0) & 0xF];
	}

	/* String descriptor 3 - USB serial number */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 3, (uint8_t *)CyFxUSBSerialNumDesc);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* Connect the USB Pins with super speed operation enabled. */
	apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}
}

/* Entry function for the slFifoAppThread. */
void SlFifoAppThread_Entry (uint32_t input)
{
	CyFxI2cInit ();
	//CyU3PSpiInit(); //impossible to use spi in 32 bit GPIF mode

	/* Initialize the slave FIFO application */
	CyFxSlFifoApplnInit();

	CyU3PGpioSimpleSetValue (FX3_MCU_BUSY, CyTrue); //FX3 is busy

	Configure_Si5351();

	Configure_FPGA_from_flash ();

	/* TCXO DAC and LM75 will be configured from NIOS

	//write default TCXO DAC value
	Control_TCXO_ADF (0, NULL); //set ADF4002 CP to three-state
	dac_val = 125; //default DAC value
	Control_TCXO_DAC (1, &dac_val); //enable DAC output, set new val

	Configure_LM75 (); //set LM75 configuration
	*/

	CyU3PGpioSimpleSetValue (FX3_MCU_BUSY, CyFalse); //FX3 is not busy

	for (;;)
	{
		CyU3PThreadSleep (100);
	}
}

/* Application define function which creates the threads. */
void CyFxApplicationDefine (void)
{
	void *ptr = NULL;
	uint32_t retThrdCreate = CY_U3P_SUCCESS;

	/* Allocate the memory for the thread */
	ptr = CyU3PMemAlloc (CY_FX_SLFIFO_THREAD_STACK);

	/* Create the thread for the application */
	retThrdCreate = CyU3PThreadCreate (&slFifoAppThread,           /* Slave FIFO app thread structure */
	                                   "21:Slave_FIFO_sync",                    /* Thread ID and thread name */
	                                   SlFifoAppThread_Entry,                   /* Slave FIFO app thread entry function */
	                                   0,                                       /* No input parameter to thread */
	                                   ptr,                                     /* Pointer to the allocated thread stack */
	                                   CY_FX_SLFIFO_THREAD_STACK,               /* App Thread stack size */
	                                   CY_FX_SLFIFO_THREAD_PRIORITY,            /* App Thread priority */
	                                   CY_FX_SLFIFO_THREAD_PRIORITY,            /* App Thread pre-emption threshold */
	                                   CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
	                                   CYU3P_AUTO_START                         /* Start the thread immediately */
	                                  );

	/* Check the return code */
	if (retThrdCreate != 0)
	{
		/* Thread Creation failed with the error code retThrdCreate */

		/* Add custom recovery or debug actions here */

		/* Application cannot continue */
		/* Loop indefinitely */
		while(1);
	}
}

/*
 * Main function
 */
int main (void)
{
	CyU3PIoMatrixConfig_t io_cfg;
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	CyU3PSysClockConfig_t clkCfg;

	/* setSysClk400 clock configurations */
	clkCfg.setSysClk400 = CyTrue;   /* FX3 device's master clock is set to a frequency > 400 MHz */
	clkCfg.cpuClkDiv = 2;           /* CPU clock divider */
	clkCfg.dmaClkDiv = 2;           /* DMA clock divider */
	clkCfg.mmioClkDiv = 2;          /* MMIO clock divider */
	clkCfg.useStandbyClk = CyFalse; /* device has no 32KHz clock supplied */
	clkCfg.clkSrc = CY_U3P_SYS_CLK; /* Clock source for a peripheral block  */

	/* Initialize the device */
	status = CyU3PDeviceInit (&clkCfg);
	if (status != CY_U3P_SUCCESS)
	{
		goto handle_fatal_error;
	}

	/* Initialize the caches. Enable instruction cache and keep data cache disabled.
	 * The data cache is useful only when there is a large amount of CPU based memory
	 * accesses. When used in simple cases, it can decrease performance due to large
	 * number of cache flushes and cleans and also it adds to the complexity of the
	 * code. */
	status = CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);
	if (status != CY_U3P_SUCCESS)
	{
		goto handle_fatal_error;
	}

	/* Configure the IO matrix for the device. On the FX3 DVK board, the COM port
	 * is connected to the IO(53:56). This means that either DQ32 mode should be
	 * selected or lppMode should be set to UART_ONLY. Here we are choosing
	 * UART_ONLY configuration for 16 bit slave FIFO configuration and setting
	 * isDQ32Bit for 32-bit slave FIFO configuration. */
	io_cfg.useUart   = CyFalse;
	io_cfg.useI2C    = CyTrue;
	io_cfg.useI2S    = CyFalse;
	io_cfg.useSpi    = CyFalse;
	io_cfg.isDQ32Bit = CyTrue;
	io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;

	io_cfg.gpioSimpleEn[0]  = 0;
	io_cfg.gpioSimpleEn[1]  = 0;
	io_cfg.gpioComplexEn[0] = 0;
	io_cfg.gpioComplexEn[1] = 0;

	status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
	if (status != CY_U3P_SUCCESS)
	{
		goto handle_fatal_error;
	}

	/* This is a non returnable call for initializing the RTOS kernel */
	CyU3PKernelEntry ();

	/* Dummy return to make the compiler happy */
	return 0;

handle_fatal_error:

	/* Cannot recover from this error. */
	while (1);
}

void Delay_us (unsigned int us)
{
	//CyU3PThreadSleep (1);
	unsigned int i;

	for (i = 0; i< (us*4); i++) {}
}

/**	This function checks if all blocks could fit in data field.
*	If blocks will not fit, function returns TRUE. */
unsigned char Check_many_blocks (unsigned char block_size)
{
	if (LMS_Ctrl_Packet_Rx->Header.Data_blocks > (sizeof(LMS_Ctrl_Packet_Tx->Data_field)/block_size))
	{
		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BLOCKS_ERROR_CMD;
		return TRUE;
	}
	else return FALSE;
	return FALSE;
}

/**
 *	@brief Function initializes FPGA configuring from FLASH memory by controlling NCONFIG pin
 */
void Configure_FPGA_from_flash (void)
{
	CyU3PI2cPreamble_t preamble;

	//configure FPGA from FLASH

	//write byte
	preamble.length    = 1;
	preamble.buffer[0] = I2C_ADDR_MAX7322;
	preamble.buffer[1] = 0xF7; //reg to write = GPIO Configuration
	preamble.ctrlMask  = 0x0000;
	sc_brdg_data[0] = 0x81; //AS_SW = 1, NCONFIG = 0 .. AS_SS = 1
	if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

	CyU3PThreadSleep (5);

	//write byte
	preamble.length    = 1;
	preamble.buffer[0] = I2C_ADDR_MAX7322;
	preamble.buffer[1] = 0xF7; //reg to write = GPIO Configuration
	preamble.ctrlMask  = 0x0000;
	sc_brdg_data[0] = 0xC1; //AS_SW = 1, NCONFIG = 1 .. AS_SS = 1
	if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;
}

/**
 *	@brief Function configures Si5351 clock synthesizer
 */
void Configure_Si5351 (void)
{
	uint8_t   I2C_Addr, Si_data[2], i, OEB = 0;
	CyU3PI2cPreamble_t preamble;

	I2C_Addr = I2C_ADDR_SI5351;

	//Disable Outputs
	preamble.length    = 2;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = 3; //reg to write
	preamble.ctrlMask  = 0x0000;

	Si_data[0] = 0xFF;
	if( CyU3PI2cTransmitBytes (&preamble, &Si_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

	//Power down all output drivers
	for(i=0; i<8; ++i)
    {
		preamble.length    = 2;
		preamble.buffer[0] = I2C_Addr;
		preamble.buffer[1] = 16 + i; //reg to write
		preamble.ctrlMask  = 0x0000;

		Si_data[0] = 0x84;
		if( CyU3PI2cTransmitBytes (&preamble, &Si_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;
    }

	//Write new configuration (registers 15 - 92)
	preamble.length    = 2;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = 15; //reg to write
	preamble.ctrlMask  = 0x0000;

	if( CyU3PI2cTransmitBytes (&preamble, &Si5351_config_map[0], 78, 0) != CY_U3P_SUCCESS)  cmd_errors++;

	//Write new configuration (registers 149 - 170)
	preamble.length    = 2;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = 149; //reg to write
	preamble.ctrlMask  = 0x0000;

	if( CyU3PI2cTransmitBytes (&preamble, &Si5351_config_map[78], 22, 0) != CY_U3P_SUCCESS)  cmd_errors++;

	//Apply PLLA and PLLB soft reset
	preamble.length    = 2;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = 177; //reg to write
	preamble.ctrlMask  = 0x0000;

	Si_data[0] = 0xAC;
	if( CyU3PI2cTransmitBytes (&preamble, &Si_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

	//scan CLKn control registers D7 to detect which clocks are powered down
	for(i=0; i<8; i++)
    {
		if (Si5351_config_map[i+1] & 0x80) sbi (OEB, i); //CLKn_PDN = 1 (powered down) then disable output in reg3 CLKn_OEB
    }

	//Enable desired outputs
	preamble.length    = 2;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = 3; //reg to write
	preamble.ctrlMask  = 0x0000;

	if( CyU3PI2cTransmitBytes (&preamble, &OEB, 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;
}

/**
 *	@brief Function to configure LM75 temperature sensor (OS polarity, THYST, TOS)
 */
void Configure_LM75 (void)
{
	uint8_t   I2C_Addr, LM75_data[2];
	CyU3PI2cPreamble_t preamble;

	//configure LM75

	I2C_Addr = I2C_ADDR_LM75;

	//write byte
	preamble.length = 2;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = 0x01; //Configuration addr
	preamble.ctrlMask  = 0x0000;

	LM75_data[0] = 0x04;//Configuration = OS polarity = 1, Comparator/int = 0, Shutdown = 0

	if( CyU3PI2cTransmitBytes (&preamble, &LM75_data[0], 1, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

	//write byte
	preamble.length = 2;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = 0x02; //THYST addr
	preamble.ctrlMask  = 0x0000;

	LM75_data[0] = 45;//THYST H
	LM75_data[1] = 0;//THYST L

	if( CyU3PI2cTransmitBytes (&preamble, &LM75_data[0], 2, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

	//write byte
	preamble.length = 2;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = 0x03; //TOS addr
	preamble.ctrlMask  = 0x0000;

	LM75_data[0] = 55;//TOS H
	LM75_data[1] = 0;//TOS L

	if( CyU3PI2cTransmitBytes (&preamble, &LM75_data[0], 2, 0)  != CY_U3P_SUCCESS)  cmd_errors++;
}

/**
 *	@brief Function to control DAC for TCXO frequency control
 *	@param oe output enable control: 0 - output disabled, 1 - output enabled
 *	@param data pointer to DAC value (1 byte)
 */
void Control_TCXO_DAC (unsigned char oe, unsigned char *data) //controls DAC (AD5601)
{
	CyU3PI2cPreamble_t preamble;
	uint8_t   I2C_Addr;
	unsigned char DAC_data[2];

	Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_SS_CTRL, TCXO_DAC_SS, TCXO_DAC_SS, 0); //select

	if (oe == 0) //set DAC out to three-state
	{
		DAC_data[0] = 0xC0; //POWER-DOWN MODE = THREE-STATE (MSB bits = 11) + MSB data
		DAC_data[1] = 0x00; //LSB data

		//Reconfigure_SPI_for_AD5601 ();

		//write-read SPI bytes using using I2C-SPI bridge
		I2C_Addr = I2C_ADDR_SC18IS602B;

		//write byte
		preamble.length    = 1;
		preamble.buffer[0] = I2C_Addr;
		preamble.buffer[1] = 0xF1; //Clear Interrupt
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0xF1; //Clear Interrupt
		CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

		preamble.length    = 2;
		preamble.buffer[0] = I2C_Addr;
		preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
		preamble.ctrlMask  = 0x0000;

		if( CyU3PI2cTransmitBytes (&preamble, &DAC_data[0], 2, 0) != CY_U3P_SUCCESS)  cmd_errors++;

		Wait_till_SC18B20_busy ();
	}
	else //enable DAC output, set new val
	{
		DAC_data[0] = (*data) >>2; //POWER-DOWN MODE = NORMAL OPERATION (MSB bits =00) + MSB data
		DAC_data[1] = (*data) <<6; //LSB data

		//Reconfigure_SPI_for_AD5601 ();

		//write-read SPI bytes using using I2C-SPI bridge
		I2C_Addr = I2C_ADDR_SC18IS602B;

		//write byte
		preamble.length    = 1;
		preamble.buffer[0] = I2C_Addr;
		preamble.buffer[1] = 0xF1; //Clear Interrupt
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0xF1; //Clear Interrupt
		CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

		preamble.length    = 2;
		preamble.buffer[0] = I2C_Addr;
		preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
		preamble.ctrlMask  = 0x0000;

		if( CyU3PI2cTransmitBytes (&preamble, &DAC_data[0], 2, 0) != CY_U3P_SUCCESS)  cmd_errors++;

		Wait_till_SC18B20_busy ();
	}

	Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_SS_CTRL, TCXO_DAC_SS, TCXO_DAC_SS, 1); //deselect
}

/**
 *	@brief Function to control ADF for TCXO frequency control
 *	@param oe output enable control: 0 - output disabled, 1 - output enabled
 *	@param data pointer to ADF data block (3 bytes)
 */
void Control_TCXO_ADF (unsigned char oe, unsigned char *data) //controls ADF4002
{
	CyU3PI2cPreamble_t preamble;
	uint8_t   I2C_Addr;
	unsigned char ADF_data[12], ADF_block;

	if (oe == 0) //set ADF4002 CP to three-state and MUX_OUT to DGND
	{
		ADF_data[0] = 0x1f;
		ADF_data[1] = 0x81;
		ADF_data[2] = 0xf3;
		ADF_data[3] = 0x1f;
		ADF_data[4] = 0x81;
		ADF_data[5] = 0xf2;
		ADF_data[6] = 0x00;
		ADF_data[7] = 0x01;
		ADF_data[8] = 0xf4;
		ADF_data[9] = 0x01;
		ADF_data[10] = 0x80;
		ADF_data[11] = 0x01;

		Reconfigure_SPI_for_LMS();

		//write data to ADF
		for(ADF_block = 0; ADF_block < 4; ADF_block++)
		{
			Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_SS_CTRL, TCXO_ADF_SS, TCXO_ADF_SS, 0); //Select ADF

			I2C_Addr = I2C_ADDR_SC18IS602B;

			//write byte
			preamble.length    = 1;
			preamble.buffer[0] = I2C_Addr;
			preamble.buffer[1] = 0xF1; //Clear Interrupt
			preamble.ctrlMask  = 0x0000;

			sc_brdg_data[0] = 0xF1; //Clear Interrupt

			CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

			I2C_Addr = I2C_ADDR_SC18IS602B;

			//write byte
			preamble.length    = 2;
			preamble.buffer[0] = I2C_Addr;
			preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
			preamble.ctrlMask  = 0x0000;

			if( CyU3PI2cTransmitBytes (&preamble, &ADF_data[ADF_block*3], 3, 0) != CY_U3P_SUCCESS)  cmd_errors++;

			Wait_till_SC18B20_busy ();

			Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_SS_CTRL, TCXO_ADF_SS, TCXO_ADF_SS, 1); //Deselect ADF
		}
	}
	else //set PLL parameters, 4 blocks must be written
	{
		Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_SS_CTRL, TCXO_ADF_SS, TCXO_ADF_SS, 0); //Select ADF

		Reconfigure_SPI_for_LMS();

		I2C_Addr = I2C_ADDR_SC18IS602B;

		//write byte
		preamble.length    = 1;
		preamble.buffer[0] = I2C_Addr;
		preamble.buffer[1] = 0xF1; //Clear Interrupt
		preamble.ctrlMask  = 0x0000;

		sc_brdg_data[0] = 0xF1; //Clear Interrupt

		CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

		I2C_Addr = I2C_ADDR_SC18IS602B;

		//write byte
		preamble.length    = 2;
		preamble.buffer[0] = I2C_Addr;
		preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
		preamble.ctrlMask  = 0x0000;

		if( CyU3PI2cTransmitBytes (&preamble, data, 3, 0) != CY_U3P_SUCCESS)  cmd_errors++;

		Wait_till_SC18B20_busy ();

		Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_SS_CTRL, TCXO_ADF_SS, TCXO_ADF_SS, 1); //Deselect ADF
	}
}

CyU3PReturnStatus_t Reconfigure_GPIF_16b (void)
{
	CyU3PIoMatrixConfig_t io_cfg;
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	CyU3PGpifDisable(CyTrue);
	//CyU3PDmaChannelReset(&glDmaChHandle);
	CyU3PPibDeInit();
	CyU3PLppDeInit(2); // Deinit SPI block

	io_cfg.useUart   = CyFalse;
	io_cfg.useI2C    = CyTrue;
	io_cfg.useI2S    = CyFalse;
	io_cfg.useSpi    = CyTrue;
	io_cfg.isDQ32Bit = CyFalse;
	io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_SPI_ONLY;

	/* No GPIOs are enabled. */
	io_cfg.gpioSimpleEn[0]  = 0;
	io_cfg.gpioSimpleEn[1]  = 0;
	io_cfg.gpioComplexEn[0] = 0;
	io_cfg.gpioComplexEn[1] = 0;

	status = CyU3PDeviceConfigureIOMatrix (&io_cfg);

	if (status != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(status);
	}

	GPIO_configuration ();

	/* Start the SPI module and configure the master. */
	status = CyU3PSpiInit();
	if (status != CY_U3P_SUCCESS)
	{
		return status;
	}

	return status;
}

void Modify_BRDSPI16_Reg_bits (unsigned short int SPI_reg_addr, unsigned char MSB_bit, unsigned char LSB_bit, unsigned short int new_bits_data)
{
	unsigned short int mask, SPI_reg_data;
	unsigned char bits_number;
	uint8_t MSB_byte, LSB_byte;

	uint8_t   I2C_Addr;
	CyU3PI2cPreamble_t preamble;

	//Reconfigure_SPI_for_LMS ();

	bits_number = MSB_bit - LSB_bit + 1;

	mask = 0xFFFF;

	//removing unnecessary bits from mask
	mask = mask << (16 - bits_number);
	mask = mask >> (16 - bits_number);

	new_bits_data &= mask; //mask new data

	new_bits_data = new_bits_data << LSB_bit; //shift new data

	mask = mask << LSB_bit; //shift mask
	mask =~ mask;//invert mask

	MSB_byte = (SPI_reg_addr >> 8 ) & 0xFF;
	LSB_byte = SPI_reg_addr & 0xFF;


	I2C_Addr = I2C_ADDR_SC18IS602B;

	//write byte
	preamble.length    = 1;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = 0xF1; //Clear Interrupt
	preamble.ctrlMask  = 0x0000;

	sc_brdg_data[0] = 0xF1; //Clear Interrupt

	CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);


	I2C_Addr = I2C_ADDR_SC18IS602B;

	//write byte
	preamble.length    = 2;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = BRDG_SPI_FPGA_SS; //FPGA SS
	preamble.ctrlMask  = 0x0000;

	sc_brdg_data[0] = MSB_byte;
	sc_brdg_data[1] = LSB_byte;
	sc_brdg_data[2] = 0x00; //dummy byte for spi reading
	sc_brdg_data[3] = 0x00; //dummy byte for spi reading

	if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

	//CyU3PThreadSleep (2);
	Wait_till_SC18B20_busy ();


	//read byte
	I2C_Addr |= 1 << 0;	//read addr

	preamble.length    = 1;
	preamble.buffer[0] = I2C_Addr;
	preamble.ctrlMask  = 0x0000;

	if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 4, 0)  != CY_U3P_SUCCESS)  cmd_errors++;;

	//read reg data
	MSB_byte = sc_brdg_data[2]; //reg data MSB
	LSB_byte = sc_brdg_data[3]; //reg data LSB


	SPI_reg_data = (MSB_byte << 8) + LSB_byte; //read current SPI reg data

	//modify reg data
	SPI_reg_data &= mask;//clear bits
	SPI_reg_data |= new_bits_data; //set bits with new data

	//write reg addr
	MSB_byte = (SPI_reg_addr >> 8 ) & 0xFF;
	LSB_byte = SPI_reg_addr & 0xFF;

	sbi(MSB_byte, 7); //set write bit


	I2C_Addr = I2C_ADDR_SC18IS602B;// I2C_ADDR_SC18IS602B

	//write byte
	preamble.length    = 1;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = 0xF1; //Clear Interrupt
	preamble.ctrlMask  = 0x0000;

	sc_brdg_data[0] = 0xF1; //Clear Interrupt

	CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);


	I2C_Addr = I2C_ADDR_SC18IS602B;

	//write byte
	preamble.length    = 2;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = BRDG_SPI_FPGA_SS; //FPGA SS
	preamble.ctrlMask  = 0x0000;

	sc_brdg_data[0] = MSB_byte;
	sc_brdg_data[1] = LSB_byte;

	////write modified data back to SPI reg
	MSB_byte = (SPI_reg_data >> 8 ) & 0xFF;
	LSB_byte = SPI_reg_data & 0xFF;

	sc_brdg_data[2] = MSB_byte;
	sc_brdg_data[3] = LSB_byte;

	if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

	//CyU3PThreadSleep (2);
	Wait_till_SC18B20_busy ();
}

void Wait_till_SC18B20_busy (void)
{
	CyBool_t int_val;
	int a=0;

	CyU3PGpioSimpleGetValue (BRDG_INT, &int_val);

	a++;

	while (int_val == CyTrue) //wait till SPI transmission has been completed
	{
		Delay_us(10); //wait 10 us
		a++;

		if(a > 300) break; //wait up to 3 ms

		CyU3PGpioSimpleGetValue (BRDG_INT, &int_val);
	}
}

/** Reconfigures SPI to match the current serial port settings issued by the host. */
void Reconfigure_SPI_for_LMS(void)
{
	uint8_t   I2C_Addr, data;
	CyU3PI2cPreamble_t preamble;

	I2C_Addr = I2C_ADDR_SC18IS602B;

	//write byte
	preamble.length    = 2;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = 0xF0; //reg to write = configure spi
	preamble.buffer[2] = 0x00;
	preamble.ctrlMask  = 0x0000;
	data = 0x00; // MSB word first, Mode 0 (CPOL = 0, CPHA = 0), 1.843 MHz
	if( CyU3PI2cTransmitBytes (&preamble, &data, 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;
	//CyU3PThreadSleep (1);
}

void Reconfigure_SPI_for_Flash(void)
{
	CyU3PSpiConfig_t spiConfig;

	CyU3PMemSet ((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
	spiConfig.isLsbFirst = CyFalse;
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

void GPIO_configuration (void)
{
	CyU3PGpioSimpleConfig_t gpioConfig;

	gpioConfig.outValue = CyFalse;
	gpioConfig.inputEn = CyFalse;
	gpioConfig.driveLowEn = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
	CyU3PDeviceGpioOverride (FX3_MCU_BUSY, CyTrue);
	CyU3PGpioSetSimpleConfig(FX3_MCU_BUSY, &gpioConfig);

	gpioConfig.outValue = CyTrue;
	gpioConfig.inputEn = CyFalse;
	gpioConfig.driveLowEn = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
	CyU3PDeviceGpioOverride (FX3_SPI_FLASH_SS, CyTrue);
	CyU3PGpioSetSimpleConfig(FX3_SPI_FLASH_SS, &gpioConfig);

	gpioConfig.outValue = CyTrue;
	gpioConfig.inputEn = CyFalse;
	gpioConfig.driveLowEn = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
	CyU3PDeviceGpioOverride (FX3_SPI_AS_SS, CyTrue);
	CyU3PGpioSetSimpleConfig(FX3_SPI_AS_SS, &gpioConfig);

	gpioConfig.outValue = CyFalse;
	gpioConfig.inputEn = CyTrue;
	gpioConfig.driveLowEn = CyFalse;
	gpioConfig.driveHighEn = CyFalse;
	gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
	CyU3PDeviceGpioOverride (BRDG_INT, CyTrue);
	CyU3PGpioSetSimpleConfig(BRDG_INT, &gpioConfig);
}
