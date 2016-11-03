/**
 * ----------------------------------------------------------------------------
 * FILE:	usb_config.h
 * DESCRIPTION:	USB configuration
 * DATE:	2016.10.27
 * AUTHOR(s):	Lime Microsystems
 * REVISION: v0r5
 * ----------------------------------------------------------------------------
 */

/* This file contains the constants and definitions */

#ifndef _INCLUDED_USB_CONFIG_H_
#define _INCLUDED_USB_CONFIG_H_

#include "cyu3externcstart.h"
#include "cyu3types.h"
#include "cyu3usbconst.h"

#define CY_FX_SLFIFO_THREAD_STACK       	(0x0400) /* Slave FIFO application thread stack size */
#define CY_FX_SLFIFO_THREAD_PRIORITY    	(8) /* Slave FIFO application thread priority */

//USB_BULK_STREAM config
#define USB_BULK_STREAM_BURST_LEN			16
#define USB_BULK_STREAM_DMA_BUF_SIZE		(4)
#define USB_BULK_STREAM_DMA_BUF_COUNT_P_2_U (8) /* USB_BULK_STREAM P_2_U channel buffer count */
#define USB_BULK_STREAM_DMA_BUF_COUNT_U_2_P	(4) /* USB_BULK_STREAM U_2_P channel buffer count */

//USB_BULK_CONTROL config
#define USB_BULK_CONTROL_DMA_BUF_COUNT     	(2) /* Slave FIFO channel buffer count */

#define DMA_TX_SIZE				        	(0) /* DMA transfer size is set to infinite */
#define DMA_RX_SIZE				        	(0) /* DMA transfer size is set to infinite */

/* Endpoint and socket definitions */

/* To change the Producer and Consumer EP enter the appropriate EP numbers for the #defines.
 * In the case of IN endpoints enter EP number along with the direction bit.
 * For eg. EP 6 IN endpoint is 0x86
 *     and EP 6 OUT endpoint is 0x06.
 * To change sockets mention the appropriate socket number in the #defines. */

/* Note: For USB 2.0 the endpoints and corresponding sockets are one-to-one mapped
         i.e. EP 1 is mapped to UIB socket 1 and EP 2 to socket 2 so on */

#define USB_BULK_STREAM_EP_PROD		0x01    /* EP 1 OUT  */
#define USB_BULK_STREAM_EP_CONS		0x81    /* EP 1 IN  */

#define USB_BULK_CONTROL_EP_PROD	0x0F    /* EP F OUT  */
#define USB_BULK_CONTROL_EP_CONS	0x8F    /* EP F IN  */

/* Producer socket used by the U-Port */
#define USB_BULK_STREAM_PROD_USB_SOCKET     CY_U3P_UIB_SOCKET_PROD_1     /* USB Socket 1 is producer */
#define USB_BULK_CONTROL_PROD_USB_SOCKET    CY_U3P_UIB_SOCKET_PROD_15    /* USB Socket 15 is producer */

/* Consumer socket used by the U-Port */
#define USB_BULK_STREAM_CONS_USB_SOCKET		CY_U3P_UIB_SOCKET_CONS_1    /* USB Socket 1 is consumer */
#define USB_BULK_CONTROL_CONS_USB_SOCKET	CY_U3P_UIB_SOCKET_CONS_15   /* USB Socket 15 is consumer */

// Producer socket used by the P-Port
#define USB_BULK_STREAM_PROD_PPORT_SOCKET	CY_U3P_PIB_SOCKET_2     // P-port Socket 2 is producer
#define USB_BULK_CONTROL_PROD_PPORT_SOCKET	CY_U3P_PIB_SOCKET_3     // P-port Socket 3 is producer

// Consumer socket used by the P-Port
#define USB_BULK_STREAM_CONS_PPORT_SOCKET	CY_U3P_PIB_SOCKET_0   // P-port Socket 0 is CONSUMER
#define USB_BULK_CONTROL_CONS_PPORT_SOCKET	CY_U3P_PIB_SOCKET_1   // P-port Socket 1 is CONSUMER

/* Extern definitions for the USB Descriptors */
extern const uint8_t CyFxUSB20DeviceDscr[];
extern const uint8_t CyFxUSB30DeviceDscr[];
extern const uint8_t CyFxUSBDeviceQualDscr[];
extern const uint8_t CyFxUSBFSConfigDscr[];
extern const uint8_t CyFxUSBHSConfigDscr[];
extern const uint8_t CyFxUSBBOSDscr[];
extern const uint8_t CyFxUSBSSConfigDscr[];
extern const uint8_t CyFxUSBStringLangIDDscr[];
extern const uint8_t CyFxUSBManufactureDscr[];
extern const uint8_t CyFxUSBProductDscr[];
extern uint8_t CyFxUSBSerialNumDesc[];

#include "cyu3externcend.h"

#endif /* _INCLUDED_USB_CONFIG_H_ */

/*[]*/
