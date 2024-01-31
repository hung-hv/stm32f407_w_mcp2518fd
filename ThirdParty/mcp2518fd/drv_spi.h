#ifndef _DRV_SPI_H
#define	_DRV_SPI_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "spi.h"//HAL��SPI��ʼ��
#include "gpio.h"//HAL��GPIO��ʼ��
#include "drv_canfdspi_api.h"
#include "drv_canfdspi_register.h"
#include "drv_canfdspi_defines.h"
//========================================================//
#ifdef	__cplusplus
extern "C" {
#endif

// Index to SPI channel
// Used when multiple MCP2517FD are connected to the same SPI interface, but with different CS    
#define DRV_CANFDSPI_INDEX_0         0
#define DRV_CANFDSPI_INDEX_1         1

// Transmit Channels
#define APP_TX_FIFO 		CAN_FIFO_CH2

// Receive Channels
#define APP_RX_FIFO 		CAN_FIFO_CH1

// Define by calculate how many times of timer ISR to reach out 1ms
#define TIMER_FACTOR 		1000

//struct canfd_frame frame;

extern const uint32_t TRANSMIT_TIMEOUT; //ms
extern uint8_t transmitTimeout_Flag;
extern uint32_t transmitTimeout_Cnt;

//! SPI Initialization
void CANFDSPI_Init(void);
//void DRV_SPI_Initialize(void);

//! SPI Read/Write Transfer

int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t XferSize);
//int8_t DRV_SPI_2_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize);

//! SPI Chip Select assert/de-assert

int8_t DRV_SPI_ChipSelectAssert(uint8_t spiSlaveDeviceIndex, bool assert);




void mcp2518fd_transpond(void);

void mcp2518fd_transmit(void);
void mcp2518fd_receive(void);
void CANFDSPI_Test(void);
//========================================================//
#ifdef	__cplusplus
}
#endif
//========================================================//
#endif	/* _DRV_SPI_H */

