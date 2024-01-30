#include "drv_spi.h"

#define BIT(_x_) ((uint32_t)(1U) << (_x_))

#define MTT_MAX_DATA_LEN 64
#define CAN_FMT 0x80000000U	/*1= EXT/ 0 = STD */
#define CAN_RTR 0x40000000U	/* RTR */
#define CAN_ERR 0x20000000U	/* ERR/Data  message frame */

struct canfd_frame {
	uint32_t can_id;	/* FMT/RTR/ERR/ID */
	uint8_t d_len;		/* data length */
	uint8_t flags;		/* FD flags */
	uint8_t resv0;
	uint8_t resv1;
	uint8_t data[MTT_MAX_DATA_LEN];
	/* Any new structure entries should be placed below the comment */
	uint32_t tstamp;
};

CAN_CONFIG config;

// Transmit objects
CAN_TX_FIFO_CONFIG txConfig;
CAN_TX_FIFO_EVENT txFlags;
CAN_TX_MSGOBJ txObj;
//uint8_t txd[MAX_DATA_BYTES];

// Receive objects
CAN_RX_FIFO_CONFIG rxConfig;
REG_CiFLTOBJ fObj;
REG_CiMASK mObj;
CAN_RX_FIFO_EVENT rxFlags;
CAN_RX_MSGOBJ rxObj;
uint8_t rxd[MAX_DATA_BYTES];

uint8_t tec;
uint8_t rec;
CAN_ERROR_STATE errorFlags;

extern uint32_t ext5_cnt;

bool ramInitialized = false;

void CANFDSPI_Init( void )
{

//	// Reset device
//	DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0);
//
//	// Enable ECC and initialize RAM
//	DRV_CANFDSPI_EccEnable(DRV_CANFDSPI_INDEX_0);
//
//	DRV_CANFDSPI_RamInit(DRV_CANFDSPI_INDEX_0, 0xff);
//
//	// Configure device
//	DRV_CANFDSPI_ConfigureObjectReset(&config);
//	config.IsoCrcEnable = 1;
//	config.StoreInTEF = 0;
//	config.TXQEnable = 0;
//
//	DRV_CANFDSPI_Configure(DRV_CANFDSPI_INDEX_0, &config);
//
//	// Setup TX FIFO
//	DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
//	txConfig.FifoSize = 7;
//	txConfig.PayLoadSize = CAN_PLSIZE_64;
//	txConfig.TxPriority = 1;
//
//	DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txConfig);
//
//	// Setup RX FIFO
//	DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
//	rxConfig.FifoSize = 15;
//	rxConfig.PayLoadSize = CAN_PLSIZE_64;
//
//	DRV_CANFDSPI_ReceiveChannelConfigure(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &rxConfig);
//
//	// Setup RX Filter
//	fObj.word = 0;
//	fObj.bF.SID = 0x300;
//	fObj.bF.EXIDE = 0;
//	fObj.bF.EID = 0x00;
//
//	DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &fObj.bF);
//
//	// Setup RX Mask
//	mObj.word = 0;
//	mObj.bF.MSID = 0x7F8;
//	mObj.bF.MIDE = 1; // Only allow standard IDs
//	mObj.bF.MEID = 0x0;
//	DRV_CANFDSPI_FilterMaskConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &mObj.bF);
//
//	// Link FIFO and Filter
//	DRV_CANFDSPI_FilterToFifoLink(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, APP_RX_FIFO, true);
//
//	// Setup Bit Time
//	DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_500K_2M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
//
//	// Setup Transmit and Receive Interrupts
//	DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);
//	DRV_CANFDSPI_TransmitChannelEventEnable(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, CAN_TX_FIFO_NOT_FULL_EVENT);
//	DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, CAN_RX_FIFO_NOT_EMPTY_EVENT);
//	DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_0, CAN_TX_EVENT | CAN_RX_EVENT);
//
//	// Select Normal Mode
//	//DRV_CANFDSPI_OperationModeSelect(CAN_NORMAL_MODE);
//
//	DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_EXTERNAL_LOOPBACK_MODE);
//
//    /**************************************************************************/
    // MCP2518FD 1#
    DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0);

    // Enable ECC and initialize RAM
    DRV_CANFDSPI_EccEnable(DRV_CANFDSPI_INDEX_0);

    DRV_CANFDSPI_RamInit(DRV_CANFDSPI_INDEX_0, 0xff);

    // Configure device
    DRV_CANFDSPI_ConfigureObjectReset(&config);
    config.IsoCrcEnable = 1;
    config.StoreInTEF = 0;
    config.TXQEnable = 0;

    DRV_CANFDSPI_Configure(DRV_CANFDSPI_INDEX_0, &config);

    // Setup TX FIFO
    DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
    txConfig.FifoSize = 7;
    txConfig.PayLoadSize = CAN_PLSIZE_64;
    txConfig.TxPriority = 1;

    DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txConfig);

    // Setup Bit Time
    DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_500K_5M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
//    DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_125K_500K, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
	// Setup Transmit and Receive Interrupts
//	DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);
//	DRV_CANFDSPI_TransmitChannelEventEnable(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, CAN_TX_FIFO_NOT_FULL_EVENT);
//	DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, CAN_RX_FIFO_NOT_EMPTY_EVENT);
//	DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_0, CAN_TX_EVENT | CAN_RX_EVENT);

    // Select Normal Mode
    DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_NORMAL_MODE);
//
////    STBY_0_Clear();
//    /**************************************************************************/
    // MCP2518FD 2#
    DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_1);

    // Enable ECC and initialize RAM
    DRV_CANFDSPI_EccEnable(DRV_CANFDSPI_INDEX_1);

    DRV_CANFDSPI_RamInit(DRV_CANFDSPI_INDEX_1, 0xff);

    // Configure device
    DRV_CANFDSPI_ConfigureObjectReset(&config);
    config.IsoCrcEnable = 1;
    config.StoreInTEF = 0;
    config.TXQEnable = 0;

    DRV_CANFDSPI_Configure(DRV_CANFDSPI_INDEX_1, &config);

    // Setup RX FIFO
    DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
    rxConfig.FifoSize = 15;
    rxConfig.PayLoadSize = CAN_PLSIZE_64;

    DRV_CANFDSPI_ReceiveChannelConfigure(DRV_CANFDSPI_INDEX_1, APP_RX_FIFO, &rxConfig);

    // Setup RX Filter
    fObj.word = 0;
    fObj.bF.SID = 0x300;
    fObj.bF.EXIDE = 0;
    fObj.bF.EID = 0x00;

    DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_1, CAN_FILTER0, &fObj.bF);

    // Setup RX Mask
    mObj.word = 0;
    mObj.bF.MSID = 0x7F8;
    mObj.bF.MIDE = 1; // Only allow standard IDs
    mObj.bF.MEID = 0x0;
    DRV_CANFDSPI_FilterMaskConfigure(DRV_CANFDSPI_INDEX_1, CAN_FILTER0, &mObj.bF);

    // Link FIFO and Filter
    DRV_CANFDSPI_FilterToFifoLink(DRV_CANFDSPI_INDEX_1, CAN_FILTER0, APP_RX_FIFO, true);

    // Setup Bit Time
    DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_1, CAN_500K_5M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);

    // Setup Transmit and Receive Interrupts
	DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_1, GPIO_MODE_INT, GPIO_MODE_INT);
	DRV_CANFDSPI_TransmitChannelEventEnable(DRV_CANFDSPI_INDEX_1, APP_TX_FIFO, CAN_TX_FIFO_NOT_FULL_EVENT);
	DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_1, APP_RX_FIFO, CAN_RX_FIFO_NOT_EMPTY_EVENT);
	DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_1, CAN_RX_EVENT);
    // Select Normal Mode
    DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_1, CAN_NORMAL_MODE);

//    STBY_1_Clear();


    /*************************************************************************************************/
//    	// Reset device
//    	DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_1);
//
//    	// Enable ECC and initialize RAM
//    	DRV_CANFDSPI_EccEnable(DRV_CANFDSPI_INDEX_1);
//
//    	DRV_CANFDSPI_RamInit(DRV_CANFDSPI_INDEX_1, 0xff);
//
//    	// Configure device
//    	DRV_CANFDSPI_ConfigureObjectReset(&config);
//    	config.IsoCrcEnable = 1;
//    	config.StoreInTEF = 0;
//    	config.TXQEnable = 0;
//
//    	DRV_CANFDSPI_Configure(DRV_CANFDSPI_INDEX_1, &config);
//
//    	// Setup TX FIFO
//    	DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
//    	txConfig.FifoSize = 7;
//    	txConfig.PayLoadSize = CAN_PLSIZE_64;
//    	txConfig.TxPriority = 1;
//
//    	DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_1, APP_TX_FIFO, &txConfig);
//
//    	// Setup RX FIFO
//    	DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
//    	rxConfig.FifoSize = 15;
//    	rxConfig.PayLoadSize = CAN_PLSIZE_64;
//
//    	DRV_CANFDSPI_ReceiveChannelConfigure(DRV_CANFDSPI_INDEX_1, APP_RX_FIFO, &rxConfig);
//
//    	// Setup RX Filter
//    	fObj.word = 0;
//    	fObj.bF.SID = 0x300;
//    	fObj.bF.EXIDE = 0;
//    	fObj.bF.EID = 0x00;
//
//    	DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_1, CAN_FILTER0, &fObj.bF);
//
//    	// Setup RX Mask
//    	mObj.word = 0;
//    	mObj.bF.MSID = 0x7F8;
//    	mObj.bF.MIDE = 1; // Only allow standard IDs
//    	mObj.bF.MEID = 0x0;
//    	DRV_CANFDSPI_FilterMaskConfigure(DRV_CANFDSPI_INDEX_1, CAN_FILTER0, &mObj.bF);
//
//    	// Link FIFO and Filter
//    	DRV_CANFDSPI_FilterToFifoLink(DRV_CANFDSPI_INDEX_1, CAN_FILTER0, APP_RX_FIFO, true);
//
//    	// Setup Bit Time
////    	DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_1, CAN_500K_2M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
//    	DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_1, CAN_125K_500K, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
//    	// Setup Transmit and Receive Interrupts
////    	DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_1, GPIO_MODE_INT, GPIO_MODE_INT);
////    	DRV_CANFDSPI_TransmitChannelEventEnable(DRV_CANFDSPI_INDEX_1, APP_TX_FIFO, CAN_TX_FIFO_NOT_FULL_EVENT);
////    	DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_1, APP_RX_FIFO, CAN_RX_FIFO_NOT_EMPTY_EVENT);
////    	DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_1, CAN_TX_EVENT | CAN_RX_EVENT);
//
//    	// Select Normal Mode
//    	//DRV_CANFDSPI_OperationModeSelect(CAN_NORMAL_MODE);
//
//    	DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_1, CAN_EXTERNAL_LOOPBACK_MODE);
}

void DRV_SPI_Initialize(void)
{
    DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0);

    CAN_OSC_CTRL oscCtrl;
    DRV_CANFDSPI_OscillatorControlObjectReset(&oscCtrl);
    oscCtrl.ClkOutDivide = OSC_CLKO_DIV10;
    DRV_CANFDSPI_OscillatorControlSet(DRV_CANFDSPI_INDEX_0, oscCtrl);

    DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);
    CAN_CONFIG config;
    DRV_CANFDSPI_ConfigureObjectReset(&config);
    config.IsoCrcEnable = 1;
    config.StoreInTEF = 1;
    config.TXQEnable = 1;
    DRV_CANFDSPI_Configure(DRV_CANFDSPI_INDEX_0, &config);
    DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_500K_2M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
    CAN_TEF_CONFIG tefConfig;
    tefConfig.FifoSize = 11;
    tefConfig.TimeStampEnable = 1;
    DRV_CANFDSPI_TefConfigure(DRV_CANFDSPI_INDEX_0, &tefConfig);
    CAN_TX_QUEUE_CONFIG txqConfig;
    DRV_CANFDSPI_TransmitQueueConfigureObjectReset(&txqConfig);
    txqConfig.TxPriority = 1;
    txqConfig.FifoSize = 7;
    txqConfig.PayLoadSize = CAN_PLSIZE_32;
    DRV_CANFDSPI_TransmitQueueConfigure(DRV_CANFDSPI_INDEX_0, &txqConfig);

    CAN_TX_FIFO_CONFIG txfConfig;
    DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txfConfig);
    txfConfig.FifoSize = 4;
    txfConfig.PayLoadSize = CAN_PLSIZE_64;
    txfConfig.TxPriority = 0;
    DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH1, &txfConfig);

    CAN_RX_FIFO_CONFIG rxfConfig;
    rxfConfig.FifoSize = 15;
    rxfConfig.PayLoadSize = CAN_PLSIZE_64;
    rxfConfig.RxTimeStampEnable = 1;
    DRV_CANFDSPI_ReceiveChannelConfigure(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH2, &rxfConfig);

    DRV_CANFDSPI_EccEnable(DRV_CANFDSPI_INDEX_0);
    DRV_CANFDSPI_RamInit(DRV_CANFDSPI_INDEX_0, 0xff);
    DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_NORMAL_MODE);

}

/**
  * @brief  SPIƬѡ�źſ���
  * @param  spiSlaveDeviceIndex: ��·SPIѡ��Ҫ����оƬ�ͺ����жϿɳ���·
  * @param  assert: �Ƿ�ѡ��ģ�顣true�������ͣ�false��������
  * @retval -1����Ƭѡʧ�ܣ�0����Ƭѡ���
  */
int8_t DRV_SPI_ChipSelectAssert(uint8_t spiSlaveDeviceIndex, bool assert)
{
    int8_t error = 0;

    // Select Chip Select
    switch (spiSlaveDeviceIndex) {
        case DRV_CANFDSPI_INDEX_0:
            if (assert)
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            else
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            break;
        case DRV_CANFDSPI_INDEX_1:
			if (assert)
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			else
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			break;
        default:
            error = -1;
            break;
    }
    return error;
}


/**
  * @brief  MCP2517FDģ�����ݷ��ͽ��պ���
  * @param  spiSlaveDeviceIndex: ��·SPIѡ��
  * @param  SpiTxData: ���͵�����
  * @param  SpiRxData: ���յ�����
  * @param  spiTransferSize: �������ݵĳ���
  * @retval Ƭѡ�ź�״̬  -1����Ƭѡʧ�ܣ�0����Ƭѡ�ɹ�
  */
uint8_t test_0 = 0;
uint8_t test_1 = 0;
uint8_t *rx_ptr;
int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
//	static uint8_t *rx_ptr;
    int8_t error = 0;
    // Assert CS
    error = DRV_SPI_ChipSelectAssert(spiSlaveDeviceIndex, true);
    if (error != 0)
        return error;

    switch (spiSlaveDeviceIndex){
        case DRV_CANFDSPI_INDEX_0:
            HAL_SPI_TransmitReceive(&hspi1,SpiTxData,SpiRxData,spiTransferSize,1000);
            rx_ptr = SpiRxData;
            test_0++;
            break;
        case DRV_CANFDSPI_INDEX_1:
        	HAL_SPI_TransmitReceive(&hspi2,SpiTxData,SpiRxData,spiTransferSize,1000);
        	test_1++;
        	break;
        default:
            break;
    }
    // De�\assert CS
    error = DRV_SPI_ChipSelectAssert(spiSlaveDeviceIndex, false);

    return error;
} 

struct canfd_frame frame;
void mcp2518fd_transpond(void)
{
		CAN_RX_FIFO_EVENT rxFlags;
		CAN_RX_MSGOBJ rxObj;
//		struct canfd_frame frame;
		uint32_t rxif;
		CAN_MODULE_EVENT eventflags;
		CAN_ECC_EVENT eccflags;
		CAN_RXCODE rxCode;

		memset(&frame, 0, sizeof(frame));

		DRV_CANFDSPI_ModuleEventGet(DRV_CANFDSPI_INDEX_0, &eventflags);
		DRV_CANFDSPI_ReceiveEventGet(DRV_CANFDSPI_INDEX_0, &rxif);
		DRV_CANFDSPI_ModuleEventRxCodeGet(DRV_CANFDSPI_INDEX_0, &rxCode);
		// printf("rxif:%d, rxCode:%d\r\n",(int)rxif, (int)rxCode);
		if ((!rxif) && (!rxCode)) {
			printf("111eventflags:%d\r\n", eventflags);
			printf("rxif:%d\r\n",(int)rxif);
			DRV_CANFDSPI_ModuleEventRxCodeGet(DRV_CANFDSPI_INDEX_0, &rxCode);
			printf("0000rxCode:%d\r\n", rxCode);
			CAN_TXCODE txCode;
			DRV_CANFDSPI_ModuleEventTxCodeGet(DRV_CANFDSPI_INDEX_0, &txCode);
			printf("0000txCode:%d\r\n", txCode);
			CAN_FILTER filterHit;
			DRV_CANFDSPI_ModuleEventFilterHitGet(DRV_CANFDSPI_INDEX_0, &filterHit);
			printf("0000filterHits:%d\r\n", filterHit);
			CAN_ICODE icode;
			DRV_CANFDSPI_ModuleEventIcodeGet(DRV_CANFDSPI_INDEX_0, &icode);
			printf("0000icode:%d\r\n", icode);
			CAN_CRC_EVENT crcflags;
			DRV_CANFDSPI_CrcEventGet(DRV_CANFDSPI_INDEX_0, &crcflags);
			printf("0000crcflags:%d\r\n", crcflags);
			CAN_TEF_FIFO_EVENT tefflags;
			DRV_CANFDSPI_TefEventGet(DRV_CANFDSPI_INDEX_0, &tefflags);
			printf("0000tefflags:%d\r\n", tefflags);
			DRV_CANFDSPI_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_0,CAN_FIFO_CH2, &rxFlags);
			printf("0000rxFlags:%d\r\n", rxFlags);
			DRV_CANFDSPI_ModuleEventGet(DRV_CANFDSPI_INDEX_0, &eventflags);
			printf("2222eventflags:%d\r\n", eventflags);
			uint32_t rxovif;
			DRV_CANFDSPI_ReceiveEventOverflowGet(DRV_CANFDSPI_INDEX_0, &rxovif);
			printf("0000rxovif:%d\r\n", rxovif);
			uint8_t tec, rec;
			CAN_ERROR_STATE flags;
			DRV_CANFDSPI_ErrorCountStateGet(DRV_CANFDSPI_INDEX_0, &tec, &rec, &flags);
			printf("tec:%d, rec:%d, flags:%d\r\n", tec, rec, flags);
			CAN_BUS_DIAGNOSTIC bd;
			DRV_CANFDSPI_BusDiagnosticsGet(DRV_CANFDSPI_INDEX_0, &bd);
			printf("bd:%d\r\n", bd);
			DRV_CANFDSPI_ReceiveChannelEventOverflowClear(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH2);
			return;
		}

		int count, start;
		for (count = 0, start = CAN_FIFO_CH2;
			count < 1; count++, start++) {
			if (rxif & BIT(start) || (rxCode == start)) {
			// 	printf("BIT(start):%d\r\n",(int)BIT(start));
				// DRV_CANFDSPI_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_0, start, &rxFlags);
				// printf("111rxFlags:%d\r\n", rxFlags);
				// if (rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) {
					DRV_CANFDSPI_ReceiveMessageGetBulk(DRV_CANFDSPI_INDEX_0, start, &rxObj, &frame.data[0], MAX_DATA_BYTES);
					if (rxObj.bF.ctrl.IDE) {
						frame.can_id = (rxObj.bF.id.SID << 18) + rxObj.bF.id.EID;
						frame.can_id |= CAN_FMT;
					} else {
						frame.can_id = rxObj.bF.id.SID;
					}

					// if (rxObj.bF.ctrl.ESI) {
					// 	frame.flags |= CAN_ESI_FLAG;
					// 	frame.can_id |= CAN_ERR;
					// } else {
					// 	frame.flags &= ~CAN_ESI_FLAG;
					// 	frame.can_id &= ~CAN_ERR;
					// }

					// if (rxObj.bF.ctrl.RTR) {
					// 	frame.can_id |= CAN_RTR;
					// } else {
					// 	frame.can_id &= ~CAN_RTR;
					// }
					// printf("\r\n Receive message's IDs = %x, DLC = %d, FLAG = %x\r\n", (unsigned int)frame.can_id, rxObj.bF.ctrl.DLC, frame.flags);
					frame.d_len = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)rxObj.bF.ctrl.DLC);
				// }
				// DRV_CANFDSPI_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_0, start, &rxFlags);
				// printf("222rxFlags:%d\r\n", rxFlags);
			}
		}
		// DRV_CANFDSPI_ModuleEventGet(DRV_CANFDSPI_INDEX_0, &eventflags);
		// printf("222eventflags:%d\r\n", eventflags);
		// DRV_CANFDSPI_EccEventGet(DRV_CANFDSPI_INDEX_0, &eccflags);
		// printf("eccflags:%d\r\n", eccflags);
		DRV_CANFDSPI_EccEventClear(DRV_CANFDSPI_INDEX_0, CAN_ECC_ALL_EVENTS);
		DRV_CANFDSPI_ModuleEventClear(DRV_CANFDSPI_INDEX_0, CAN_ALL_EVENTS);
}

CAN_TX_MSGOBJ txObj;
uint8_t flag_transmit = 0;
//CAN_TX_MSGOBJ txObj;
uint8_t txd[MAX_DATA_BYTES];
void mcp2518fd_transmit(void) {
//	uint8_t attempts = 50;
	uint8_t n;
	int16_t i;
	bool flush = true;
	static uint16_t messageID_add = 0;

    /**********************Prepare Data****************************************/
//    Nop();
//    Nop();
    txObj.bF.id.SID = 0x300 + ((messageID_add++) & 0xF);

    txObj.bF.ctrl.DLC = CAN_DLC_64;
    txObj.bF.ctrl.IDE = 0; //0: standard frame | 1: extended frame
    txObj.bF.ctrl.BRS = 1; //switch bit rate
    txObj.bF.ctrl.FDF = 1; //1: CAN FD frame | 0: CAN frame

    n = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC) txObj.bF.ctrl.DLC);
    //create random data with size of buffer = size of DLC
    for (i = 0; i < 2; i++)
    {
        txd[i] = rand() & 0xff;
    }

//    while(ext5_cnt == 0);

    DRV_CANFDSPI_TransmitChannelEventGet(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txFlags);
//
    while (!(txFlags & CAN_TX_FIFO_NOT_FULL_EVENT)); //WAIT

//    if (txFlags & CAN_TX_FIFO_NOT_FULL_EVENT) {
	DRV_CANFDSPI_TransmitChannelLoad(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txObj, txd, n, flush);
	printf("\r\n Transmit message's ID = %04x, and txd[0] = %02x", txObj.bF.id.SID, txd[0]);
	flag_transmit = 1;
	ext5_cnt = 0;
//    }
}

void mcp2518fd_receive(void) {
//	DRV_CANFDSPI_2_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &rxFlags);
//	if (rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) {
		// Get message
	DRV_CANFDSPI_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_1, APP_RX_FIFO, &rxFlags);
	if (rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) {
		DRV_CANFDSPI_ReceiveMessageGet(DRV_CANFDSPI_INDEX_1, APP_RX_FIFO, &rxObj, rxd, MAX_DATA_BYTES);
		printf("\r\n Receive message's ID = %04x, and rxd[0] = %02x", txObj.bF.id.SID, rxd[0]);
	}
//	rxFlags = CAN_RX_FIFO_NO_EVENT;
//	}
}


void CANFDSPI_Test( void )
{
    uint8_t attempts = 50;
    uint8_t n;
    int16_t i;
    static uint16_t messageID_add = 0;

    /**********************Prepare Data****************************************/
//    Nop();
//    Nop();
    txObj.bF.id.SID = 0x300 + ((messageID_add++) & 0xF);

    txObj.bF.ctrl.DLC = CAN_DLC_64;
    txObj.bF.ctrl.IDE = 0;
    txObj.bF.ctrl.BRS = 1;
    txObj.bF.ctrl.FDF = 1;

    n = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC) txObj.bF.ctrl.DLC);

    for (i = 0; i < n; i++)
    {
        txd[i] = rand() & 0xff;
    }

    /**********************transmit********************************************/
    // Check if FIFO is not full
    do {
#ifdef APP_USE_TX_INT

#else
        DRV_CANFDSPI_TransmitChannelEventGet(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txFlags);
#endif
//        if (attempts == 0) {
//            Nop();
//            Nop();
//            DRV_CANFDSPI_ErrorCountStateGet(&tec, &rec, &errorFlags);
//            return;
//        }
//        attempts--;
    }
#ifdef APP_USE_TX_INT
    while (nINT0_Get());
#else
    while (!(txFlags & CAN_TX_FIFO_NOT_FULL_EVENT));
#endif

    DRV_CANFDSPI_TransmitChannelLoad(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txObj, txd, n, true);
    printf("\r\n Transmit message's ID = %04x, and txd[0] = %02x", txObj.bF.id.SID, txd[0]);

//    SYSTICK_DelayMs (10);
    /**********************receive*********************************************/
    // Check if FIFO is not empty
#ifdef APP_USE_RX_INT
    if (!nINT1_Get()) {
#else
    DRV_CANFDSPI_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_1, APP_RX_FIFO, &rxFlags);
    if (rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) {
#endif
        // Get message
        DRV_CANFDSPI_ReceiveMessageGet(DRV_CANFDSPI_INDEX_1, APP_RX_FIFO, &rxObj, rxd, MAX_DATA_BYTES);

        printf("\r\n Receive message's ID = %04x, and rxd[0] = %02x", txObj.bF.id.SID, rxd[0]);
    }

}
