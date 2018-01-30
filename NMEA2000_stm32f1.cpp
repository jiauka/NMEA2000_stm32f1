/*
 NMEA2000_stm32f1.cpp



Copyright (c) 2017-2018 Jaume Clarens jiauka_at_gmail_dot_com

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Inherited NMEA2000 object for the stm32f103c8 internal CAN
based setup. See also NMEA2000 library.
  */

#include "Arduino.h"
#include <cstring>
#include <string.h>
#define USE_CAN_EXT_ID
#include <NMEA2000_stm32f1.h>


//#define INTTX

extern "C" void canIsrTaskEntry(void* pvParameters);
extern "C" void USB_LP_CAN1_RX0_IRQHandler(void);


static QueueTX canOutQ;
static QueueRX canInQ;
static CAN_HandleTypeDef    CanHandle;


static int isFullRX()
{
    if ((canInQ.front == canInQ.rear + 1) ||
        (canInQ.front == 0 && canInQ.rear == canInQ.size - 1))
        return 1;
    return 0;
}

static int isEmptyRX()
{
    if (canInQ.front == -1)
        return 1;
    return 0;
}

static bool pushRX(CanRxMsgTypeDef* item)
{
    if (isFullRX())
        return false;
    else {
        if (canInQ.front == -1)
            canInQ.front = 0;
        canInQ.rear = (canInQ.rear + 1) % canInQ.size;
        //		trace_puts("[pushRX] 1");
        memcpy(&canInQ.elements[canInQ.rear], item, sizeof(CanRxMsgTypeDef));
    }
    return true;
}

static bool popRX(CanRxMsgTypeDef* item)
{
    if (isEmptyRX()) {
        return false;
    } else {
        //		trace_puts("[popRX] 1");
        memcpy(item, &canInQ.elements[canInQ.front], sizeof(CanRxMsgTypeDef));
        if (canInQ.front ==
            canInQ.rear) { /* Q has only one element, so we reset the queue
                              after dequeing it. */
            canInQ.front = -1;
            canInQ.rear = -1;
        } else {
            canInQ.front = (canInQ.front + 1) % canInQ.size;
        }
        return true;
    }
}

NMEA2000_stm32f1::NMEA2000_stm32f1()
  : tNMEA2000()
{
	static CanTxMsgTypeDef        TxMessage;
	static CanRxMsgTypeDef        RxMessage;
	CAN_FilterConfTypeDef  sFilterConfig;
    GPIO_InitTypeDef GPIO_InitStruct;


	SetDeviceCount(1);
    canOutQ.size = MAXELEMENTS;
    canOutQ.front = -1;
    canOutQ.rear = -1;

    canInQ.size = MAXELEMENTS;
    canInQ.front = -1;
    canInQ.rear = -1;

 #if 0 // def INTTX
    NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
      configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
    __HAL_RCC_CAN1_CLK_ENABLE();

      /* Enable GPIO clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Enable AFIO clock and remap CAN PINs to PB_8 and PB_9*/
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 RX GPIO pin configuration */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    /* Configure CAN1 **************************************************/
    /* Struct init*/
    CanHandle.Instance = CAN1;
    CanHandle.pTxMsg = &TxMessage;
    CanHandle.pRxMsg = &RxMessage;

    CanHandle.Init.TTCM = DISABLE;
    CanHandle.Init.ABOM = DISABLE;
    CanHandle.Init.AWUM = DISABLE;
    CanHandle.Init.NART = DISABLE;
    CanHandle.Init.RFLM = DISABLE;
    CanHandle.Init.TXFP = ENABLE; // JCB ENABLE  ;
    CanHandle.Init.Mode = CAN_MODE_NORMAL;
    CanHandle.Init.SJW = CAN_SJW_1TQ;
    CanHandle.Init.BS1 = CAN_BS1_3TQ;
    CanHandle.Init.BS2 = CAN_BS2_5TQ;
    CanHandle.Init.Prescaler = 16;

    /*Initializes the CAN1 */
    HAL_CAN_Init(&CanHandle);

    /* CAN filter init */
    sFilterConfig.FilterNumber = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.BankNumber = 0;
    if (HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
    {
    	   __HAL_CAN_ENABLE_IT(&CanHandle, CAN_IT_FMP0);
      /* Filter configuration Error */
//      Error_Handler();
    	int i=1;
    }
    HAL_CAN_Receive_IT(&CanHandle,CAN_FIFO0);
//    		__HAL_CAN_ENABLE_IT(&CanHandle, CAN_IT_FMP0);
   HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

#if 0 // def INTTX
    CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);
#endif
    SetN2kCANMsgBufSize(5);
    SetN2kCANSendFrameBufSize(5);
  //  SetMode(tNMEA2000::N2km_ListenAndNode, N2K_Def_DevId);
  //  EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  //  Open();
}

NMEA2000_stm32f1::~NMEA2000_stm32f1()
{
}

bool NMEA2000_stm32f1::CANOpen()
{
    //	mcp.begin(CAN_250KBPS, MCP_8MHz);

    return true;
}

struct canStat
{
    uint32_t sendCnt;
    uint32_t isrCnt;
    uint32_t semCnt;
    uint32_t txCnt;
    uint32_t rxCnt;
    uint32_t txEmpty;
    uint32_t isrEnable;
    uint32_t isrDisable;
    uint32_t lostFrame;
} canStat;

/* Realization of tNMEA2000 virtual function. Notice that this implementation
 * ignores wait_sent flag.
 * It always wait for frame to be sent.
 * This is considered acceptable because it was called either once for short
 * packages with 'no-wait'
 * or multiple times for long packages with 'wait'. There seems to be no reason
 * why we can't also wait
 * on short packages.
 */
bool NMEA2000_stm32f1::CANSendFrame(unsigned long id,
                                    unsigned char len,
                                    const unsigned char* buf,
                                    bool wait_sent)
{
	uint32_t transmit_mailbox = CAN_TXSTATUS_NOMAILBOX;
	bool ret;
#if 0 //JCB TODO    (void)wait_sent;
	CanTxMsgTypeDef frametx;
    uint8_t transmit_mailbox = CAN_TxStatus_NoMailBox;
    if (len > 8) {
        return -1;
    }
#ifndef INTTX
#ifdef USE_CAN_EXT_ID
    frametx.IDE = CAN_ID_EXT;
#else
    frametx.IDE = CAN_ID_STD;
#endif
#ifdef USE_CAN_EXT_ID
    frametx.ExtId = id;
#else
    frametx.StdId = id;
#endif
    frametx.DLC = len;
    frametx.RTR = CAN_RTR_DATA;
    //  trace_puts("[CANSendFrame] 2");
    memcpy(frametx.Data, buf, len);

    while (transmit_mailbox == CAN_TxStatus_NoMailBox) {
        transmit_mailbox = CAN_Transmit(CAN1, &frametx);
    }
    //	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    return true;
#else
#ifdef USE_CAN_EXT_ID
    frametx.IDE = CAN_ID_EXT;
#else
    frametx.IDE = CAN_ID_STD;
#endif
#ifdef USE_CAN_EXT_ID
    frametx.ExtId = id;
#else
    frametx.StdId = id;
#endif
    frametx.DLC = len;
    frametx.RTR = CAN_RTR_DATA;
    //  trace_puts("[CANSendFrame] 2");
    memcpy(frametx.Data, buf, len);
#if 0
	if(CAN_GetITStatus(CAN1, CAN_IT_TME) == RESET) {
		CAN_Transmit(CAN1, &frametx);
		CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);
		canStat.sendCnt++;
		CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
		return true;
	}
	else
#endif
    {
        if (pushTX(&frametx)) {
            //			trace_puts("[CANSendFrame] 1");
            NVIC_SetPendingIRQ(USB_HP_CAN1_TX_IRQn);
            canStat.sendCnt++;
            return true;
        }
    }
    canStat.lostFrame++;
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    return false;
#endif

#else
    int i = 0;
#ifdef USE_CAN_EXT_ID
    CanHandle.pTxMsg->IDE = CAN_ID_EXT;
#else
    CanHandle.pTxMsg->IDE = CAN_ID_STD;
#endif
#ifdef USE_CAN_EXT_ID
    CanHandle.pTxMsg->ExtId = id;
    CanHandle.pTxMsg->StdId = 0;
#else
    CanHandle.pTxMsg->ExtId = 0;
    CanHandle.pTxMsg->StdId = id;
#endif


   CanHandle.pTxMsg->RTR = CAN_RTR_DATA;
   CanHandle.pTxMsg->DLC = len;

     for(i = 0; i < len; i++)
    	 CanHandle.pTxMsg->Data[i] = buf[i];

     if(HAL_CAN_Transmit(&CanHandle, 10) != HAL_OK) {
    	 ret= false;
     }
     else
    	 ret= true;
     __HAL_CAN_ENABLE_IT(&CanHandle, CAN_IT_FMP0);
      return ret;
#endif
}

/* Realization of tNMEA2000 virtual function. Notice that this implementation
 * actually
  */
bool NMEA2000_stm32f1::CANGetFrame(unsigned long& id,
                                   unsigned char& len,
                                   unsigned char* buf)
{
    CanRxMsgTypeDef frame;
    if (popRX(&frame) == true) {
        if (frame.IDE == CAN_ID_STD)
            id = frame.StdId;
        else
            id = frame.ExtId;
        len = frame.DLC;
        for (int i = 0; i < len; i++) {
            buf[i] = frame.Data[i];
        }
        return true;
    }
    return false;
}

bool NMEA2000_stm32f1::SendMsg(const tN2kMsg& N2kMsg, int DeviceIndex)
{
    bool res;

//    xSemaphoreTake(sendMutex, portMAX_DELAY);
    res = tNMEA2000::SendMsg(N2kMsg, DeviceIndex);
 //   xSemaphoreGive(sendMutex);
    return res;
}


extern "C" void USB_LP_CAN1_RX0_IRQHandler(void)
{
    //	long lHigherPriorityTaskWoken = pdFALSE;
#if 0
	CanRxMsgTypeDef frame;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
        CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
        //		xSemaphoreGiveFromISR(canIsrSem,
        //&lHigherPriorityTaskWoken);
        canStat.isrCnt++;
        CAN_Receive(CAN1, CAN_FIFO0, &frame);
        pushRX(&frame);
        CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

        //		CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
        //		portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
    }
#else
    HAL_CAN_IRQHandler(&CanHandle);

#endif

}
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _canHandle) {
#if 0
    pushRX(_canHandle->pRxMsg);
    if(HAL_CAN_Receive_IT(_canHandle, CAN_FIFO0) == HAL_OK) {

    }
		// BUG: CAN race condition if HAL_CAN_Receive_IT() is used.
    // See https://my.st.com/public/STe2ecommunities/mcu/Lists/STM32Java/Flat.aspx?RootFolder=%2Fpublic%2FSTe2ecommunities%2Fmcu%2FLists%2FSTM32Java%2FBUG%20CAN%20race%20condition%20if%20HAL%5FCAN%5FReceive%5FIT%20is%20used
    //
    // Fixed by Mark Burton:
    // ideally, we should be able to call HAL_CAN_Receive_IT() here to set up for another
    // receive but the API is flawed because that function will fail if HAL_CAN_Transmit()
    // had already locked the handle when the receive interrupt occurred - so we do what
    // HAL_CAN_Receive_IT() would do

 //   if (rxCompleteCallback != NULL)
  //      rxCompleteCallback();
#else
    if (_canHandle->State == HAL_CAN_STATE_BUSY_TX)
        _canHandle->State = HAL_CAN_STATE_BUSY_TX_RX0;
    else {
        pushRX(_canHandle->pRxMsg);

    	_canHandle->State = HAL_CAN_STATE_BUSY_RX0;

        /* Set CAN error code to none */
        _canHandle->ErrorCode = HAL_CAN_ERROR_NONE;

        /* Enable Error warning Interrupt */
        __HAL_CAN_ENABLE_IT(_canHandle, CAN_IT_EWG);

        /* Enable Error passive Interrupt */
        __HAL_CAN_ENABLE_IT(_canHandle, CAN_IT_EPV);

        /* Enable Bus-off Interrupt */
        __HAL_CAN_ENABLE_IT(_canHandle, CAN_IT_BOF);

        /* Enable Last error code Interrupt */
        __HAL_CAN_ENABLE_IT(_canHandle, CAN_IT_LEC);

        /* Enable Error Interrupt */
        __HAL_CAN_ENABLE_IT(_canHandle, CAN_IT_ERR);
    }

    // Enable FIFO 0 message pending Interrupt
    __HAL_CAN_ENABLE_IT(_canHandle, CAN_IT_FMP0);
#endif
}

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* _canHandle)
{

    __HAL_CAN_ENABLE_IT(_canHandle, CAN_IT_TME);
}

