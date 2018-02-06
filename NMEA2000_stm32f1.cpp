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

#include <NMEA2000_stm32f1.h>
#include <STM32F1_CAN.h>
//extern "C" void canIsrTaskEntry(void* pvParameters);
//extern "C" void USB_LP_CAN1_RX0_IRQHandler(void);

NMEA2000_stm32f1::NMEA2000_stm32f1() :
		tNMEA2000() {
	SetDeviceCount(1);
	SetN2kCANMsgBufSize(5);
	SetN2kCANSendFrameBufSize(5);

	STM32F1_CAN::getInstance().begin(true);
	//  SetMode(tNMEA2000::N2km_ListenAndNode, N2K_Def_DevId);
	//  EnableForward(false); // Disable all msg forwarding to USB (=Serial)
	//  Open();
}

NMEA2000_stm32f1::~NMEA2000_stm32f1() {
}

bool NMEA2000_stm32f1::CANOpen() {

	return true;
}

struct canStat {
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
bool NMEA2000_stm32f1::CANSendFrame(unsigned long id, unsigned char len,
		const unsigned char* buf, bool wait_sent) {
	//JCB TODO    (void)wait_sent;
	STM32F1_CAN::CanMsgTypeDef frametx;
	frametx.id = id;
	frametx.len = len;

	for (int i = 0; i < len; i++)
		frametx.Data[i] = buf[i];
	return STM32F1_CAN::getInstance().write(frametx, wait_sent);
}

/* Realization of tNMEA2000 virtual function. Notice that this implementation
 * actually
 */
bool NMEA2000_stm32f1::CANGetFrame(unsigned long& id, unsigned char& len,
		unsigned char* buf) {
	STM32F1_CAN::CanMsgTypeDef frame;

	if (STM32F1_CAN::getInstance().read(frame) == true) {
		id = frame.id;
		len = frame.len;
		for (int i = 0; i < len; i++) {
			buf[i] = frame.Data[i];
		}
		return true;
	}
	return false;

}
void NMEA2000_stm32f1::InitCANFrameBuffers() {
	if (MaxCANReceiveFrames == 0)
		MaxCANReceiveFrames = 10; // Use default, if not set
	if (MaxCANReceiveFrames < 10)
		MaxCANReceiveFrames = 10; // Do not allow less that 10 should have enough memory.
	STM32F1_CAN::getInstance().setRxBufferSize(MaxCANReceiveFrames);
	MaxCANSendFrames = 4;

#ifdef INTTX
//TODO
#if 0
	CANbus->setNumTXBoxes(NumTxMailBoxes);

// With this support system can have different buffers for high and low priority and fast packet messages.
// After message has been sent to driver, it buffers it automatically and sends it by interrupt.
// We may need to make these possible to set.
	uint16_t HighPriorityBufferSize=CANGlobalBufSize / 10;
	HighPriorityBufferSize=(HighPriorityBufferSize<15?HighPriorityBufferSize:15);// just guessing
	CANGlobalBufSize-=HighPriorityBufferSize;
	uint16_t FastPacketBufferSize= (CANGlobalBufSize * 9 / 10);
	CANGlobalBufSize-=FastPacketBufferSize;

	CANbus->setMailBoxTxBufferSize(CANbus->getFirstTxBox(),HighPriorityBufferSize);// Highest priority buffer
	CANbus->setMailBoxTxBufferSize(CANbus->getLastTxBox(),FastPacketBufferSize);// Fastpacket buffer
	STM32F1_CAN::getInstance().setTxBufferSize(CANGlobalBufSize);
#endif
#endif

	tNMEA2000::InitCANFrameBuffers(); // call main initialization
}

#if 0
extern "C" void CAN1_TX_IRQHandler(void) {
	CanTxMsgTypeDef frame;

	if (popTX(&frame) == true) {
		memcpy(CanHandle.pTxMsg,&frame,sizeof(CanTxMsgTypeDef));
		HAL_CAN_Transmit_IT(&CanHandle);  // Rearm transmit
	}
	HAL_CAN_IRQHandler(&CanHandle);
}
#if 0
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* _canHandle)
{
	HAL_CAN_Transmit_IT(_canHandle); // Rearm transmit
}
#endif
#endif
