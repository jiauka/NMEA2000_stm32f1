/*
 * NMEA2000_stm32f1.h


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

#ifndef NMEA2000_stm32f1_H_
#define NMEA2000_stm32f1_H_
//#include "semphr.h"
#include <NMEA2000.h>

#define MAXELEMENTS 5

class NMEA2000_stm32f1 : public tNMEA2000
{

  public:
    NMEA2000_stm32f1();
    virtual ~NMEA2000_stm32f1();

    virtual bool CANSendFrame(unsigned long id,
                              unsigned char len,
                              const unsigned char* buf,
                              bool wait_sent = true);
    virtual bool CANOpen();
    virtual bool CANGetFrame(unsigned long& id,
                             unsigned char& len,
                             unsigned char* buf);

    bool SendMsg(const tN2kMsg& N2kMsg,
                 int DeviceIndex = 0); // Override super class variant
	void InitCANFrameBuffers();
  private:

};

#endif /* NMEA2000_stm32f1_H_ */
