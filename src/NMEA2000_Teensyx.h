/*
  NMEA2000_Teensyx.h

  Copyright (c) 2020-2022 Timo Lappalainen - https://github.com/ttlappalainen

  tNMEA2000_Teensyx library for Teensy 3.x and Teensy 4.x. 
  This will replace old NMEA2000_teensy library in future.

  Code is modified from FlexCAN4 library. I cleaned unnecessary code, added right handling for
  fastpacket, interrupted message sending etc.

  tNMEA2000_Teensyx contains whole CAN code and does not require FlexCAN4 library. The simple reason
  is that I got already frustrated with FlexCAN library, which were not accepted with my NMEA2000
  required extensions. In this way it is much simple to keep library working.

  Thanks for FlexCAN4 library writers for original code.
  
  On constructor you can set bus, tx and rx pins. You can also use defined before constructor:
  #define NMEA2000_TEENSYX_CAN_BUS tNMEA2000_Teensyx::CAN2 // select CAN bus 2
  #define NMEA2000_TEENSYX_TX_PIN tNMEA2000_Teensyx::pinAlternate // Use alternate tx pin
  #define NMEA2000_TEENSYX_RX_PIN tNMEA2000_Teensyx::pinAlternate // Use alternate rx pin
  
  Note that bus numbering differs on between Teensy 4.x and Teensy 3.x.
  

  MIT License

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
#if !defined(__NMEA2000_TEENSY_X__)
#define __NMEA2000_TEENSY_X__

#include "Arduino.h"
#include <NMEA2000.h> 
#include <RingBuffer.h>
#include "imxrt_flexcan.h"

#ifndef NMEA2000_TEENSYX_CAN_BUS
  #if defined(__IMXRT1062__)
    #define NMEA2000_TEENSYX_CAN_BUS tNMEA2000_Teensyx::CAN1
  #endif
  #if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    #define NMEA2000_TEENSYX_CAN_BUS tNMEA2000_Teensyx::CAN0
  #endif
#endif

#ifndef NMEA2000_TEENSYX_TX_PIN
  #define NMEA2000_TEENSYX_TX_PIN tNMEA2000_Teensyx::pinDefault
#endif

#ifndef NMEA2000_TEENSYX_RX_PIN
  #define NMEA2000_TEENSYX_RX_PIN tNMEA2000_Teensyx::pinDefault
#endif

// -----------------------------------------------------------------------------
class tNMEA2000_Teensyx : public tNMEA2000 {
protected:
  bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent);
  bool CANOpen();
  bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf);
  void InitCANFrameBuffers();

public:
  enum tPins { pinAlternate=0, pinDefault=1 };
  enum tCANDevice {
  #if defined(__IMXRT1062__)
    CAN1 = (uint32_t)0x401D0000,
    CAN2 = (uint32_t)0x401D4000,
    CAN3 = (uint32_t)0x401D8000
  #endif
  #if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    CAN0 = (uint32_t)0x40024000,
    CAN1 = (uint32_t)0x400A4000,
  #endif
  };

public:
  tNMEA2000_Teensyx(tCANDevice _bus=NMEA2000_TEENSYX_CAN_BUS, tPins _txPin=NMEA2000_TEENSYX_TX_PIN, tPins _rxPin=NMEA2000_TEENSYX_RX_PIN);

protected:
  struct CAN_message_t {
    uint32_t id = 0;          // can identifier
//    uint16_t timestamp = 0;   // FlexCAN time when message arrived
//    uint8_t idhit = 0; // filter that id came from
    struct {
      bool extended = 0; // identifier is extended (29-bit)
      bool remote = 0;  // remote transmission request packet type
      bool overrun = 0; // message overrun
      bool reserved = 0;
    } flags;
    uint8_t len = 8;      // length of data
    uint8_t buf[8] = { 0 };       // data
//    uint8_t mb = 0;       // used to identify mailbox reception
//    uint8_t bus = 0;      // used to identify where the message came from when events() is used.
//    bool seq = 0;         // sequential frames
  };

  tCANDevice bus;
  tPriorityRingBuffer<CAN_message_t>  *rxRing;
  tPriorityRingBuffer<CAN_message_t>  *txRing;
  uint8_t firstTxBox;
  tPins txPin:1;
  tPins rxPin:1;
  
protected:    
  enum FLEXCAN_RXTX {
    TX,
    RX,
    LISTEN_ONLY
  };

  enum FLEXCAN_IDE {
    NONE = 0,
    EXT = 1,
    RTR = 2,
    STD = 3,
    INACTIVE
  };

  enum FLEXCAN_FLTEN {
    ACCEPT_ALL = 0,
    REJECT_ALL = 1
  };

  void begin();
  uint32_t getBaudRate() { return currentBitrate; }
  void setTX(tPins pin = pinDefault);
  void setRX(tPins pin = pinDefault);
  void setBaudRate(uint32_t baud = 250000, FLEXCAN_RXTX listen_only = TX);
  void reset() { softReset(); } /* reset flexcan controller (needs register restore capabilities...) */
  void setMaxMB(uint8_t last=0xff);
  void enableFIFO(bool status = 1);
  void disableFIFO() { enableFIFO(0); }
  void enableFIFOInterrupt(bool status = 1);
  void disableFIFOInterrupt() { enableFIFOInterrupt(0); }
  bool setMB(uint8_t mb_num, const FLEXCAN_RXTX &mb_rx_tx, const FLEXCAN_IDE &ide = STD);
  void enableMBInterrupt(uint8_t mb_num, bool status = 1);
  void disableMBInterrupt(uint8_t mb_num) { enableMBInterrupt(mb_num, 0); }
  void enableMBInterrupts(bool status = 1);
  void disableMBInterrupts() { enableMBInterrupts(0); }
  void setMRP(bool mrp = 1); /* mailbox(1)/fifo(0) priority */
  void setRRS(bool rrs = 1); /* store remote frames */

  bool sendFromTxRing(uint8_t prio);
//    bool write(const CAN_message_t &msg); /* use any available mailbox for transmitting */
//  void writeTxMailbox(uint8_t mb_num, const CAN_message_t &msg);
  void writeTxMailbox(uint8_t mb_num, unsigned long id, unsigned char len, const unsigned char *buf, bool extended=true);
  uint64_t readIMASK();// { return (((uint64_t)FLEXCANb_IMASK2(bus) << 32) | FLEXCANb_IMASK1(bus)); }
  void flexcan_interrupt();
#if defined(__IMXRT1062__)
  enum FLEXCAN_CLOCK {
    CLK_OFF,
    CLK_8MHz = 8,
    CLK_16MHz = 16,
    CLK_20MHz = 20,
    CLK_24MHz = 24,
    CLK_30MHz = 30,
    CLK_40MHz = 40,
    CLK_60MHz = 60,
    CLK_80MHz = 80
  };

  void setClock(FLEXCAN_CLOCK clock = CLK_24MHz);
  uint32_t getClock();
#endif
  void ExitFreezeMode();
  void EnterFreezeMode();
  uint8_t getNumMailBoxes() { return FLEXCANb_MAXMB_SIZE(bus); }
  uint8_t mailboxOffset();
  void softReset();
  uint8_t getFirstTxBox();
  uint64_t readIFLAG();// { return (((uint64_t)FLEXCANb_IFLAG2(bus) << 32) | FLEXCANb_IFLAG1(bus)); }
  void writeIFLAG(uint64_t value);
  void writeIFLAGBit(uint8_t mb_num);
  void writeIMASK(uint64_t value);
  void writeIMASKBit(uint8_t mb_num, bool set = 1);
  uint32_t currentBitrate = 0UL;
  uint8_t mailbox_reader_increment = 0;
  uint8_t busNumber;

  bool IrqEnabled=false;
  uint32_t IrqMessage=0;

  void irqLock() { IrqEnabled=NVIC_IS_ENABLED(IrqMessage); NVIC_DISABLE_IRQ(IrqMessage); }
  void irqRelease() { if (IrqEnabled) NVIC_ENABLE_IRQ(IrqMessage); }

  #if defined(__IMXRT1062__)
  static void flexcan_isr_can3();
  static void flexcan_isr_can2();
  static void flexcan_isr_can1();
  #endif
  #if defined(__MK20DX256__) || defined(__MK64FX512__)
  static void flexcan_isr_can0();
  #endif
  #if defined(__MK66FX1M0__)
  static void flexcan_isr_can0();
  static void flexcan_isr_can1();
  #endif
};

#endif
