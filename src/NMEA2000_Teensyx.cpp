/*
  NMEA2000_Teensyx.cpp

  Copyright (c) 2020 Timo Lappalainen - https://github.com/ttlappalainen

  tNMEA2000_Teensyx library for Teensy 3.x and Teensy 4.x. 
  This will replace old NMEA2000_teensy library in future.

  Code is modified from FlexCAN4 library. I cleaned unnecessary code, added right handling for
  fastpacket, interrupted message sending etc.

  tNMEA2000_Teensyx contains whole CAN code and does not require FlexCAN4 library. The simple reason
  is that I got already frustrated with FlexCAN library, which were not accepted with my NMEA2000
  required extensions. In this way it is much simple to keep library working.

  Thanks for FlexCAN4 library writers for original code.

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
#include "NMEA2000_Teensyx.h"
#include "imxrt_flexcan.h"
#include "Arduino.h"

//#define TEENSYX_ERROR_DEBUG
//#define TEENSYX_DEBUG

#if defined(TEENSYX_DEBUG) || defined(TEENSYX_ERROR_DEBUG)
  #include <Arduino.h>
  #ifndef DebugStream
  #define DebugStream Serial
  #endif
#endif

#if defined(TEENSYX_ERROR_DEBUG)
  # define TeensyxErrDbgf(fmt, args...)   DebugStream.printf (fmt , ## args)
#else
  # define TeensyxErrDbgf(fmt, args...)
#endif

#if defined(TEENSYX_DEBUG)
  # define TeensyxDbgf(fmt, args...)   DebugStream.printf (fmt , ## args)
#else
  # define TeensyxDbgf(fmt, args...)
#endif


#if defined(__IMXRT1062__)
static tNMEA2000_Teensyx* _CAN1 = 0;
static tNMEA2000_Teensyx* _CAN2 = 0;
static tNMEA2000_Teensyx* _CAN3 = 0;
#endif
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
static tNMEA2000_Teensyx* _CAN0 = 0;
static tNMEA2000_Teensyx* _CAN1 = 0;
#endif

#define INVALID_MAILBOX 0xff

// *****************************************************************************
tNMEA2000_Teensyx::tNMEA2000_Teensyx(tCANDevice _bus) : bus(_bus), rxRing(0), txRing(0), firstTxBox(INVALID_MAILBOX) {
#if defined(__IMXRT1062__)
  if ( bus == CAN3 ) _CAN3 = this;
  if ( bus == CAN2 ) _CAN2 = this;
  if ( bus == CAN1 ) _CAN1 = this;
#endif
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  if ( bus == CAN1 ) _CAN1 = this;
  if ( bus == CAN0 ) _CAN0 = this;
#endif
}

#define NUM_TX_MAILBOXES 8

// *****************************************************************************
bool tNMEA2000_Teensyx::CANOpen() {
  setTX(tNMEA2000_Teensyx::pinDefault);
  begin();
  setBaudRate(250000);

  setMaxMB();
  uint8_t nRxMailBoxes=getNumMailBoxes()-mailboxOffset()-NUM_TX_MAILBOXES;
  uint8_t lastRxBox=mailboxOffset()+nRxMailBoxes;
  int i=mailboxOffset();
  for ( ; i<lastRxBox; i++ ) {
    setMB(i,RX,EXT);
  }
  for ( ; i<getNumMailBoxes(); i++ ) {
    setMB(i,TX,EXT);
  }
  getFirstTxBox();

  enableMBInterrupts();

  return true;
}

#if defined(TEENSYX_DEBUG)
extern void CanIdToN2k(unsigned long id, unsigned char &prio, unsigned long &pgn, unsigned char &src, unsigned char &dst);
#endif

// *****************************************************************************
bool tNMEA2000_Teensyx::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent) {
  uint8_t prio = (uint8_t)((id >> 26) & 0x7);
  bool ret=false;

  irqLock();

  uint8_t mb=getFirstTxBox()+prio;
  bool mbBusy=FLEXCAN_get_code(FLEXCANb_MBn_CS(bus, mb)) != FLEXCAN_MB_CODE_TX_INACTIVE;
  bool SendFromBuffer=false;
#if defined(TEENSYX_DEBUG)
  unsigned long pgn;
  unsigned char src;
  unsigned char dst;
  CanIdToN2k(id,prio,pgn,src,dst);
  TeensyxDbgf("%6lu - tNMEA2000_Teensyx::CANSendFrame pgn:%6lu, prio:%u, src:%u, dst:%u, data:%02X, mb:%u\n",millis(),pgn,prio,src,dst,buf[0],mb);
#endif

  // If tx buffer has this priority data or mailbox is busy, buffer this
  if ( !txRing->isEmpty(prio) || mbBusy ) {
    CAN_message_t *msg=txRing->getAddRef(prio);
    if ( msg!=0 ) {
      msg->id=id;
      msg->len=len;
      memcpy(msg->buf,buf,len<=8?len:8);
      ret=true;
      TeensyxDbgf("  frame buffered\n");
    }
    SendFromBuffer=true;
  }

  if ( !mbBusy ) {
    if ( SendFromBuffer ) {
      sendFromTxRing(prio);
    } else {
      writeTxMailbox(mb, id, len, buf);
    }
    ret=true; /* transmit entry accepted */
  }

  irqRelease();

  return ret; /* transmit entry failed, no mailboxes or queues available */
}

// *****************************************************************************
bool tNMEA2000_Teensyx::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  bool ret=false;

  irqLock();
  const CAN_message_t *msg=rxRing->getReadRef();
  if ( msg!=0 ) {
    id=msg->id;
    len=msg->len;
    if ( len>8 ) len=8;
    memcpy(buf,msg->buf,len);
    ret=true;
  }
  irqRelease();

  #if defined(XTEENSYX_DEBUG)
    unsigned long pgn;
    unsigned char prio;
    unsigned char src;
    unsigned char dst;
    CanIdToN2k(id,prio,pgn,src,dst);
    TeensyxDbgf("%6lu - tNMEA2000_Teensyx::CANGetFrame pgn:%6lu, prio:%u, src:%u, dst:%u, data:%02X\n",millis(),pgn,prio,src,dst,buf[0]);
  #endif

  return ret;
}

// *****************************************************************************
void tNMEA2000_Teensyx::InitCANFrameBuffers() {
  #if defined(__IMXRT1062__)
  if ( MaxCANReceiveFrames==0 ) MaxCANReceiveFrames=60; // Use default, if not set
  if ( MaxCANReceiveFrames<30 ) MaxCANReceiveFrames=30; // Do not allow less than 30 - Teensy 4.x should have enough memory.
  if (MaxCANSendFrames==0 ) MaxCANSendFrames=80;  // Use big enough default buffer
  if (MaxCANSendFrames<50 ) MaxCANSendFrames=50; // Do not allow less than 30 - Teensy should have enough memory.
  #else
  if ( MaxCANReceiveFrames==0 ) MaxCANReceiveFrames=32; // Use default, if not set
  if ( MaxCANReceiveFrames<10 ) MaxCANReceiveFrames=10; // Do not allow less than 10 - Teensy should have enough memory.
  if (MaxCANSendFrames==0 ) MaxCANSendFrames=50;  // Use big enough default buffer
  if (MaxCANSendFrames<30 ) MaxCANSendFrames=30; // Do not allow less than 30 - Teensy should have enough memory.
  #endif

  if ( rxRing!=0 && rxRing->getSize()!=MaxCANReceiveFrames ) {
    delete rxRing;
    rxRing=0;
  }
  if ( txRing!=0 && txRing->getSize()!=MaxCANSendFrames ) {
    delete rxRing;
    txRing=0;
  }

  if ( rxRing==0 ) rxRing=new tPriorityRingBuffer<CAN_message_t>(MaxCANReceiveFrames,7);
  if ( txRing==0 ) txRing=new tPriorityRingBuffer<CAN_message_t>(MaxCANSendFrames,7);
}

#if defined(__IMXRT1062__)
// *****************************************************************************
void tNMEA2000_Teensyx::setClock(FLEXCAN_CLOCK clock) {
  if ( clock == CLK_OFF ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(3) | CCM_CSCMR2_CAN_CLK_PODF(0);
  if ( clock == CLK_8MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(9);
  if ( clock == CLK_16MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(4);
  if ( clock == CLK_24MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(1) | CCM_CSCMR2_CAN_CLK_PODF(0);
  if ( clock == CLK_20MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(3);
  if ( clock == CLK_30MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(0) | CCM_CSCMR2_CAN_CLK_PODF(1);
  if ( clock == CLK_40MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(1);
  if ( clock == CLK_60MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(0) | CCM_CSCMR2_CAN_CLK_PODF(0);
  if ( clock == CLK_80MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(0);

  if ( _CAN1 ) _CAN1->setBaudRate(currentBitrate, (( FLEXCANb_CTRL1(bus) & FLEXCAN_CTRL_LOM ) ? LISTEN_ONLY : TX));
  if ( _CAN2 ) _CAN2->setBaudRate(currentBitrate, (( FLEXCANb_CTRL1(bus) & FLEXCAN_CTRL_LOM ) ? LISTEN_ONLY : TX));
  if ( _CAN3 ) _CAN3->setBaudRate(currentBitrate, (( FLEXCANb_CTRL1(bus) & FLEXCAN_CTRL_LOM ) ? LISTEN_ONLY : TX));
}

// *****************************************************************************
uint32_t tNMEA2000_Teensyx::getClock() {
  const uint8_t clocksrc[4] = {60, 24, 80, 0};
  return clocksrc[(CCM_CSCMR2 & 0x300) >> 8];
}
#endif

// *****************************************************************************
void tNMEA2000_Teensyx::begin() {
  InitCANFrameBuffers();

#if defined(__IMXRT1062__)
  if ( !getClock() ) setClock(CLK_24MHz); /* no clock enabled, enable osc clock */

  if ( bus == CAN3 ) {
    IrqMessage = IRQ_CAN3;
    _VectorsRam[16 + IrqMessage] = flexcan_isr_can3;
    CCM_CCGR7 |= 0x3C0;
    busNumber = 3;
  }
  if ( bus == CAN2 ) {
    IrqMessage = IRQ_CAN2;
    _VectorsRam[16 + IrqMessage] = flexcan_isr_can2;
    CCM_CCGR0 |= 0x3C0000;
    busNumber = 2;
  }
  if ( bus == CAN1 ) {
    IrqMessage = IRQ_CAN1;
    _VectorsRam[16 + IrqMessage] = flexcan_isr_can1;
    CCM_CCGR0 |= 0x3C000;
    busNumber = 1;
  }
#endif

#if defined(__MK20DX256__)
  if ( bus == CAN0 ) {
    IrqMessage = IRQ_CAN_MESSAGE;
    _VectorsRam[16 + IrqMessage] = flexcan_isr_can0;
    busNumber = 0;
  }
#endif

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  if ( bus == CAN0 ) {
    IrqMessage = IRQ_CAN0_MESSAGE;
    _VectorsRam[16 + IrqMessage] = flexcan_isr_can0;
    busNumber = 0;
  }
#endif

#if defined(__MK66FX1M0__)
  else if ( bus == CAN1 ) {
    IrqMessage = IRQ_CAN1_MESSAGE;
    _VectorsRam[16 + IrqMessage] = flexcan_isr_can1;
    busNumber = 1;
  }
#endif

#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  OSC0_CR |= OSC_ERCLKEN;
  if ( bus == CAN0 ) SIM_SCGC6 |= SIM_SCGC6_FLEXCAN0;
#if defined(__MK66FX1M0__)
  else if ( bus == CAN1 ) SIM_SCGC3 |= SIM_SCGC3_FLEXCAN1;
#endif
  FLEXCANb_CTRL1(bus) &= ~FLEXCAN_CTRL_CLK_SRC;
#endif

  setTX(); setRX();

  FLEXCANb_MCR(bus) &= ~FLEXCAN_MCR_MDIS; /* enable module */
  EnterFreezeMode();
  FLEXCANb_CTRL1(bus) |= FLEXCAN_CTRL_LOM; /* listen only mode */
  FLEXCANb_MCR(bus) |= FLEXCAN_MCR_FRZ; /* enable freeze bit */
  while (FLEXCANb_MCR(bus) & FLEXCAN_MCR_LPM_ACK);
  softReset(); /* reset bus */
  while (!(FLEXCANb_MCR(bus) & FLEXCAN_MCR_FRZ_ACK));
  FLEXCANb_MCR(bus) |= FLEXCAN_MCR_SRX_DIS; /* Disable self-reception */
  FLEXCANb_MCR(bus) |= FLEXCAN_MCR_IRMQ; // individual mailbox masking
  FLEXCANb_MCR(bus) |= FLEXCAN_MCR_AEN; // TX ABORT FEATURE
  FLEXCANb_MCR(bus) |= FLEXCAN_MCR_LPRIO_EN; // TX PRIORITY FEATURE
  FLEXCANb_MCR(bus) &= ~0x8800; // disable DMA and FD (valid bits are reserved in legacy controllers)
  FLEXCANb_CTRL2(bus) |= FLEXCAN_CTRL2_RRS | // store remote frames
                                  FLEXCAN_CTRL2_EACEN | /* handles the way filtering works. Library adjusts to whether you use this or not */ 
                                  FLEXCAN_CTRL2_MRP; // mailbox > FIFO priority.
  FLEXCANb_MCR(bus) |= FLEXCAN_MCR_WRN_EN;
  FLEXCANb_MCR(bus) |= FLEXCAN_MCR_WAK_MSK;

  disableFIFO(); /* clears all data and layout to legacy mailbox mode */
  ExitFreezeMode();
  NVIC_ENABLE_IRQ(IrqMessage);
}

// *****************************************************************************
void tNMEA2000_Teensyx::enableFIFO(bool status) {
  bool frz_flag_negate = !(FLEXCANb_MCR(bus) & FLEXCAN_MCR_FRZ_ACK);
  EnterFreezeMode();
  FLEXCANb_MCR(bus) &= ~FLEXCAN_MCR_FEN; // Disable FIFO if already enabled for cleanup.
  writeIMASK(0ULL); // disable all FIFO/MB Interrupts

  for (uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(bus); i++ ) { // clear all mailboxes
    volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(bus + 0x80 + (i * 0x10)));
    mbxAddr[0] = mbxAddr[1] = mbxAddr[2] = mbxAddr[3] = 0; // code, id, word0, word1
    FLEXCANb_RXIMR(bus, i) = 0UL; // CLEAR MAILBOX MASKS (RXIMR)
  }

  FLEXCANb_RXMGMASK(bus) = FLEXCANb_RXFGMASK(bus) = 0;
  writeIFLAG(readIFLAG()); // (all bits reset when written back)

  if ( status ) {
    FLEXCANb_MCR(bus) |= FLEXCAN_MCR_FEN;
    for (uint8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(bus); i++) FLEXCANb_MBn_CS(bus,i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  } else { // FIFO disabled default setup of mailboxes, 0-7 RX, 8-15 TX
    for (uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(bus); i++ ) { // clear all mailboxes
      volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(bus + 0x80 + (i * 0x10)));
      if ( i < (FLEXCANb_MAXMB_SIZE(bus) / 2) ) {
        mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | ((i < (FLEXCANb_MAXMB_SIZE(bus) / 4)) ? 0 : FLEXCAN_MB_CS_IDE | FLEXCAN_MB_CS_SRR);
        FLEXCANb_RXIMR(bus, i) = 0UL | ((FLEXCANb_CTRL2(bus) & FLEXCAN_CTRL2_EACEN) ? (1UL << 30) : 0); // (RXIMR)
      } else {
        mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
      }
    }
  }
  if ( frz_flag_negate ) ExitFreezeMode();
}

// *****************************************************************************
void tNMEA2000_Teensyx::enableFIFOInterrupt(bool status) {
  if ( !(FLEXCANb_MCR(bus) & FLEXCAN_MCR_FEN) ) return; /* FIFO must be enabled first */
  if ( FLEXCANb_IMASK1(bus) & FLEXCAN_IMASK1_BUF5M ) return; /* FIFO interrupts already enabled */
  FLEXCANb_IMASK1(bus) &= ~0xFF; /* disable FIFO interrupt flags */
  if ( status ) FLEXCANb_IMASK1(bus) |= FLEXCAN_IMASK1_BUF5M; /* enable FIFO interrupt */
}

// *****************************************************************************
void tNMEA2000_Teensyx::enableMBInterrupts(bool status) {
  EnterFreezeMode();
  for ( uint8_t mb_num = mailboxOffset(); mb_num < FLEXCANb_MAXMB_SIZE(bus); mb_num++ ) {
    enableMBInterrupt(mb_num, status);
  }
  ExitFreezeMode();
}

// *****************************************************************************
void tNMEA2000_Teensyx::enableMBInterrupt(uint8_t mb_num, bool status) {
  if ( mb_num < mailboxOffset() ) return; /* mailbox not available */
  if ( status ) writeIMASKBit(mb_num); /* enable mailbox interrupt */
  else writeIMASKBit(mb_num, 0); /* disable mailbox interrupt */
}

// *****************************************************************************
bool tNMEA2000_Teensyx::setMB(uint8_t mb_num, const FLEXCAN_RXTX &mb_rx_tx, const FLEXCAN_IDE &ide) {
  if ( mb_num < mailboxOffset() || mb_num>=getNumMailBoxes() ) return 0; // mailbox not available
  firstTxBox=INVALID_MAILBOX;
  writeIMASKBit(mb_num, 0); // immediately disable mailbox interrupt
  FLEXCAN_get_code(FLEXCANb_MBn_CS(bus, mb_num)); // Reading Control Status atomically locks mailbox (if it is RX mode).
  FLEXCANb_MBn_ID(bus, mb_num) = 0UL;
  FLEXCANb_MBn_WORD0(bus, mb_num) = 0UL;
  FLEXCANb_MBn_WORD1(bus, mb_num) = 0UL;
  if ( mb_rx_tx == RX ) {
    if ( ide != EXT ) FLEXCANb_MBn_CS(bus, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
    else FLEXCANb_MBn_CS(bus, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
  }
  if ( mb_rx_tx == TX ) {
    FLEXCANb_MBn_CS(bus, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  }
  if ( ide == INACTIVE ) {
    FLEXCANb_MBn_CS(bus, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_INACTIVE);
  }
  (void)FLEXCANb_TIMER(bus);
  writeIFLAGBit(mb_num); // clear mailbox reception flag 
  return 1;
}

// *****************************************************************************
uint64_t tNMEA2000_Teensyx::readIFLAG() {
#if defined(__IMXRT1062__)
  return (((uint64_t)FLEXCANb_IFLAG2(bus) << 32) | FLEXCANb_IFLAG1(bus));
#else
  return FLEXCANb_IFLAG1(bus);
#endif
}

// *****************************************************************************
void tNMEA2000_Teensyx::writeIFLAG(uint64_t value) {
#if defined(__IMXRT1062__)
  FLEXCANb_IFLAG2(bus) = value >> 32;
#endif
  FLEXCANb_IFLAG1(bus) = value;
}

// *****************************************************************************
void tNMEA2000_Teensyx::writeIFLAGBit(uint8_t mb_num) {
  if ( mb_num < 32 ) {
    FLEXCANb_IFLAG1(bus) |= (1UL << mb_num);
  } else {
    FLEXCANb_IFLAG2(bus) |= (1UL << (mb_num - 32));
  }
}

// *****************************************************************************
void tNMEA2000_Teensyx::writeIMASK(uint64_t value) {
#if defined(__IMXRT1062__)
  FLEXCANb_IMASK2(bus) = value >> 32;
#endif
  FLEXCANb_IMASK1(bus) = value;
}

// *****************************************************************************
uint64_t tNMEA2000_Teensyx::readIMASK() {
#if defined(__IMXRT1062__)
  return (((uint64_t)FLEXCANb_IMASK2(bus) << 32) | FLEXCANb_IMASK1(bus));
#else
  return FLEXCANb_IMASK1(bus);
#endif
}

// *****************************************************************************
void tNMEA2000_Teensyx::writeIMASKBit(uint8_t mb_num, bool set) {
  if ( mb_num < 32 ) {
    (( set ) ? FLEXCANb_IMASK1(bus) |= (1UL << mb_num) : FLEXCANb_IMASK1(bus) &= ~(1UL << mb_num));
  } else {
    (( set ) ? FLEXCANb_IMASK2(bus) |= (1UL << (mb_num - 32)) : FLEXCANb_IMASK2(bus) &= ~(1UL << (mb_num - 32)));
  }
}

// *****************************************************************************
void tNMEA2000_Teensyx::writeTxMailbox(uint8_t mb_num, unsigned long id, unsigned char len, const unsigned char *buf, bool extended) {
  writeIFLAGBit(mb_num);
  uint32_t code = 0;
  volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(bus + 0x80 + (mb_num * 0x10)));
//  Commented below by TTL. That caused lock time to time to FLEXCAN_MB_CODE_TX_ONCE state.
//  mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  mbxAddr[1] = id & FLEXCAN_MB_ID_EXT_MASK;
  if ( extended ) code |= (3UL << 21);
  for ( uint8_t i = 0; i < (8 >> 2); i++ ) {
    mbxAddr[2 + i] = (buf[0 + i * 4] << 24) | 
                     (buf[1 + i * 4] << 16) | 
                     (buf[2 + i * 4] << 8) | 
                     buf[3 + i * 4];
  }
  code |= len << 16;
  mbxAddr[0] = code | FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE);
}
/*
// *****************************************************************************
void tNMEA2000_Teensyx::writeTxMailbox(uint8_t mb_num, const CAN_message_t &msg) {
  writeIFLAGBit(mb_num);
  uint32_t code = 0;
  volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(bus + 0x80 + (mb_num * 0x10)));
//  Commented below by TTL. That caused lock time to time to FLEXCAN_MB_CODE_TX_ONCE state.
//  mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  mbxAddr[1] = (( msg.flags.extended ) ? ( msg.id & FLEXCAN_MB_ID_EXT_MASK ) : FLEXCAN_MB_ID_IDSTD(msg.id));
  if ( msg.flags.remote ) code |= (1UL << 20);
  if ( msg.flags.extended ) code |= (3UL << 21);
  for ( uint8_t i = 0; i < (8 >> 2); i++ ) mbxAddr[2 + i] = (msg.buf[0 + i * 4] << 24) | (msg.buf[1 + i * 4] << 16) | (msg.buf[2 + i * 4] << 8) | msg.buf[3 + i * 4];
  code |= msg.len << 16;
  mbxAddr[0] = code | FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE);

  if ( msg.flags.remote ) {
    uint32_t timeout = millis();
    while ( !(readIFLAG() & (1ULL << mb_num)) && (millis() - timeout < 20) );
    writeIFLAGBit(mb_num);
    mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  }
}
*/
// *****************************************************************************
uint8_t tNMEA2000_Teensyx::mailboxOffset() {
  if ( !(FLEXCANb_MCR(bus) & FLEXCAN_MCR_FEN ) ) return 0; /* return offset 0 since FIFO is disabled */
  uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(bus) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
  if ( FLEXCANb_MAXMB_SIZE(bus) < (6 + ((((FLEXCANb_CTRL2(bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
  return (FLEXCANb_MAXMB_SIZE(bus) - remaining_mailboxes); /* otherwise return offset MB position after FIFO area */
}

#if defined(__IMXRT1062__)
#define MAX_DEV_MAILBOXES 64
#else
#define MAX_DEV_MAILBOXES 16
#endif

// *****************************************************************************
void tNMEA2000_Teensyx::setMaxMB(uint8_t last) {
  last = constrain(last,1,MAX_DEV_MAILBOXES);
  last--;
  EnterFreezeMode();
  
  bool fifo_was_cleared = FLEXCANb_MCR(bus) & FLEXCAN_MCR_FEN;
  disableFIFO();
  writeIFLAG(readIFLAG()); // (all bits reset when written back) (needed for MAXMB changes)
  FLEXCANb_MCR(bus) &= ~0x7F; // clear current value
  FLEXCANb_MCR(bus) |= last; // set mailbox max
  if ( fifo_was_cleared ) enableFIFO();

  ExitFreezeMode();
}

// *****************************************************************************
void tNMEA2000_Teensyx::softReset() {
  FLEXCANb_MCR(bus) |= FLEXCAN_MCR_SOFT_RST;
  while (FLEXCANb_MCR(bus) & FLEXCAN_MCR_SOFT_RST);
}

// *****************************************************************************
void tNMEA2000_Teensyx::ExitFreezeMode() {
  FLEXCANb_MCR(bus) &= ~FLEXCAN_MCR_HALT;
  while (FLEXCANb_MCR(bus) & FLEXCAN_MCR_FRZ_ACK);
}

// *****************************************************************************
void tNMEA2000_Teensyx::EnterFreezeMode() {
  FLEXCANb_MCR(bus) |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT;
  while (!(FLEXCANb_MCR(bus) & FLEXCAN_MCR_FRZ_ACK));
}

// *****************************************************************************
void tNMEA2000_Teensyx::setBaudRate(uint32_t baud, FLEXCAN_RXTX listen_only) {
  currentBitrate = baud;

#if defined(__IMXRT1062__)
  uint32_t clockFreq = getClock() * 1000000;
#else
  uint32_t clockFreq = 16000000;
#endif

  uint32_t divisor = 0, bestDivisor = 0, result = clockFreq / baud / (divisor + 1);
  int error = baud - (clockFreq / (result * (divisor + 1))), bestError = error;

  bool frz_flag_negate = !(FLEXCANb_MCR(bus) & FLEXCAN_MCR_FRZ_ACK);
  EnterFreezeMode();

  while (result > 5) {
    divisor++;
    result = clockFreq / baud / (divisor + 1);
    if (result <= 25) {
      error = baud - (clockFreq / (result * (divisor + 1)));
      if (error < 0) error *= -1;
      if (error < bestError) {
        bestError = error;
        bestDivisor = divisor;
      }
      if ((error == bestError) && (result > 11) && (result < 19)) {
        bestError = error;
        bestDivisor = divisor;
      }
    }
  }

  divisor = bestDivisor;
  result = clockFreq / baud / (divisor + 1);

  if ((result < 5) || (result > 25) || (bestError > 300)) {
    if ( frz_flag_negate ) ExitFreezeMode();
    return;
  }

  result -= 5; // the bitTimingTable is offset by 5 since there was no reason to store bit timings for invalid numbers
  uint8_t bitTimingTable[21][3] = {
    {0, 0, 1}, //5
    {1, 0, 1}, //6
    {1, 1, 1}, //7
    {2, 1, 1}, //8
    {2, 2, 1}, //9
    {2, 3, 1}, //10
    {2, 3, 2}, //11
    {2, 4, 2}, //12
    {2, 5, 2}, //13
    {2, 5, 3}, //14
    {2, 6, 3}, //15
    {2, 7, 3}, //16
    {2, 7, 4}, //17
    {3, 7, 4}, //18
    {3, 7, 5}, //19
    {4, 7, 5}, //20
    {4, 7, 6}, //21
    {5, 7, 6}, //22
    {6, 7, 6}, //23
    {6, 7, 7}, //24
    {7, 7, 7}, //25
  }, propSeg = bitTimingTable[result][0], pSeg1 = bitTimingTable[result][1], pSeg2 = bitTimingTable[result][2];
  FLEXCANb_CTRL1(bus) = (FLEXCAN_CTRL_PROPSEG(propSeg) | FLEXCAN_CTRL_RJW(1) | FLEXCAN_CTRL_PSEG1(pSeg1) |
                    FLEXCAN_CTRL_PSEG2(pSeg2) | FLEXCAN_CTRL_ERR_MSK | FLEXCAN_CTRL_PRESDIV(divisor));
  ( listen_only != LISTEN_ONLY ) ? FLEXCANb_CTRL1(bus) &= ~FLEXCAN_CTRL_LOM : FLEXCANb_CTRL1(bus) |= FLEXCAN_CTRL_LOM; /* listen-only mode */
  if ( frz_flag_negate ) ExitFreezeMode();
}

// *****************************************************************************
void tNMEA2000_Teensyx::setMRP(bool mrp) { /* mailbox priority (1) or FIFO priority (0) */
  EnterFreezeMode();
  if ( mrp ) {
    FLEXCANb_CTRL2(bus) |= FLEXCAN_CTRL2_MRP;
  } else {
    FLEXCANb_CTRL2(bus) &= ~FLEXCAN_CTRL2_MRP;
  }
  ExitFreezeMode();
}

// *****************************************************************************
void tNMEA2000_Teensyx::setRRS(bool rrs) { /* store remote frames */
  EnterFreezeMode();
  if ( rrs ) {
    FLEXCANb_CTRL2(bus) |= FLEXCAN_CTRL2_RRS;
  } else {
    FLEXCANb_CTRL2(bus) &= ~FLEXCAN_CTRL2_RRS;
  }
  ExitFreezeMode();
}

// *****************************************************************************
void tNMEA2000_Teensyx::setTX(tPins pin) {
#if defined(__IMXRT1062__)
  if ( bus == CAN3 ) {
    if ( pin == pinDefault ) {
      IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_36 = 0x19; // pin31 T3B2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_36 = 0x10B0; // pin31 T3B2
    }
  }
  if ( bus == CAN2 ) {
    if ( pin == pinDefault ) {
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 = 0x10; // pin 1 T4B1+B2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_02 = 0x10B0; // pin 1 T4B1+B2
    }
  }
  if ( bus == CAN1 ) {
    if ( pin == pinDefault ) {
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08 = 0x12; // pin 22 T4B1+B2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_08 = 0x10B0; // pin 22 T4B1+B2
    }
    if ( pin == pinAlternate ) {
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x12; // pin 11 T4B1+B2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02= 0x10B0; // pin 11 T4B1+B2
    }
  }
#endif

#if defined(__MK20DX256__)
  CORE_PIN3_CONFIG = PORT_PCR_MUX(2);
#endif
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  if ( bus == CAN0 ) {
    if ( pin == pinAlternate ) {
      CORE_PIN3_CONFIG = 0; CORE_PIN29_CONFIG = PORT_PCR_MUX(2);
    }
    else if ( pin == pinDefault ) {
      CORE_PIN29_CONFIG = 0; CORE_PIN3_CONFIG = PORT_PCR_MUX(2);
    }
  } /* Alternative CAN1 pins are not broken out on Teensy 3.6 */
#endif
#if defined(__MK66FX1M0__)
  if ( bus == CAN1 ) {
    CORE_PIN33_CONFIG = PORT_PCR_MUX(2);
  }
#endif

}

// *****************************************************************************
void tNMEA2000_Teensyx::setRX(tPins pin) {
#if defined(__IMXRT1062__)
  /* DAISY REGISTER CAN3
    00 GPIO_EMC_37_ALT9 â€” Selecting Pad: GPIO_EMC_37 for Mode: ALT9
    01 GPIO_AD_B0_15_ALT8 â€” Selecting Pad: GPIO_AD_B0_15 for Mode: ALT8
    10 GPIO_AD_B0_11_ALT8 â€” Selecting Pad: GPIO_AD_B0_11 for Mode: ALT8
  */
  /* DAISY REGISTER CAN2
    00 GPIO_EMC_10_ALT3 â€” Selecting Pad: GPIO_EMC_10 for Mode: ALT3
    01 GPIO_AD_B0_03_ALT0 â€” Selecting Pad: GPIO_AD_B0_03 for Mode: ALT0
    10 GPIO_AD_B0_15_ALT6 â€” Selecting Pad: GPIO_AD_B0_15 for Mode: ALT6
    11 GPIO_B1_09_ALT6 â€” Selecting Pad: GPIO_B1_09 for Mode: ALT6
  */
  /* DAISY REGISTER CAN1
    00 GPIO_SD_B1_03_ALT4 â€” Selecting Pad: GPIO_SD_B1_03 for Mode: ALT4
    01 GPIO_EMC_18_ALT3 â€” Selecting Pad: GPIO_EMC_18 for Mode: ALT3
    10 GPIO_AD_B1_09_ALT2 â€” Selecting Pad: GPIO_AD_B1_09 for Mode: ALT2
    11 GPIO_B0_03_ALT2 â€” Selecting Pad: GPIO_B0_03 for Mode: ALT2
  */
  if ( bus == CAN3 ) {
    if ( pin == pinDefault ) {
      IOMUXC_CANFD_IPP_IND_CANRX_SELECT_INPUT = 0x00;
      IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_37 = 0x19; // pin30 T3B2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_37 = 0x10B0; // pin30 T3B2
    }
  }
  if ( bus == CAN2 ) {
    if ( pin == pinDefault ) {
      IOMUXC_FLEXCAN2_RX_SELECT_INPUT = 0x01;
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 0x10; // pin 0 T4B1+B2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_03 = 0x10B0; // pin 0 T4B1+B2
    }
  }
  if ( bus == CAN1 ) {
    if ( pin == pinDefault ) {
      IOMUXC_FLEXCAN1_RX_SELECT_INPUT = 0x02;
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09 = 0x12; // pin 23 T4B1+B2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_09 = 0x10B0; // pin 23 T4B1+B2
    }
    if ( pin == pinAlternate ) {
      IOMUXC_FLEXCAN1_RX_SELECT_INPUT = 0x03;
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x12; // pin 13 T4B1+B2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = 0x10B0; // pin 13 T4B1+B2
    }
  }
#endif

#if defined(__MK20DX256__)
  CORE_PIN4_CONFIG = PORT_PCR_MUX(2);
#endif
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  if ( bus == CAN0 ) {
    if ( pin == pinAlternate ) {
      CORE_PIN4_CONFIG = 0; CORE_PIN30_CONFIG = PORT_PCR_MUX(2);
    }
    else if ( pin == pinDefault ) {
      CORE_PIN30_CONFIG = 0; CORE_PIN4_CONFIG = PORT_PCR_MUX(2);
    }
  } /* Alternative CAN1 pins are not broken out on Teensy 3.6 */
#endif
#if defined(__MK66FX1M0__)
  if ( bus == CAN1 ) {
    CORE_PIN34_CONFIG = PORT_PCR_MUX(2);
  }
#endif

}

// *****************************************************************************
uint8_t tNMEA2000_Teensyx::getFirstTxBox() {
  if ( firstTxBox!=INVALID_MAILBOX ) return firstTxBox;

  for (uint8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(bus); i++) {
    if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(bus, i)) >> 3) ) {
      firstTxBox=i;
      return i; // if TX
    }
  }
  return INVALID_MAILBOX;
}
/*
// *****************************************************************************
bool tNMEA2000_Teensyx::write(const CAN_message_t &msg) {
  uint8_t prio = (uint8_t)((msg.id >> 26) & 0x7);
  bool ret=true;

  irqLock();

  uint8_t mb=getFirstTxBox()+prio;

  if ( FLEXCAN_get_code(FLEXCANb_MBn_CS(bus, mb)) == FLEXCAN_MB_CODE_TX_INACTIVE ) {
    writeTxMailbox(mb, msg);
    ret=true; // transmit entry accepted 
  } else { // Busy, buffer it
    ret=txRing->add(msg,prio);
  }

  irqRelease();

  return ret; // transmit entry failed, no mailboxes or queues available 
}
*/
#if defined(__IMXRT1062__)
// *****************************************************************************
void tNMEA2000_Teensyx::flexcan_isr_can1() {
  if ( _CAN1 ) _CAN1->flexcan_interrupt();
}

// *****************************************************************************
void tNMEA2000_Teensyx::flexcan_isr_can2() {
  if ( _CAN2 ) _CAN2->flexcan_interrupt();
}

// *****************************************************************************
void tNMEA2000_Teensyx::flexcan_isr_can3() {
  if ( _CAN3 ) _CAN3->flexcan_interrupt();
}
#endif
#if defined(__MK20DX256__) || defined(__MK64FX512__)
// *****************************************************************************
void tNMEA2000_Teensyx::flexcan_isr_can0() {
  if ( _CAN0 ) _CAN0->flexcan_interrupt();
}
#endif
#if defined(__MK66FX1M0__)
// *****************************************************************************
void tNMEA2000_Teensyx::flexcan_isr_can0() {
  if ( _CAN0 ) _CAN0->flexcan_interrupt();
}
// *****************************************************************************
void tNMEA2000_Teensyx::flexcan_isr_can1() {
  if ( _CAN1 ) _CAN1->flexcan_interrupt();
}
#endif

// *****************************************************************************
bool tNMEA2000_Teensyx::sendFromTxRing(uint8_t prio) {
  const CAN_message_t *txMsg;
  uint8_t mb=getFirstTxBox()+prio;

  txMsg=txRing->getReadRef(prio);
  if ( txMsg!=0 ) {
    #if defined(TEENSYX_DEBUG)
      unsigned long pgn;
      unsigned char src;
      unsigned char dst;
      CanIdToN2k(txMsg->id,prio,pgn,src,dst);
      TeensyxDbgf("    pgn:%6lu, prio:%u, src:%u, dst:%u, data:%02X\n",pgn,prio,src,dst,txMsg->buf[0]);
    #endif
    writeTxMailbox(mb,txMsg->id,txMsg->len,txMsg->buf);
    return true;
  }

  return false;
}

// *****************************************************************************
void tNMEA2000_Teensyx::flexcan_interrupt() {
  CAN_message_t *rxMsg;
  bool extended;
  uint32_t id;
  uint8_t prio;
  uint64_t imask = readIMASK(), iflag = readIFLAG();

  if ( !(FLEXCANb_MCR(bus) & (1UL << 15)) ) { /* if DMA is disabled, ONLY THEN you can handle FIFO in ISR */
    if ( (FLEXCANb_MCR(bus) & FLEXCAN_MCR_FEN) && (imask & FLEXCAN_IMASK1_BUF5M) && (iflag & FLEXCAN_IFLAG1_BUF5I) ) { /* FIFO is enabled, capture frames if triggered */
      volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(bus + 0x80 + (0 * 0x10)));
      uint32_t code = mbxAddr[0];

      extended=(bool)(code & (1UL << 21));
      id=(mbxAddr[1] & 0x1FFFFFFF) >> (extended ? 0 : 18);
      prio=(uint8_t)((id >> 26) & 0x7);
      rxMsg=rxRing->getAddRef(prio);
      if ( rxMsg!=0 ) {
        rxMsg->len = (code & 0xF0000) >> 16;
        rxMsg->flags.remote = (bool)(code & (1UL << 20));
        rxMsg->flags.extended = extended;
//        rxMsg->timestamp = code & 0xFFFF;
        rxMsg->id = id;
//        rxMsg->idhit = code >> 23;
        for ( uint8_t i = 0; i < (8 >> 2); i++ ) {
          for ( int8_t d = 0; d < 4 ; d++ ) {
            rxMsg->buf[(4 * i) + 3 - d] = (uint8_t)(mbxAddr[2 + i] >> (8 * d));
          }
        }
//        rxMsg->bus = busNumber;
//        rxMsg->mb = 0xff; // store the mailbox the message came from (for callback reference) 
      }
      (void)FLEXCANb_TIMER(bus);
      writeIFLAGBit(5); /* clear FIFO bit only! */
      if ( iflag & FLEXCAN_IFLAG1_BUF6I ) writeIFLAGBit(6); /* clear FIFO bit only! */
      if ( iflag & FLEXCAN_IFLAG1_BUF7I ) writeIFLAGBit(7); /* clear FIFO bit only! */
    }
  }

  uint8_t exit_point = 64 - __builtin_clzll(iflag | 1); /* break from MSB's if unset, add 1 to prevent undefined behaviour in clz for 0 check */
  uint8_t lastMB=getNumMailBoxes();
  for ( uint8_t mb_num = mailboxOffset(); mb_num < lastMB; mb_num++ ) {
    if ( mb_num >= exit_point ) break; /* early exit from higher unflagged mailboxes */
    if (!(imask & (1ULL << mb_num))) continue; /* don't read non-interrupt mailboxes */
    if (!(iflag & (1ULL << mb_num))) continue; /* don't read unflagged mailboxes */

    volatile uint32_t *mbxAddr = &(*(volatile uint32_t*)(bus + 0x80 + (mb_num * 0x10)));
    uint32_t code = mbxAddr[0];
    uint32_t reason=FLEXCAN_get_code(code);
    bool overrun=false;

    switch ( reason ) {
      case FLEXCAN_MB_CODE_RX_OVERRUN:
        overrun=true;
      case FLEXCAN_MB_CODE_RX_FULL:
        extended=(bool)(code & (1UL << 21));
        id=(mbxAddr[1] & 0x1FFFFFFF) >> (extended ? 0 : 18);
        prio=(uint8_t)((id >> 26) & 0x7);
        rxMsg=rxRing->getAddRef(prio);
        if ( rxMsg!=0 ) {
          rxMsg->flags.extended = extended;
          rxMsg->id = id;
          rxMsg->len = (code & 0xF0000) >> 16;
//          rxMsg->mb = mb_num;
//          rxMsg->timestamp = code & 0xFFFF;
//          rxMsg->bus = busNumber;
          rxMsg->flags.overrun = overrun;
          for ( uint8_t i = 0; i < (8 >> 2); i++ ) {
            for ( int8_t d = 0; d < 4 ; d++ ) {
              rxMsg->buf[(4 * i) + 3 - d] = (uint8_t)(mbxAddr[2 + i] >> (8 * d));
            }
          }
        }
        mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | ((extended) ? (FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE) : 0);
        (void)FLEXCANb_TIMER(bus);
        writeIFLAGBit(mb_num);
        break;
      case FLEXCAN_MB_CODE_TX_INACTIVE:
        prio=mb_num-getFirstTxBox();
        TeensyxDbgf("%6lu - Tx inactive mb:%u, prio:%u\n",millis(),mb_num,prio);
        if ( !sendFromTxRing(prio) ) { 
          writeIFLAGBit(mb_num); // clear IFLAG
        }
        break;
      default:;
    }
  }

  FLEXCANb_ESR1(bus) |= FLEXCANb_ESR1(bus);
}
