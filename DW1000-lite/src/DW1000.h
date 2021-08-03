/* 
 *  File: DW1000.h
 *  Date: 2021-07-31
 *  Original Author: https://github.com/Richardn2002
 *  Separation in .h/.cpp and mods for ESP32: matdru
 *  For some documentation, see DS-TWR-Initiator-Master.cpp and 
 *  original project at https://github.com/Richardn2002/arduino-dw1000-lite
 */

#ifndef DW1000_H
#define DW1000_H

#include <Arduino.h>
#include <SPI.h>

// matdru: Antenna delay - this is best determined by testing range measurements
#define TX_ANT_DLY  (16445)
#define TX_ANT_DLY_MSB  ((TX_ANT_DLY>>8) & 0xFF)
#define TX_ANT_DLY_LSB  (TX_ANT_DLY & 0xFF)

// matdru: Distance calculation params
#define DWT_TIME_UNITS          (1.0/499.2e6/128.0) //!< = 15.65e-12 s
#define SPEED_OF_LIGHT          299702547
#define DISTANCE_PER_DWT_TIME_UNIT ((double)DWT_TIME_UNITS * (double)SPEED_OF_LIGHT)
// matdru

void _SPITxn(int CSN, int headerLen, byte* header, int dataLen, const byte* data, int respLen, byte resp[]);

byte* _header(bool writes, byte address);
byte* _headerWithIndex(bool writes, byte address, int index);
byte* _headerWithExtendedIndex(bool writes, byte address, int index);

void dwReadReg(int CSN, byte address, int index, int respLen, byte resp[]);
void dwWriteReg(int CSN, byte address, int index, int dataLen, const byte* data);

void dwReadOTP(int CSN, uint16_t address, int respLen, byte resp[]);

void dwEnableClock(int CSN, byte clock);
void dwReset(int RST);
void dwInit(int CSN, int RST);

void dwSetPAN_IDAndSHORT_ADDR(int CSN, uint16_t PAN_ID, uint16_t SHORT_ADDR) ;

void dwSetDelayedTime(int CSN, uint64_t delayedTime);

void dwStartReceive(int CSN, bool delayed);
bool dwIsReceiveTimestampAvailable(int CSN);

bool dwIsReceiveDone(int CSN);
bool dwIsReceiveError(int CSN);

uint64_t dwGetTimestamp(int CSN);
uint64_t dwGetReceiveTimestamp(int CSN);

void dwGetReceiveData(int CSN, int dataLen, byte* data);
void dwClearReceiveStatus(int CSN);
void dwResetReceiver(int CSN);

void dwStartAccMemRead(int CSN);
void dwReadAccMem(int CSN, int index, int dataLen, byte* data);
void dwEndAccMemRead(int CSN);

void dwSetTransmitData(int CSN, int dataLen, const byte* data);
void dwStartTransmit(int CSN, bool delayed, bool wait4resp);
bool dwIsTransmitDone(int CSN);
void dwClearTransmitStatus(int CSN);
void dwForceTRxOff(int CSN);

void dwGetPrintableDeviceIdentifier(int CSN, char msgBuffer[]);


// Registers

constexpr byte DEV_ID =     0x00;
constexpr byte EUI =        0x01;
constexpr byte PANADR =     0x03;
constexpr byte SYS_CFG =    0x04;
constexpr byte SYS_TIME =   0x06;
constexpr byte TX_FCTRL =   0x08;
constexpr byte TX_BUFFER =  0x09;
constexpr byte DX_TIME =    0x0A;
constexpr byte RX_WFTO =    0x0C;
constexpr byte SYS_CTRL =   0x0D;
constexpr byte SYS_MASK =   0x0E;
constexpr byte SYS_STATUS = 0x0F;
constexpr byte RX_FINFO =   0x10;
constexpr byte RX_BUFFER =  0x11;
constexpr byte RX_FQUAL =   0x12;
constexpr byte RX_TIME =    0x15;
constexpr byte TX_TIME =    0x17;
constexpr byte EC_CTRL =    0x24;
constexpr byte FS_CTRL =    0x2B;
constexpr byte OTP_IF =     0x2D;

constexpr int FS_XTALT_INDEX = 0x0E;

#endif