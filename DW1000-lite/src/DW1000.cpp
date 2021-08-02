/* 
 *  File: DW1000.cpp
 *  Date: 2021-07-31
 *  Original Author: https://github.com/Richardn2002
 *  Separation in .h/.cpp and mods for ESP32: matdru
 *  For documentation, see DS-TWR-Initiator-Master.cpp and 
 *  original project at https://github.com/Richardn2002/arduino-dw1000-lite
 */

#include <DW1000.h>

const SPISettings SPI_FAST = SPISettings(16000000, MSBFIRST, SPI_MODE0);
const SPISettings SPI_SLOW = SPISettings(2000000, MSBFIRST, SPI_MODE0);
SPISettings _SPISettings;

void _SPITxn(int CSN, int headerLen, byte* header, int dataLen, const byte* data, int respLen, byte resp[]) {
    SPI.beginTransaction(_SPISettings);
    digitalWrite(CSN, LOW);
    for (int i = 0; i < headerLen; i++) {
        SPI.transfer(header[i]);
    }
    for (int i = 0; i < dataLen; i++) {
        SPI.transfer(data[i]);
    }
    for (int i = 0; i < respLen; i++) {
        resp[i] = SPI.transfer(0x00);
    }
    delayMicroseconds(5);
    digitalWrite(CSN, HIGH);
    SPI.endTransaction();
}

byte* _header(bool writes, byte address) {
    static byte header[1];
    header[0] = (writes << 7) + address;

    return header;
}

byte* _headerWithIndex(bool writes, byte address, int index) {
    static byte header[2];
    header[0] = (writes << 7) + 0x40 + address;
    header[1] = index & 0x7F;

    return header;
}

byte* _headerWithExtendedIndex(bool writes, byte address, int index) {
    static byte header[3];
    header[0] = (writes << 7) + 0x40 + address;
    header[1] = 0x80 + (index & 0x7F);
    header[2] = (index >> 7) & 0xFF;

    return header;
}

void dwReadReg(int CSN, byte address, int index, int respLen, byte resp[]) {
    if (!index) {
        _SPITxn(CSN, 1, _header(false, address), 0, NULL, respLen, resp);
    } else if (index < 0x7F) {
        _SPITxn(CSN, 2, _headerWithIndex(false, address, index), 0, NULL, respLen, resp);
    } else {
        _SPITxn(CSN, 3, _headerWithExtendedIndex(false, address, index), 0, NULL, respLen, resp);
    }
}

void dwWriteReg(int CSN, byte address, int index, int dataLen, const byte* data) {
    if (!index) {
        _SPITxn(CSN, 1, _header(true, address), dataLen, data, 0, NULL);
    } else if (index < 0x7F) {
        _SPITxn(CSN, 2, _headerWithIndex(true, address, index), dataLen, data, 0, NULL);
    } else {
        _SPITxn(CSN, 3, _headerWithExtendedIndex(true, address, index), dataLen, data, 0, NULL);
    }
}

void dwReadOTP(int CSN, uint16_t address, int respLen, byte resp[]) {
    byte OTPAddr[2];
    OTPAddr[0] = address & 0xFF;
    OTPAddr[1] = (address >> 8) & 0xFF;
    dwWriteReg(CSN, OTP_IF, 0x04, 2, OTPAddr);
    dwWriteReg(CSN, OTP_IF, 0x06, 1, (const byte[]){0x03});
    dwWriteReg(CSN, OTP_IF, 0x06, 1, (const byte[]){0x01}); //Note: conflicts with the user manual example.
    dwReadReg(CSN, OTP_IF, 0x0A, respLen, resp);
    dwWriteReg(CSN, OTP_IF, 0x06, 1, (const byte[]){0x00});
}

constexpr byte PMSC = 0x36;
constexpr byte SYS_AUTO_CLOCK = 0x00;
constexpr byte SYS_XTI_CLOCK  = 0x01;
constexpr byte SYS_PLL_CLOCK  = 0x02;

void dwEnableClock(int CSN, byte clock) {
    byte pmscCtrl0[4];
    dwReadReg(CSN, PMSC, 0, 2, pmscCtrl0);
    if(clock == SYS_AUTO_CLOCK) {
        pmscCtrl0[0] &= 0xFC;
        pmscCtrl0[0] = SYS_AUTO_CLOCK;
	} else if(clock == SYS_XTI_CLOCK) {
		pmscCtrl0[0] &= 0xFC;
		pmscCtrl0[0] |= 0x01;
	} else if(clock == SYS_PLL_CLOCK) {
		pmscCtrl0[0] &= 0xFC;
		pmscCtrl0[0] |= 0x02;
	}
	dwWriteReg(CSN, PMSC, 0, 2, pmscCtrl0);
    delay(5);
}

void dwReset(int RST) { //This is an Arduino-specific way of reseting. Likely to change.

/*
    pinMode(RST, OUTPUT);
    digitalWrite(RST, LOW);
    delay(2);
    pinMode(RST, INPUT);
    delay(5);
*/
    // matdru: ESP32 has OPEN_DRAIN
    digitalWrite(RST, LOW);
    pinMode(RST, OUTPUT_OPEN_DRAIN);
    delay(2);
    digitalWrite(RST, HIGH);
    delay(5);
    // matdru
}



void dwInit(int CSN, int RST) {
    delay(5);
    pinMode(CSN, OUTPUT);
    digitalWrite(CSN, HIGH);
    dwReset(RST);

    _SPISettings = SPI_SLOW;

    dwEnableClock(CSN, SYS_XTI_CLOCK);

    //CPLL lock detect. Can rescue clock PLL losing lock warning.
    dwWriteReg(CSN, EC_CTRL, 0, 1, (const byte[1]){0x04});

    //Read clock calibration setting. If not present, use midrange setting.
    byte fsXtalt[1];
    byte otpBuf[1];
    dwReadOTP(CSN, 0x001E, 1, otpBuf);
    if (!otpBuf[0]) {
        fsXtalt[0] = (otpBuf[0] & 0x1F) | 0x60;
	} else {
        fsXtalt[0] = (0x10 & 0x1F) | 0x60;
	}
    dwWriteReg(CSN, FS_CTRL, FS_XTALT_INDEX, 1, fsXtalt);

    //LDOTUNE

    //Load LDE microcode.
    dwWriteReg(CSN, PMSC, 0, 2, (const byte[]){0x01, 0x03}); //Switch to LDE_CLOCK.
    delay(5);
    dwWriteReg(CSN, OTP_IF, 0x06, 2, (const byte[]){0x00, 0x80}); //Load microcode.
    delay(1);
    dwWriteReg(CSN, PMSC, 0, 2, (const byte[]){0x00, 0x02}); //Switch to SYS_AUTO_CLOCK.
    delay(5);

    /*
    // Collect needed information for voltage and temperature measurement
    dwReadOTP(CSN, 0x0008, otpBuf); // Steals the definition above.
    _vmeas = otpBuf[0]; // Voltage = ((SAR_LVBAT – Vmeas) / 173) + 3.3
    dwReadOTP(CSN, 0x0009, otpBuf);
    _tmeas = otpBuf[0]; // Temperature(°C ) = ((SAR_LTEMP – Tmeas) x 1.14) + 23
    */

    dwEnableClock(CSN, SYS_AUTO_CLOCK);

    _SPISettings = SPI_FAST;

    //Initialization for deepsleep function. (AON:CFG1(0x2C:0x0A))

    //Tune according to default config: 6.8Mbps, default 16MHz, 128, PAC 8, code 4, channel 5, los optimization
    dwWriteReg(CSN, 0x23, 0x04, 2, (const byte[]){0x70, 0x88}); //AGC_TUNE1 for default 16MHz PRF
    dwWriteReg(CSN, 0x23, 0x0C, 4, (const byte[]){0x07, 0xA9, 0x02, 0x25}); //AGC_TUNE2
    dwWriteReg(CSN, 0x23, 0x12, 2, (const byte[]){0x35, 0x00}); //AGC_TUNE3
    dwWriteReg(CSN, 0x27, 0x02, 2, (const byte[]){0x01, 0x00}); //DRX_TUNE0b for standard SFD, 6.8Mbps
    dwWriteReg(CSN, 0x27, 0x04, 2, (const byte[]){0x87, 0x00}); //DRX_TUNE1a for 16Mhz PRF, 6.8Mbps
    dwWriteReg(CSN, 0x27, 0x06, 2, (const byte[]){0x20, 0x00}); //DRX_TUNE1b for preamble length 128, 6.8Mbps
    dwWriteReg(CSN, 0x27, 0x08, 4, (const byte[]){0x2D, 0x00, 0x1A, 0x31}); //DRX_TUNE2 for 16Mhz PRF, PAC size 8
    dwWriteReg(CSN, 0x27, 0x26, 2, (const byte[]){0x28, 0x00}); //DRX_TUNE4H for preamble length 128
    dwWriteReg(CSN, 0x2E, 0x0806, 1, (const byte[]){0x0D}); //LDE_CFG for better line-of-sight performance
    dwWriteReg(CSN, 0x2E, 0x1806, 2, (const byte[]){0x07, 0x16}); //LDE_CFG2 for better line-of-sight, 16MHz PRF
    dwWriteReg(CSN, 0x2E, 0x2804, 2, (const byte[]){0x8E, 0x42}); //LDE_REPC for preamble code 4, line-of-sight
    dwWriteReg(CSN, 0x1E, 0, 4, (const byte[]){0x48, 0x28, 0x08, 0x0E}); //TX_POWER for STXP, 16MHz PRF, channel 5
    dwWriteReg(CSN, 0x28, 0x0B, 1, (const byte[]){0xD8}); //RF_RXCTRLH for channel 5
    dwWriteReg(CSN, 0x28, 0x0C, 4, (const byte[]){0xE3, 0x3F, 0x1E, 0x00}); //RF_TXCTRL for channel 5
    dwWriteReg(CSN, 0x2A, 0x0B, 1, (const byte[]){0xB5}); //TC_PGDELAY for channel 5
    dwWriteReg(CSN, 0x2B, 0x07, 4, (const byte[]){0x1D, 0x04, 0x00, 0x08}); //FS_PLLCFG for channel 5
    dwWriteReg(CSN, 0x2B, 0x0B, 1, (const byte[]){0xBE}); //FS_PLLTUNE for channel 5

     // matdru: Set approximate RX antenna delay
    //dwWriteReg(CSN, 0x2E, 0x1804, 2, (const byte[]){0x79, 0x40}); // 16505 = 0x4079
    dwWriteReg(CSN, 0x2E, 0x1804, 2, (const byte[]){TX_ANT_DLY_LSB, TX_ANT_DLY_MSB}); 
     // matdru: Set approximate TX antenna delay
    //dwWriteReg(CSN, 0x18, 0, 2, (const byte[]){0x79, 0x40}); // 16505 = 0x4079
    dwWriteReg(CSN, 0x18, 0, 2, (const byte[]){TX_ANT_DLY_LSB, TX_ANT_DLY_MSB}); 
    // matdru

    delay(5);
}

void dwSetPAN_IDAndSHORT_ADDR(int CSN, uint16_t PAN_ID, uint16_t SHORT_ADDR) {
    byte buf[4];
    buf[0] = SHORT_ADDR & 0xFF;
    buf[1] = (SHORT_ADDR >> 8) & 0xFF;
    buf[2] = PAN_ID & 0xFF;
    buf[3] = (PAN_ID >> 8) & 0xFF;
    dwWriteReg(CSN, PANADR, 0, 4, buf);
}

void dwSetDelayedTime(int CSN, uint64_t delayedTime) {
    byte buf[5];
    buf[0] = 0x00;
    buf[1] = (delayedTime >> 8) & 0xFF;
    buf[2] = (delayedTime >> 16) & 0xFF;
    buf[3] = (delayedTime >> 24) & 0xFF;
    buf[4] = (delayedTime >> 32) & 0xFF;
    dwWriteReg(CSN, DX_TIME, 0, 5, buf); 
}

void dwStartReceive(int CSN, bool delayed) {
    byte buf[2] = {0x00, 0x01};
    if (delayed) buf[1] = 0x03;
    dwWriteReg(CSN, SYS_CTRL, 0, 2, buf);
}

bool dwIsReceiveTimestampAvailable(int CSN) {
    byte buf[2];
    dwReadReg(CSN, SYS_STATUS, 0, 2, buf);
    return (buf[1] & 0x04) == 0x04;
}

uint64_t dwGetTimestamp(int CSN) {
    byte buf[5];
    dwReadReg(CSN, SYS_TIME, 0, 5, buf);
    uint64_t timestamp = buf[4]; timestamp <<= 8;
    timestamp += buf[3]; timestamp <<= 8;
    timestamp += buf[2]; timestamp <<= 8;
    timestamp += buf[1]; timestamp <<= 8;
    timestamp += buf[0];
    return timestamp;
}

bool dwIsReceiveDone(int CSN) {
    byte buf[2];
    dwReadReg(CSN, SYS_STATUS, 0, 2, buf);
    return (buf[1] & 0x60) == 0x60;
}

bool dwIsReceiveError(int CSN) { //Note: Rejection due to frame filtering or timeout is NOT an error.
    byte buf[4];
    dwReadReg(CSN, SYS_STATUS, 0, 4, buf);
    return (buf[2] & 0x05) | (buf[1] & 0x90);
}

uint64_t dwGetReceiveTimestamp(int CSN) {
    byte buf[5];
    dwReadReg(CSN, RX_TIME, 0, 5, buf);
    uint64_t timestamp = buf[4]; timestamp <<= 8;
    timestamp += buf[3]; timestamp <<= 8;
    timestamp += buf[2]; timestamp <<= 8;
    timestamp += buf[1]; timestamp <<= 8;
    timestamp += buf[0];
    return timestamp;
}

void dwGetReceiveData(int CSN, int dataLen, byte* data) {
    dwReadReg(CSN, RX_BUFFER, 0, dataLen, data);
}

void dwClearReceiveStatus(int CSN) {
    byte buf[4] = {0x00, 0b11111111, 0b00100111, 0b00100100};
    //Reset RXDFR RXFCG RXPRD RXSFDD RXPHD LDEDONE
    //Reset RXRFTO RXPTO RXSFDTO
    //Reset RXPHE RXFCE RXRFSL AFFREJ LDEERR
    dwWriteReg(CSN, SYS_STATUS, 0, 4, buf);
}

void dwResetReceiver(int CSN) {
    byte pmscCtrl0[4];
    dwReadReg(CSN, PMSC, 0, 4, pmscCtrl0);
	pmscCtrl0[0] &= 0xFC;
	pmscCtrl0[0] |= 0x01;
	dwWriteReg(CSN, PMSC, 0, 4, pmscCtrl0); //Switch to SYS_XTI_CLOCK
    pmscCtrl0[3] &= 0xEF;
    dwWriteReg(CSN, PMSC, 0, 4, pmscCtrl0);
    pmscCtrl0[3] |= 0x10;
    dwWriteReg(CSN, PMSC, 0, 4, pmscCtrl0);
    //Further note that clock adjustment and AON block
}

void dwStartAccMemRead(int CSN) {
    byte reg[2]; //Reference to the official deca_device.cpp _dwt_enableclocks
    dwReadReg(CSN, PMSC, 0, 2, reg);
    reg[0] = 0x48 | (reg[0] & 0xb3);
    reg[1] = 0x80 | reg[1];
    dwWriteReg(CSN, PMSC, 0, 2, reg);
    delayMicroseconds(5);
}

void dwReadAccMem(int CSN, int index, int dataLen, byte* data) { //dataLen < 16
    byte tempData[16];
    dwReadReg(CSN, 0x25, index, dataLen + 1, tempData);
    for (int i = 0; i < dataLen; i++) {
        data[i] = tempData[i + 1]; //Dummy byte
    }
}

void dwEndAccMemRead(int CSN) {
    byte reg[2];
    dwReadReg(CSN, PMSC, 0, 2, reg);
    reg[0] = reg[0] & 0xb3;
    reg[1] = 0x7f & reg[1];
    dwWriteReg(CSN, PMSC, 0, 2, reg);
    delayMicroseconds(5);
}

void dwSetTransmitData(int CSN, int dataLen, const byte* data) {
    dwWriteReg(CSN, TX_BUFFER, 0, dataLen, data);
    byte buf[1];
    buf[0] = dataLen + 2;
    dwWriteReg(CSN, TX_FCTRL, 0, 1, buf);
}

//Advanced control like CANSFCS or HRBPT shall be implemented manually.
void dwStartTransmit(int CSN, bool delayed, bool wait4resp) {
    byte buf[1] = {0x02};
    if (delayed) buf[0] += 0x04;
    if (wait4resp) buf[0] += 0x80;
    dwWriteReg(CSN, SYS_CTRL, 0, 1, buf);
}

bool dwIsTransmitDone(int CSN) {
    byte buf[1];
    dwReadReg(CSN, SYS_STATUS, 0, 1, buf);
    return (buf[0] >> 7);
}

void dwClearTransmitStatus(int CSN) {
    byte buf[1];
    dwReadReg(CSN, SYS_STATUS, 0, 1, buf);
    buf[0] |= 0xF8;
    dwWriteReg(CSN, SYS_STATUS, 0, 1, buf);
}

void dwForceTRxOff(int CSN) {
    byte buf[1] = {0x40};
    dwWriteReg(CSN, SYS_CTRL, 0, 1, buf);
}

// matdru (from F-Army/https://github.com/F-Army/arduino-dw1000-ng)
void dwGetPrintableDeviceIdentifier(int CSN, char msgBuffer[]) {
		byte data[4];
		// _readBytesFromRegister(DEV_ID, NO_SUB, data, LEN_DEV_ID=4);
        dwReadReg(CSN, DEV_ID, 0, 4, data);
		sprintf(msgBuffer, "%02X - model: %d, version: %d, revision: %d",
						(uint16_t)((data[3] << 8) | data[2]), data[1], (data[0] >> 4) & 0x0F, data[0] & 0x0F);
}
// matdru