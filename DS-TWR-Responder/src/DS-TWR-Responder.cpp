/* 
 *  File: DS-TWR-Responder.cpp
 *  Date: 2021-07-31
 *  Original Author: https://github.com/Richardn2002
 *  Mods: Separation in .h/.cpp, mods for ESP32: mat6780@gmail.com (matdru)
 *  For some documentation, see DS-TWR-Initiator-Master.cpp and 
 *  original project at https://github.com/Richardn2002/arduino-dw1000-lite
 */
/*
 *******
 * matdru:
 ******* 
 * Connections:

    ESP32                        DWM1000
    OUT: GND--------------------------IN:  GND   (ground in)
    OUT: Ext-5V-----------------------IN:  5V    (Power to DC-DC: CON2 Pin 5 .. equiv. to Arduino 5V out .. 4th from bottom upwards)
    OUT_ SS pin GPIO5-----------------IN:  SPICsn(Chip Select in: CON1 Pin 3 .. 3rd from bottom upwards)
    OUT: SCK SPI pin GPIO18-----------IN:  SCK   (SPI clock in:  CON1 Pin 6 .. 6th from bottom upwards)
    OUT: MOSI SPI pin GPIO23----------IN:  MOSI  (SPI Data in: CON1 Pin 4 .. 4th from bottom upwards)
    IN:  MISO SPI pin GPIO19----------OUT: MISO  (SPI Data out: CON1 Pin 5 .. 5th from bottom upwards) 
    OUT: RESET pin GPIO25-------------IN:  RSTn  (OpenDrain! CON4 on Pin 8 ... 8th from bottom upwards, 1st from top)    

    Note: The pinout is for VSPI on ESP32 (NodeMCU/DevKitC) with Arduino lib!     

    Not used:
    IN: interrupt D0 pin GPIO27-------OUT: IRQ  (interrupt request out: CON1 Pin 1 .. equiv. to Arduino CLK0 ..1st from bottom upwards)
 *
 ******
 */
#include <DW1000.h>

const int A_CSN = 5;
const int A_RST = 25;

unsigned long disconnectTimer;

void setup() {
    Serial.begin(9600);
    SPI.begin();

    dwInit(A_CSN, A_RST);
    disconnectTimer = millis();
 
    // matdru: test SPI connection
    char devString[64];
    dwGetPrintableDeviceIdentifier(A_CSN, devString);
    Serial.println(devString);
    // matdru

}

uint64_t previousTx;
bool timeout;

uint64_t DelayedTx() {
    dwForceTRxOff(A_CSN);
    dwClearTransmitStatus(A_CSN);

    // matdru: This sets delay to approxx. 1ms (less did not work...)
    uint64_t sendTime = dwGetTimestamp(A_CSN) >> 25;
    sendTime += 2;
    sendTime <<= 25;

    dwSetDelayedTime(A_CSN, sendTime);

    // matdru: add antenna delay
    sendTime += TX_ANT_DLY;
    // matdru

    byte msg[10];
    uint64_t replyTime;
    uint64_t roundTime;
    uint64_t receiveTime = dwGetReceiveTimestamp(A_CSN);
    replyTime = sendTime - receiveTime;
    roundTime = receiveTime - previousTx;
    msg[0] = 0xFF&(replyTime>>32);
    msg[1] = 0xFF&(replyTime>>24);
    msg[2] = 0xFF&(replyTime>>16);
    msg[3] = 0xFF&(replyTime>>8);
    msg[4] = 0xFF&replyTime;
    msg[5] = 0xFF&(roundTime>>32);
    msg[6] = 0xFF&(roundTime>>24);
    msg[7] = 0xFF&(roundTime>>16);
    msg[8] = 0xFF&(roundTime>>8);
    msg[9] = 0xFF&roundTime;
    dwSetTransmitData(A_CSN, 10, msg);
    dwStartTransmit(A_CSN, true, false);
    unsigned long startTime = micros();
    while(!dwIsTransmitDone(A_CSN)) {
        delayMicroseconds(40);
        if (micros() - startTime > 2000) {
            timeout = true;
            break;
        }
    }

    return sendTime;
}

void loop() {
    timeout = false;
    dwForceTRxOff(A_CSN);
    dwStartReceive(A_CSN, false);
    unsigned long startTime = millis();
    
    while(!dwIsReceiveDone(A_CSN)) {
        delayMicroseconds(40);
        if (millis() - startTime > 5) {
            timeout = true;
            break;
        }
    }
    
    previousTx = DelayedTx();
    
    if (timeout) {
        if (millis() - disconnectTimer > 100) {
            Serial.println("Timed out.");
            dwInit(A_CSN, A_RST);
        }
    } else {
        disconnectTimer = millis();
    }
}
