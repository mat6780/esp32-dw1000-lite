/* 
 *  File: DS-TWR-Initiator-Slave.cpp
 *  Date: 2021-07-31
 *  Original Author: https://github.com/Richardn2002
 *  Mods: Separation in .h/.cpp, mods for ESP32, second anchor: matdru
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
#include <Wifi.h>
#include <esp_now.h>

const int A_CSN = 5;
const int A_RST = 25;

uint8_t myMACAddress[] = {0x24, 0x6F, 0x28, 0x10, 0x8C, 0x60};    // {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t otherMACAddress[] = {0xA4, 0xCF, 0x12, 0x24, 0x52, 0x24};
bool myturn = 0;

#define MSG_STR_SIZ 60

typedef struct {
    char msg[MSG_STR_SIZ];
    float fval;
} MsgBuf;

MsgBuf msgBuf;

// Callback when data is sent
// From: https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/
void cbOnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void cbOnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&msgBuf, incomingData, sizeof(msgBuf));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  //Serial.printf("Received \"%s\"\t\t%.2f\n", msgBuf.msg, msgBuf.fval);
  if (myturn == false)
    myturn = true;
}

// matdru: This apparently needs to be global (error "Peer interface is invalid" otherwise)
esp_now_peer_info_t peerInfo;

void initAnchorComm() {
    // From: https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/

    // Set device as a Wi-Fi Station and print MAC-Address informationally
    WiFi.mode(WIFI_STA);
    Serial.println(WiFi.macAddress());

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully initialized, we will register for Send CB to
    // get the status of transmitted packet
    esp_now_register_send_cb(cbOnDataSent);

    // Register peer
    
    memcpy(peerInfo.peer_addr, otherMACAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }
    // Register a callback function that will be called when data is received
    esp_now_register_recv_cb(cbOnDataRecv);
}

// matdru: Removed magic OFFSET in original code by setting antenna delay in device 
// _and_ adding it to delayed transmit times where aprpropriate...
//double OFFSET = 153900;
// matddru

unsigned long updateTime;
unsigned long disconnectTimer;

void setup() {
    Serial.begin(9600);

    // matdru: setup ESP-Now simple comm between the 2 "anchors" (in this case: ranging initiators)
    initAnchorComm();
    // matdru

    SPI.begin();
    dwInit(A_CSN, A_RST);

    // matdru: test SPI connection
    char devString[64];
    dwGetPrintableDeviceIdentifier(A_CSN, devString);
    Serial.println(devString);
    // matdru

    updateTime = millis();
    disconnectTimer = millis();
}

bool timeout = false;

uint64_t DelayedTx(int CSN) {
    dwForceTRxOff(CSN);
    dwClearTransmitStatus(CSN);

    // matdru: Delay is set to approx. 1ms with this code (less did not work, tried it...)
    // (this is the original code)
    uint64_t sendTime = dwGetTimestamp(A_CSN) >> 25;
    sendTime += 2;
    sendTime <<= 25;
    // matdru

    dwSetDelayedTime(CSN, sendTime);
    dwSetTransmitData(CSN, 1, (const byte[]){0x00});
    dwStartTransmit(CSN, true, false);
    unsigned long startTime = micros();
    while(!dwIsTransmitDone(CSN)) {
        delayMicroseconds(40);
        if (micros() - startTime > 2000) {
            timeout = true;
            break;
        }
    }

    return sendTime;
}

uint64_t WaitForResponse(int CSN) {
    dwStartReceive(CSN, false);

    unsigned long startTime = micros();
    while(!dwIsReceiveDone(CSN)) {
        delayMicroseconds(40);
        if (micros() - startTime > 2000) {
            timeout = true;
            break;
        }
    }

    return dwGetReceiveTimestamp(CSN);
}

uint64_t GetReplyTime(int CSN) {
    byte data[10];
    uint64_t replyTime;

    dwGetReceiveData(CSN, 10, data);
    replyTime = data[0]; replyTime <<= 8;
    replyTime += data[1]; replyTime <<= 8;
    replyTime += data[2]; replyTime <<= 8;
    replyTime += data[3]; replyTime <<= 8;
    replyTime += data[4];

    return replyTime;
}

uint64_t GetRoundTime(int CSN) {
    byte data[10];
    uint64_t roundTime;

    dwGetReceiveData(CSN, 10, data);
    roundTime = data[5]; roundTime <<= 8;
    roundTime += data[6]; roundTime <<= 8;
    roundTime += data[7]; roundTime <<= 8;
    roundTime += data[8]; roundTime <<= 8;
    roundTime += data[9];

    return roundTime;
}

double GetDistance(int CSN) {
    timeout = false;

    uint64_t round1A;
    uint64_t round1B;
    uint64_t round1;
    uint64_t reply1;
    uint64_t reply2;
    uint64_t round2;
    uint64_t propTime;

    // matdru: Mod: Add antenna delay to T_round1 calculation
    round1A = DelayedTx(CSN) + TX_ANT_DLY;
    round1B = WaitForResponse(CSN);
    reply1 = GetReplyTime(CSN);
    round1 = round1B - round1A;
    // matdru: Mod: Add antenna delay to T_reply2 calculation
    reply2 = DelayedTx(CSN) - round1B + TX_ANT_DLY;
    WaitForResponse(CSN);

    if (timeout) {
        return 0;
    }
    round2 = GetRoundTime(CSN);
    propTime = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);

    // matdru:
    //Serial.printf("round1=%0llx round2=%0llx reply1=%0llx reply2=%0llx propTime=%0llx\r\n", round1, round2, reply1, reply2, propTime);
    // matdru

    double prop = (double)propTime;
    
    return prop * DISTANCE_PER_DWT_TIME_UNIT;        
    // matdru
}

bool disconnected = false;

// matdru: mod for simple moving average
#define AVG_BUF_LEN 64
double avgBuf[AVG_BUF_LEN], *distp = avgBuf;


double movAvg(double buf[], int len) {
    double *dp = buf, sum = 0;

    while ((dp < buf + len) && *dp) {
        sum += *dp++;
    }
    if (dp > buf)
        return sum / (dp - buf);
    else
        return 0;
}

u_int counter = 0;
// matdru: end mod

void loop() {

    if (myturn) {
        double distance = GetDistance(A_CSN);

        // matdru: mod for simple moving average
        if (distp - avgBuf >= AVG_BUF_LEN)
            distp = avgBuf;

        if (distance >= 0 && distance < 100)
            *distp++ = distance;
        // matdru: end mod


        if (timeout) {
            if (millis() - disconnectTimer > 50) {
                dwInit(A_CSN, A_RST);
                disconnected = true;
            }
        } else {
            disconnected = false;
            disconnectTimer = millis();
        }

        if (distance) {
            //Serial.println(distance);
        }

        if (counter++%AVG_BUF_LEN == 0) {
            float avgDistance = (float) movAvg(avgBuf, AVG_BUF_LEN);
            Serial.printf("%.2lfcm\r\n", avgDistance*100);
            // Send message via ESP-NOW
            const char *messageStr = "Hello Master anchor!";
            memcpy(msgBuf.msg, messageStr, strlen(messageStr)+1);
            msgBuf.fval = (float) avgDistance;
            esp_err_t result = esp_now_send(otherMACAddress, (uint8_t *) &msgBuf, sizeof(msgBuf));

            if (result == ESP_OK) {
                //Serial.print("Sent with success");
                //Serial.println(avgDistance);
            }
            else {
                Serial.println("Error sending the data");
            }
            myturn = false;
        }
    }
}
