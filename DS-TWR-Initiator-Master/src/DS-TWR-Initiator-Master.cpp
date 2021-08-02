/* 
 *  File: DS-TWR-Initiator-Master.cpp/ino
 *  Date: 2021-07-31
 *  Original Author: https://github.com/Richardn2002
 *  Mods: Separation in .h/.cpp, mods for ESP32, second anchor: matdru
 */
/*
 *********
 * matdru
 ********* 

esp32-dw1000-lite - Simple Two-Way-Ranging application with DW1000 and ESP32
============================================================================

Description:
------------

This is a simple application of the  "DW1000lite" library and its example programms for
ranging as contributed at https://github.com/Richardn2002/arduino-dw1000-lite, but adapted 
to the ESP32 running with Arduino-Platform. As stated by the original author, this is not a 
full-fledged solution, but a way to get started with DW1000 very fast and test basic ranging 
functionality. See the original project at https://github.com/Richardn2002/arduino-dw1000-lite 
for further caveats and restrictions.

The test application was used as a quick proof-of-conceüt for determining distance of a remote 
(moving) "Responder"/tag from a base of two "Initiators"/anchors at a fixed distance e.g. 50cm 
and using the difference of the two measured distances to estimate direction (answer to question: 
"Is tag to the left or to the right and approx. by how much?):

```

                                Responder/Tag
                                       O
                                     /   \
                                    /     \
                                   /       \      ^
                                  /         \     |  UWB (802.15.4a @6489.6 GHz)
                                 /           \    v
                                /             \
                               /               \
                              o                 o
                        Initiator/Anchor  Initiator/Anchor
                            "Master"   <-->    "Slave"
                                     ESP-NOW 
                                     (Espressif proprietary @2.4 GHz)     
```                                                       

Function:
---------

DS-TWR-Initiator-Master and DS-TWR-Initiator-Slave duplicate the same hardware (ESP32 and DWS1000 shield) and the
same - simple - code, but DS-TWR-Initiator-Slave is synchronized to the DS-TWR-Initiator-Master using a basic ESP-NOW 
frame exchange (this could of course be achieved by other means e.g. using UWB itself -- but not with this library). 

So both anchors take turns at exchanging a POLL-REPLY-FINAL-DATA ranging conversation with the remote Responder/Tag:
The master measures its distance, than sends an ESP-NOW frame to the slave to indicate that it's its turn.
When the slave has finished its measurement, it sends the measured distance with an ESP-NOW reply frame, 
enabling the master to calculate the delta (difference of distance) and then performing the next train of 
measurements.

Note that the Responder/Tag does no frame filterung and is ignorant of the specific anchor it is dealing with.

Pin Connections:
----------------

```
    ESP32                        DWS1000 (with DWM1000)
    OUT: GND--------------------------IN:  GND   (= 0V: CON2 Pin 6 .. equiv to Arduino GND in .. 3rd from bottom upwards)
    OUT: Ext-5V-----------------------IN:  5V    (Power to onboard DC-DC: CON2 Pin 5 .. equiv. to Arduino 5V out .. 4th from bottom upwards)
    OUT: SS pin GPIO5-----------------IN:  SPICSn(Chip Select in: CON1 Pin 3 .. 3rd from bottom upwards)
    OUT: SCK SPI pin GPIO18-----------IN:  SCK   (SPI clock in:  CON1 Pin 6 .. 6th from bottom upwards)
    OUT: MOSI SPI pin GPIO23----------IN:  MOSI  (SPI Data in: CON1 Pin 4 .. 4th from bottom upwards)
    IN:  MISO SPI pin GPIO19----------OUT: MISO  (SPI Data out: CON1 Pin 5 .. 5th from bottom upwards) 
    OUT: RESET pin GPIO25-------------IN:  RSTn  (OpenDrain! CON4 on Pin 8 ... 8th from bottom upwards, 1st from top)    

    Note: Above pinout is for VSPI on ESP32 (NodeMCU/DevKitC) with Arduino lib!     

    Not used:
    IN: interrupt D0 pin GPIO27-------OUT: IRQ  (interrupt request out: CON1 Pin 1 .. equiv. to Arduino CLK0 ..1st from bottom upwards)
```
 *
 *********
 */

#include <DW1000.h>
#include <esp_now.h>
#include <Wifi.h>

const int A_CSN = 5;
const int A_RST = 25;

double distance;

// matdru: For simple ESP-Now inter-anchor communication (taking turns with ranging)
uint8_t slaveMACAddress[] = {0x24, 0x6F, 0x28, 0x10, 0x8C, 0x60}; 
bool myturn = true;

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
  Serial.printf("*** DELTA = %.2f ***\n", (float)(distance - msgBuf.fval)*100);
  myturn = true;
}

esp_now_peer_info_t peerInfo;   // needs to be global in Arduino implementation

void initAnchorComm() {
    // From: https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    Serial.println(WiFi.macAddress());

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully initialized, we will register Send callback to
    // get the status of transmitted packet
    esp_now_register_send_cb(cbOnDataSent);
    
    memcpy(peerInfo.peer_addr, slaveMACAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }
    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(cbOnDataRecv);
}
// matdru

// matdru: Removed magic OFFSET in original code by setting antenna delay in device 
// _and_ adding it to delayed transmit times where aprpropriate...
//double OFFSET = 153900;
// matdru

unsigned long updateTime;
unsigned long disconnectTimer;

void setup() {
    Serial.begin(9600);

    // matdru: setup ESP-Now simple comm between the 2 "anchors" (in this case = ranging initiators)
    initAnchorComm();
    // matdru

    SPI.begin();
    dwInit(A_CSN, A_RST);

    // matdru: test SPI connection to DW1000
    char devString[64];
    dwGetPrintableDeviceIdentifier(A_CSN, devString);
    Serial.println(devString);
    // matdru

    updateTime = millis();
    disconnectTimer = millis();

}

bool timeout = false;

uint64_t delayedTx(int CSN) {
    dwForceTRxOff(CSN);
    dwClearTransmitStatus(CSN);

    // matdru: Delay is set to approx. 1ms with this code (less did not work, tried it...)
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

uint64_t waitForResponse(int CSN) {
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

uint64_t getReplyTime(int CSN) {
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

uint64_t getRoundTime(int CSN) {
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

double getDistance(int CSN) {
    timeout = false;

    uint64_t round1A;
    uint64_t round1B;
    uint64_t round1;
    uint64_t reply1;
    uint64_t reply2;
    uint64_t round2;
    uint64_t propTime;

    // matdru: Mod: Add antenna delay to T_round1 calculation
    round1A = delayedTx(CSN) + TX_ANT_DLY;
    round1B = waitForResponse(CSN);
    reply1 = getReplyTime(CSN);
    round1 = round1B - round1A;
    // matdru: Mod: Add antenna delay to T_reply2 calculation
    reply2 = delayedTx(CSN) - round1B + TX_ANT_DLY;
    waitForResponse(CSN);

    if (timeout) {
        return 0;
    }
    round2 = getRoundTime(CSN);
    propTime = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);

    // matdru:
    //Serial.printf("round1=%0llx round2=%0llx reply1=%0llx reply2=%0llx propTime=%0llx\r\n", round1, round2, reply1, reply2, propTime);
    // matdru

    double prop = (double)propTime;
    
    // matdru: Removed magic OFFSET in original code by setting antenna delay in device 
    // _and_ adding it to delayed transmit times where aprpropriate...
    //return prop * 4.6917639786 - OFFSET; 
    
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
// matdru

unsigned long waitForSlaveStartTime = millis();
#define WAIT_FOR_SLAVE_TIMEOUT_MS  1000

void loop() {
    if (myturn) {
        distance = getDistance(A_CSN);

        // matdru: mod for simple moving average
        if (distp - avgBuf >= AVG_BUF_LEN)
            distp = avgBuf;

        if (distance >= 0 && distance < 100)
            *distp++ = distance;
        // matdru

        if (timeout) {
            if (millis() - disconnectTimer > 50) {
                dwInit(A_CSN, A_RST);
                disconnected = true;
            }
        } else {
            disconnected = false;
            disconnectTimer = millis();
        }
/*
        if (distance) {
            Serial.println(distance);
        }
*/
        if (counter++ % AVG_BUF_LEN == 0) {
            Serial.printf("%.2lfcm\r\n", movAvg(avgBuf, AVG_BUF_LEN)*100);
             // Send go-ahead message via ESP-NOW
            const char *messageStr = "Hello Slave anchor!";
            memcpy(msgBuf.msg, messageStr, strlen(messageStr)+1);
            msgBuf.fval = 666.6;    // dummy value
            esp_err_t result = esp_now_send(slaveMACAddress, (uint8_t *) &msgBuf, sizeof(msgBuf));
            if (result == ESP_OK) {
                //Serial.println("Sent with success");
            }
            else {
                //Serial.println("Error sending the data");
            }
            waitForSlaveStartTime = millis();
            myturn = false;
        }
    }
    else {
        if (millis() - waitForSlaveStartTime > WAIT_FOR_SLAVE_TIMEOUT_MS) {
            //Serial.println("Timed out waiting for slave anchor...");
            myturn = true;
        }
    }
}
