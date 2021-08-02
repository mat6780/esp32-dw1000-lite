sp32-dw1000-lite - Very Basic Two-Way-Ranging application with DW1000 and ESP32
===================================================================)============

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