/**
    \file ground.ino
    \date 2017-03-10
    \author Rhys Thomas
    \brief Transmits control bytes to the drone.
    \details Gets 5 control bytes from Raspberry Pi via serial and transmits to the
    drone.
*/

#include <RFM12B.h>
#include <avr/sleep.h>

#define NODEID      2       // network ID used for this unit
#define NETWORKID   99      // the network ID we are on
#define GATEWAYID   1       // the node ID we're sending to
#define SERIAL_BAUD 115200

// instance of radio module
RFM12B radio;

uint8_t payload[5];
byte sendSize = sizeof(payload); // sending 4 bytes
bool requestACK = true;

/**
    \brief Sets up the radio module for communcations.
    \author Rhys Thomas
 */
void setup()
{
    Serial.begin(SERIAL_BAUD);
    radio.Initialize(NODEID, RF12_433MHZ, NETWORKID);
}

/**
    \brief Transmission function for ground station.
    \author Rhys Thomas

    The loop waits for 5 bytes of data from the raspberry pi, these bytes have
    the format of [stop,x1,y1,x2,y2] where stop is the kill switch control on
    the drone. The stop tells the drone if the packet order is correct and if
    not the rampDown() function is called to stop the motors and land.

    \see rampDown
 */
void loop()
{
    while (Serial.available() < 5) {} // wait for 5 bytes of data

    for (int n = 0; n < 5; n++) {
        payload[n] = Serial.read();
    }

    // transmit
    radio.Send(GATEWAYID, payload, sendSize, requestACK);
}
