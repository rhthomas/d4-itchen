/**
	\file comms.cpp
    \date 2017-03-11
    \author Rhys Thomas
    \author George Brown
	\brief Communication functions.
*/

#include "comms.hpp"

// instance of radio module
extern RFM12B radio;
extern Parameters elevation, pitch, roll, yaw;

/**
    \brief Compute the XOR sum for the first len bytes of an array.
    \author George Brown
    \param packet Data to compute the checksum on.
    \param len Number of bytes to compute the checksum for.
    \return Computed checksum.
*/
unsigned char checksum(unsigned char *packet, unsigned int len)
{
	unsigned char csum = 0;
	unsigned int idx;
	for(idx = 0; idx < len; idx++)
	{
		csum ^= packet[idx];
	}
	return csum;
}

/**
    \brief Receives Radio data.
    \author Rhys Thomas

    Gets complete radio data and sets the input controls to the relevant
    variables of elevation, roll, pitch and yaw. If data does not pass first
    byte alignment, call the rampDown function.

    \see rampDown
*/
void radio_rx()
{
	if(radio.ReceiveComplete()) {
		if(radio.CRCPass()) {
			if(radio.Data[0] != 0xAA) {
				rampDown(2);
			}
			// update the params setpoints
			elevation.updateInput(radio.Data[1]);
			yaw.updateInput(radio.Data[2]);
			pitch.updateInput(radio.Data[3]);
			roll.updateInput(radio.Data[4]);
			#ifdef SERIAL_USB_ACTIVE
				Serial.print("\t\t\t\t\t");
				Serial.print(radio.Data[1]); Serial.print(",\t");
				Serial.print(radio.Data[2]); Serial.print(",\t");
				Serial.print(radio.Data[3]); Serial.print(",\t");
				Serial.print(radio.Data[4]); Serial.println("");
			#endif
			if(radio.ACKRequested())
				radio.SendACK();
		}
	}
}

/**
    \brief Test code for radio recieving.
    \author Rhys Thomas

    Gets radio data and prints over serial USB forever.
*/
void radioTest()
{
	#ifdef SERIAL_USB_ACTIVE
		Serial.println("Starting Radio Test");
	#endif
	while(true)
	{
		if(radio.ReceiveComplete()) {
			if(radio.CRCPass()) {
				if(radio.Data[0] != 0xAA) {
					rampDown(2);
				}
				// update the params setpoints
				Serial.print(radio.Data[1]);
				Serial.print(",\t");
				Serial.print(radio.Data[2]);
				Serial.print(",\t");
				Serial.print(radio.Data[3]);
				Serial.print(",\t");
				Serial.print(radio.Data[4]);
				Serial.println("");
			}
		}
	}
}
