/**
	\file comms.hpp
    \date 2017-03-11
	\author Rhys Thomas
    \author George Brown
	\brief Communication functions.
*/

#ifndef _COMMS_H_
#define _COMMS_H_

// includes
#include "config.hpp"
#include "params.hpp" // parameters class
#include "pwm.hpp"    // rampDown function

// libraries
#include <RFM12B.h>   // rfm12b class

// function declarations
unsigned char checksum(unsigned char *packet, unsigned int len);
void radio_rx();
void radioTest();

#endif // _COMMS_H_
