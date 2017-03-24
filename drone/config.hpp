/**
    \file config.hpp
    \date 2017-03-12
    \author Tom Darlison
    \brief Configuration defines for debugging and drone operation.
*/

#ifndef CONFIG_HPP
#define CONFIG_HPP

// radio defines
#define NODEID			 1  //network ID used for this unit
#define NETWORKID		99  //the network ID we are on
#define SERIAL_BAUD 115200

// interdrone serial defines
#define CHECKSUM_FAILURE_MAX 10

// program configuration parameters
//#define SERIAL_USB_ACTIVE  	// send serial data down USB
//#define DEBUG 				// allows wired elevation control
#define CALIBRATE_ESC		// recalibarates the ESC's on startup
#define RAMP_UPDOWN_TEST	// ramps up to a %of duty cycle range and back down
#define USE_RADIO			// activates radio control
//#define RADIO_TEST 			// just prints recieved radio - InfLoop
//#define MOTOR_CUT_TEST 		// results in infinite loop
#define MOTORS_AT_ZERO_ELEV	// allow motors to work at zero elevation
#define AUDIBLE_SAFTEY 		// sounds buzzer on rampdown
//#define BATTERY_MONITOR		// check battery voltage, ramp when low

#endif // CONFIG_HPP
