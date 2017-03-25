/**
    \file drone.ino
    \date 2017-03-08
    \author George Brown
    \author Rhys Thomas
    \author Tom Darlison
    \brief Collection of all code efforts into one main file to run on the
    leonardo.
    \image html main-code.png
*/

// includes
#include "config.hpp"
#include "comms.hpp"
#include "batteryMonitor.hpp"

// instance of the radio module
RFM12B radio;

// control parameters
Parameters elevation(false);
Parameters roll(false);
Parameters pitch(false);
Parameters yaw(true);

// motor speed pwm values
float motorA = PWM_DUTY_MIN;
float motorB = PWM_DUTY_MIN;
float motorC = PWM_DUTY_MIN;
float motorD = PWM_DUTY_MIN;

unsigned char msg[10];
unsigned int bytesRead = 0;

float yaw_v;
float pitch_v;
float roll_v;
int16_t yaw_in;
int16_t pitch_in;
int16_t roll_in;
int16_t elev_in = 0;

uint8_t checksum_failures = 0;
bool newMPUData = false;
unsigned long lastMPUDataTime;
unsigned long lastRadioData;

/**
    \brief Setup peripherals.
    \author George Brown
    \author Rhys Thomas
    \author Tom Darlison

    The setup uses the following workflow. First start the serial Communications
    between Leonardo and Pro Micro, initialise the timers used to drive the
    motors, set PID values for each control parameter, reset setpoints for
    takeoff, calibrate ESCs and test motors with a ramping function and finally
    initialise the radio at 433MHz. The program then moves into the loop
    function.
*/
void setup()
{
	#ifdef BATTERY_MONITOR
		init_battery_monitor();
	#endif

	Serial1.begin(115200);
	//Serial1.setTimeout(10);
	#ifdef SERIAL_USB_ACTIVE
		Serial.begin(115200);
		while (!Serial) {;}
		_delay_ms(1000);
	    Serial.println("Begin RX!");
	#endif

    // setup the timer registers
    init_timers();

	// set PID values
	elevation.setPID(1,0,0);
	roll.setPID(0.271473,0.444332,0.034295);
	pitch.setPID(-0.376223,-0.619688,-0.013624);
	yaw.setPID(-0.2,0,0);

	// set up rest points
	elevation.updateInput(0);
	roll.updateInput(127);
	pitch.updateInput(127);
	yaw.updateInput(127);

	#ifdef CALIBRATE_ESC
		calibarateESC();
	#endif

	#ifdef RAMP_UPDOWN_TEST
		rampTest(0.1); // 10%
	#endif

	#ifdef USE_RADIO
		#ifdef SERIAL_USB_ACTIVE
			Serial.println("Setting up radio");
		#endif

        	radio.Initialize(NODEID, RF12_433MHZ, NETWORKID);

		#ifdef SERIAL_USB_ACTIVE
			Serial.println("Radio set up");
		#endif
	#endif

	#ifdef RADIO_TEST
		#ifndef USE_RADIO
			radio.Initialize(NODEID, RF12_433MHZ, NETWORKID);
		#endif
		radioTest();
	#endif

	#ifdef MOTOR_CUT_TEST
		elevation.updateInput(50);
		elevation.updateElev();
		roll.updateOutput();
		pitch.updateOutput();
		yaw.updateOutput();

		motorA = elevation.output - roll.output - pitch.output + yaw.output;
		motorB = elevation.output + roll.output - pitch.output - yaw.output;
		motorC = elevation.output + roll.output + pitch.output + yaw.output;
		motorD = elevation.output - roll.output + pitch.output - yaw.output;

		pwm_duty(motorA, motorB, motorC, motorD);

		#ifdef SERIAL_USB_ACTIVE
			Serial.println("wait 1s");
		#endif
		_delay_ms(1000);

		rampDown(0);
	#endif

	#ifdef SERIAL_USB_ACTIVE
		Serial.println("Waiting for interdrone serial");
	#endif

    // Clear the buffer.
    while(Serial1.available()){
        Serial1.read();
    }
    // Wait for fresh serial data from the IMU before starting.
    while(!Serial1.available());

    #ifdef SERIAL_USB_ACTIVE
    	Serial.println("Starting main loop");
    #endif

    lastMPUDataTime = millis();
}

/**
    \brief Loop for main drone operation.
    \author George Brown
    \author Rhys Thomas
    \author Tom Darlison

    The main loop gets a packet from the Pro Micro which is reading the IMU data
    then updates the direction setpoints, checks for data being received by the
    radio, if so update the setpoints with those values, then update the speed
    of the motors in order to reach these setpoints.
*/
void loop()
{
    while(1)//Serial1.available());
    {
        while((Serial1.peek() != 'P') && Serial1.available())
        {
            Serial1.read();
        }
        if(Serial1.available() >= 11){
	        bytesRead = Serial1.readBytes(msg, 11);
	        if ((msg[0] == 'P') && (msg[10] == '\n') && (bytesRead == 11))
	        {
	            if(checksum(msg, 9) == msg[9])
	            {
	                if(checksum_failures > 0)
	                    checksum_failures--;
	                lastMPUDataTime = millis();
	                break;
	            }
	            else
	            {
	                checksum_failures++;
	                if(checksum_failures > CHECKSUM_FAILURE_MAX)
	                    rampDown(1);
	            }
	        }
    	}
    	// Fall over if the MPU data isn't fresh.
    	if((millis() - lastMPUDataTime) > 200){
    		// Serial.print("time now: "); Serial.println(millis());
    		// Serial.print("last data time: "); Serial.println(lastMPUDataTime);
    		rampDown(1);
    	}
    }


    // update MPU inputs
    yaw_in   = (msg[1] << 8) | msg[2];
    pitch_in = (msg[4] << 8) | msg[5];
    roll_in  = (msg[7] << 8) | msg[8];

    yaw_v = yaw_in / 182.0f;
    pitch_v = pitch_in / 364.0f;
    roll_v = roll_in / 364.0f;

    yaw.updateActual(yaw_v);
    pitch.updateActual(pitch_v);
    roll.updateActual(roll_v);

    #ifdef SERIAL_USB_ACTIVE
        Serial.print(yaw_v);
        Serial.print(",\t");
        Serial.print(pitch_v);
        Serial.print(",\t");
        Serial.println(roll_v);
    #endif

	// radio recieve
	#ifdef USE_RADIO
		radio_rx();
	#endif

	// update inputs
	elevation.updateElev();
	roll.updateOutput();
	pitch.updateOutput();
	yaw.updateOutput();

	// calculate motor values
	#ifdef DEBUG
		if(Serial1.available())
		{
			char command = Serial1.read();
			Serial1.print(command);
			if(command == '+')
			{
				motorA++;
				motorA > PWM_DUTY_MAX ? motorA = PWM_DUTY_MAX : motorA;
			}
			else if(command == '-')
			{
				motorA--;
				motorA < PWM_DUTY_MIN ? motorA = PWM_DUTY_MIN : motorA;
			}
			else if(command == '0')
			{
				motorA = PWM_DUTY_MIN;
			}
			motorB = motorC = motorD = motorA;
		}
	#else
		#ifdef MOTORS_AT_ZERO_ELEV
			motorA = elevation.output - roll.output - pitch.output + yaw.output;
			motorB = elevation.output + roll.output - pitch.output - yaw.output;
			motorC = elevation.output + roll.output + pitch.output + yaw.output;
			motorD = elevation.output - roll.output + pitch.output - yaw.output;
		#else

			if(elevation.output < (PWM_DUTY_MIN + 20))
			{
				motorA = PWM_DUTY_MIN;
				motorB = PWM_DUTY_MIN;
				motorC = PWM_DUTY_MIN;
				motorD = PWM_DUTY_MIN;
			}
			else
			{
				motorA = elevation.output - roll.output - pitch.output + yaw.output;
				motorB = elevation.output + roll.output - pitch.output - yaw.output;
				motorC = elevation.output + roll.output + pitch.output + yaw.output;
				motorD = elevation.output - roll.output + pitch.output - yaw.output;
			}
		#endif
	#endif

	// update PWM flags
	pwm_duty(motorA, motorB, motorC, motorD);
}
