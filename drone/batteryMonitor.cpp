/**
    \file batteryMonitor.cpp
    \date 2017-03-12
    \author Felix de Neve
    \brief Monitors battery level with ADC and comparator.
*/

#include "batteryMonitor.hpp"

/**
    \brief ISR to ramp down motors when battery is low.
*/
ISR(ANALOG_COMP_vect)
{
    rampDown(3);
}

/**
    \brief Initialise the ADC and comparator for battery monitoring.
*/
void init_battery_monitor()
{
    ADCSRA = B00000000;
    ADCSRB = B01000000;
    ACSR   = B00011011;
    ADMUX  = B11000101;
    sei();
}
