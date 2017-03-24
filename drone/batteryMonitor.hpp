/**
    \file batteryMonitor.hpp
    \date 2017-03-12
    \author Felix de Neve
    \brief Monitors battery level with ADC and comparator.
*/

#ifndef BATTERY_MONITOR_HPP
#define BATTERY_MONITOR_HPP

// includes
#include "pwm.hpp"
#include <avr/interrupt.h>

// interupt service routines
ISR(ANALOG_COMP_vect);

// function declarations
void init_battery_monitor();

#endif // BATTERY_MONITOR_HPP
