/**
    \file pwm.hpp
    \date 2017-03-11
    \author Tom Darlison
    \author George Brown
    \brief PWM motor control functions.
*/

#ifndef PWM_HPP
#define PWM_HPP

// includes
#include "config.hpp"

// libraries
#include <avr/io.h>
#include <util/delay.h>
#include <Arduino.h>

// PWM definess
#define PWM_MAX 	   5000	// 1<<10.64 - to get 10kHz
#define PWM_DUTY_MIN   2500 // §>0
#define PWM_DUTY_MAX   4050	// §<PWM_MAX
#define PWM_DUTY_LIMIT 3500	// §<PWM_MAX

// function declarations
void init_timers();
void pwm_duty(const float& motorA, const float& motorB,
              const float& motorC, const float& motorD);
void calibarateESC();
void rampTest(const float& percentageOfMax);
void rampDown(const uint8_t ERROR_NUMBER = 0);

#endif // PWM_HPP
