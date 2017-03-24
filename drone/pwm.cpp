/**
    \file pwm.cpp
    \date 2017-03-11
    \author Tom Darlison
    \author George Brown
    \brief PWM motor control functions.
*/

#include "pwm.hpp"

/**
    \brief Initialise the timers for PWM.
    \author Tom Darlison
    \author George Brown

    Sets up timers to produce 400Hz PWM wave.
*/
void init_timers()
{
    // timer 1
    #ifdef SERIAL_USB_ACTIVE
        Serial.println("Setting up timers");
    #endif

    DDRB |= _BV(PB5); // timer 1 A --pin9
    DDRB |= _BV(PB6); // timer 1 B --pin10
    DDRB |= _BV(PB7); // timer 1 C --pin11

    TCCR1A = _BV(WGM11) // fast PWM 10-bit
        | _BV(COM1A1)    // flag A
        | _BV(COM1B1)    // flag B
        | _BV(COM1C1);   // flag C

    TCCR1B = _BV(WGM12)
        | _BV(WGM13)
        | _BV(CS11); // 8 prescale
    ICR1 = PWM_MAX;

    // timer 3
    DDRC |= _BV(PC6); // timer 3 A --pin5

    TCCR3A = _BV(WGM31) // fast PWM 10-bit
        | _BV(COM3A1);   // flag A

    TCCR3B = _BV(WGM32)
        | _BV(WGM33)
        | _BV(CS31); // 8 prescale
    ICR3 = PWM_MAX;

    #ifdef SERIAL_USB_ACTIVE
        Serial.println("Timers set up");
    #endif
}

/**
    \brief Set the duty cycle of each motor.
    \author Tom Darlison
    \param motorA Sets motorA duty cycle.
    \param motorB Sets motorB duty cycle.
    \param motorC Sets motorC duty cycle.
    \param motorD Sets motorD duty cycle.
*/
void pwm_duty(const float& motorA, const float& motorB,
              const float& motorC, const float& motorD)
{
    (motorA < PWM_DUTY_MIN) ? (OCR1A = PWM_DUTY_MIN) : ((motorA > PWM_DUTY_LIMIT)
            ? (OCR1A = PWM_DUTY_LIMIT) : (OCR1A = motorA)); // set duty cycle for T1A
    (motorB < PWM_DUTY_MIN) ? (OCR1B = PWM_DUTY_MIN) : ((motorB > PWM_DUTY_LIMIT)
            ? (OCR1B = PWM_DUTY_LIMIT) : (OCR1B = motorB)); // set duty cycle for T1B
    (motorC < PWM_DUTY_MIN) ? (OCR1C = PWM_DUTY_MIN) : ((motorC > PWM_DUTY_LIMIT)
            ? (OCR1C = PWM_DUTY_LIMIT) : (OCR1C = motorC)); // set duty cycle for T1C
    (motorD < PWM_DUTY_MIN) ? (OCR3A = PWM_DUTY_MIN) : ((motorD > PWM_DUTY_LIMIT)
            ? (OCR3A = PWM_DUTY_LIMIT) : (OCR3A = motorD)); // set duty cycle for T3A
}

/**
    \brief Calibrate ESCs for our values.
    \author Tom Darlison
*/
void calibarateESC()
{
#ifdef SERIAL_USB_ACTIVE
    Serial.println("Starting ESC calibration\ncalibrate max, 7s wait");
#endif

    pwm_duty(PWM_DUTY_MAX, PWM_DUTY_MAX, PWM_DUTY_MAX, PWM_DUTY_MAX);
    _delay_ms(7000);

#ifdef SERIAL_USB_ACTIVE
    Serial.println("calibrate low, 12s wait");
#endif

    pwm_duty(PWM_DUTY_MIN, PWM_DUTY_MIN, PWM_DUTY_MIN, PWM_DUTY_MIN);
    _delay_ms(12000);

#ifdef SERIAL_USB_ACTIVE
    Serial.println("ESC's calibrated");
#endif
}

/**
    \brief Test the motors and ESCs to a user defined level then ramp back.
    \author Tom Darlison
    \param percentageOfMax Percentage of duty cycle to ramp up to and then
    back to 0.
*/
void rampTest(const float& percentageOfMax)
{
    #ifdef SERIAL_USB_ACTIVE
        Serial.print("Starting Ramp test up to ");
        Serial.print(percentageOfMax);
        Serial.println("% of duty cycle");
    #endif
    int max = (PWM_DUTY_MAX - PWM_DUTY_MIN) * percentageOfMax;
    for (int i = PWM_DUTY_MIN; i < (PWM_DUTY_MIN + max); i++) {
        pwm_duty(i, i, i, i);
        #ifdef SERIAL_USB_ACTIVE
            Serial.println(i);
        #endif
        _delay_ms(100);
    }
    for (int i = (max + PWM_DUTY_MIN); i > PWM_DUTY_MIN; i--) {
        pwm_duty(i, i, i, i);
        #ifdef SERIAL_USB_ACTIVE
            Serial.println(i);
        #endif
        _delay_ms(100);
    }
    #ifdef SERIAL_USB_ACTIVE
        Serial.println("Ramp test complete");
    #endif
}

/**
    \brief Kill procedure.
    \param ERROR_NUMBER Error code to call audable signal.
    
    Turns the motors off in a controlled manner and locks program in while
    loop.
*/
void rampDown(const uint8_t ERROR_NUMBER)
{
    #ifdef SERIAL_USB_ACTIVE
        Serial.println("Starting rampDown");
    #endif

    while (1)
    {
        if (OCR1A > PWM_DUTY_MIN) OCR1A -= 1;
        if (OCR1B > PWM_DUTY_MIN) OCR1B -= 1;
        if (OCR1C > PWM_DUTY_MIN) OCR1C -= 1;
        if (OCR3A > PWM_DUTY_MIN) OCR3A -= 1;
        _delay_ms(10);
        if((OCR1A == PWM_DUTY_MIN) && (OCR1B == PWM_DUTY_MIN)
            && (OCR1C == PWM_DUTY_MIN) && (OCR3A == PWM_DUTY_MIN))
        {
            break;
        }
    }
    #ifdef AUDIBLE_SAFTEY
        pinMode(A0, OUTPUT);
        pinMode(A3, OUTPUT);
        while(true)
        {
            switch (ERROR_NUMBER)
            {
                case 0: // default
                    digitalWrite(A3, LOW);
                    digitalWrite(A0, HIGH);
                    break;
                case 1: // 1 beep
                    digitalWrite(A3, LOW);
                    digitalWrite(A0, HIGH);
                    _delay_ms(50);
                    digitalWrite(A3, LOW);
                    digitalWrite(A0, LOW);
                    _delay_ms(950);
                    break;
                case 2: // 2 beeps
                    digitalWrite(A3, LOW);
                    digitalWrite(A0, HIGH);
                    _delay_ms(50);
                    digitalWrite(A3, LOW);
                    digitalWrite(A0, LOW);
                    _delay_ms(50);
                    digitalWrite(A3, LOW);
                    digitalWrite(A0, HIGH);
                    _delay_ms(50);
                    digitalWrite(A3, LOW);
                    digitalWrite(A0, LOW);
                    _delay_ms(850);
                    break;
                case 3: // 3 beeps
                    digitalWrite(A3, LOW);
                    digitalWrite(A0, HIGH);
                    _delay_ms(50);
                    digitalWrite(A3, LOW);
                    digitalWrite(A0, LOW);
                    _delay_ms(50);
                    digitalWrite(A3, LOW);
                    digitalWrite(A0, HIGH);
                    _delay_ms(50);
                    digitalWrite(A3, LOW);
                    digitalWrite(A0, LOW);
                    _delay_ms(50);
                    digitalWrite(A3, LOW);
                    digitalWrite(A0, HIGH);
                    _delay_ms(50);
                    digitalWrite(A3, LOW);
                    digitalWrite(A0, LOW);
                    _delay_ms(750);
                    break;
                default:
                    digitalWrite(A3, LOW);
                    digitalWrite(A0, HIGH);
                    break;
            }
        }
    #endif
}
