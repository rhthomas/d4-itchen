/**
    \file params.cpp
    \date 2017-03-11
    \author Tom Darlison
    \author Adam Melvin
    \brief Control parameters class.
*/

#include "params.hpp"

extern Parameters elevation;

/**
    \brief Constructor.
    \author Tom Darlison
    \param is_yaw_read Sets parameter to true if object is yaw related.
*/
Parameters::Parameters(const bool is_yaw_read)
    : output(0), error(0), error_sum(0), d_error(0), input(0),
      actual(0), lastError(0), K_P(0), K_I(0), K_D(0), yaw_accum(0)
{
    is_Yaw = is_yaw_read;
}

/**
    \brief Destructor.
    \author Tom Darlison
*/
Parameters::~Parameters()
{/* left blank */}

/**
    \brief Sets PID weighting values.
    \author Tom Darlison
    \param P Proportional control constant.
    \param I Integral control constant.
    \param D Differential control constant
*/
void Parameters::setPID(const float& P, const float& I, const float& D)
{
    float sample_rate_sec = SAMPLE_RATE * 0.001;
    K_P = P;
    K_I = I * sample_rate_sec;
    K_D = D / sample_rate_sec;
}

/**
    \brief Calculates PID error value.
    \author Tom Darlison
    \author Adam Melvin
*/
void Parameters::updateOutput()
{
    if(is_Yaw)
    {
        yaw_accum += input - 127;
        yaw_accum >  180 ? yaw_accum -= 360 : yaw_accum;
        yaw_accum < -180 ? yaw_accum += 360 : yaw_accum;
        error = yaw_accum - actual * 12;
    }
    else
    {
        error = (input - 127) - (actual * 17);
    }

    if(((error > 50) || (error < -50)) && (elevation.output > 2600))
    {
        error_sum = 0;
    }
    else
    {
        error_sum += K_I * error;
        error_sum > INTERGRAL_MAX ? error_sum = INTERGRAL_MAX :
            (error_sum < -INTERGRAL_MAX ? error_sum = -INTERGRAL_MAX : error_sum);
    }

    d_error = error - lastError;

    output = K_P * error + error_sum + K_D * d_error;

    lastError = error;
}

/**
    \brief Calculates elevation thrust value.
    \author Tom Darlison
    \auhtor Adam Melvin
*/
void Parameters::updateElev()
{
    output = input * 5.470588 + 2500;
}

/**
    \brief Updates user input.
    \author Tom Darlison
    \param newInput Sets input in class.
*/
void Parameters::updateInput(const float& newInput)
{
    input = newInput;
}

/**
    \brief Updates actual value.
    \author Tom Darlison
    \param newActual Sets MPU data in class.
*/
void Parameters::updateActual(const float& newActual)
{
    actual = newActual;
}
