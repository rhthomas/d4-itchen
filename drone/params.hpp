/**
    \file params.hpp
    \date 2017-03-11
    \author Tom Darlison
    \author Adam Melvin
    \brief Control parameters class.
*/

#ifndef _PARAMS_H_
#define _PARAMS_H_

#define INTERGRAL_MAX   100
#define SAMPLE_RATE     10 // Milliseconds

/**
    \class Parameters
    \brief Class containing control parameters and PID update functions.
    \author Tom Darlison
    \author Adam Melvin
    \date 2017-03-08
*/
class Parameters
{
    public: // methods
        Parameters(const bool is_yaw);
        ~Parameters();
        void updateOutput();
        void updateElev();
        void updateInput(const float& newInput);
        void updateActual(const float& newActual);
        void setPID(const float& P, const float& I, const float& D);
    public: // members
        float output;
    private: // methods
    private: // members
        bool is_Yaw;
        float error;
        float error_sum;
        float d_error;
        float input;
        float actual;
        float lastError;
        float K_P;
        float K_I;
        float K_D;
        int yaw_accum;
};

#endif // _PARAMS_H_
