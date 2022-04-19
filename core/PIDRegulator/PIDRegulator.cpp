/*
 * FileName: PIDRegulator.cpp
 * Description: Compute the PI(D) output for a PI(D) regulator
 */

#include "PIDRegulator.h"
PIDRegulator::PIDRegulator(){};
PIDRegulator::PIDRegulator(float Kp, float Ki, float Kd, float Kaw, float upper_limit_output, float lower_limit_output)
{
    _kp = Kp;
    _ki = Ki;
    _kd = Kd;
    _kaw = Kaw;
    _upper_limit_output = upper_limit_output;
    _lower_limit_output = lower_limit_output;
}

float PIDRegulator::computePID(float reference, float feeddback)
{
    float cur_error, proportional_term, houtput_32;
    float wtemp, differential_term;

    // error computation
    cur_error = reference - feeddback;  // delta_e

    // Proportional term computation
    proportional_term = _kp * cur_error; // wP = Kp * delta_e

    // Integral term computation
    if (_ki == 0)
    {
        integral = 0;
    }
    else
    {
        integral += _ki * cur_error;    // wI = Ki * delta_e
    }

    wtemp = cur_error - pre_error;      // error between current error and last error
    differential_term = _kd * wtemp;    // wD = Kd * delta_d
    pre_error = cur_error;              // update last error

    houtput_32 = proportional_term + integral + differential_term; 
    // Anti Integral Saturation
    if (houtput_32 >= _upper_limit_output)
    {
        integral += (_upper_limit_output - houtput_32) * _kaw;
    }
    else
		{ 
			if (houtput_32 <= _lower_limit_output)
			{
					integral += (_lower_limit_output - houtput_32) * _kaw;
			}
    
    }
    output = proportional_term + integral + differential_term; 
    // limit output
    if (output >= _upper_limit_output)
    {
        output = _upper_limit_output;
    }
    else 
    {
			if (output <= _lower_limit_output)
        output = _lower_limit_output;
    }
    return output;
}

void PIDRegulator ::setKp(float kp)
{
	_kp = kp;
}

void PIDRegulator ::setKi(float ki)
{
	_ki = ki;
}

/*
 * Current Version: 1.0
 * Author: Fang Yun
 * Create Time: 2022/1/21
 */
