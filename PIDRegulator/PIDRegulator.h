/*
 * FileName: PIDRegulator.h
 * Description: PIDRegulator Class
 */

#ifndef PIDREGULATOR_H
#define PIDREGULATOR_H

class PIDRegulator
{
public:
    PIDRegulator();
    PIDRegulator(float Kp, float Ki, float Kd, float Kaw, float upper_limit_output, float lower_limit_output);
    float computePID(float reference, float feeddback);
		void setKp(float kp);
		void setKi(float ki);

private:
    float _kp, _ki, _kd, _kaw;
    float _upper_limit_output, _lower_limit_output;
    float integral;
    float pre_error;
    float output;
};
#endif

/*
 * Current Version: 1.0
 * Author: Fang Yun
 * Create Time: 2022/1/21
 */
