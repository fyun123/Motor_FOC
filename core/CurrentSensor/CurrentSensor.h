/*
 * File Name: CurrentSensor.h
 * Description: CurrentSensor Class
 */

#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H


#include "hw_config.h"
class CurrentSensor
{
private:
    float _i_scale;
    int adc1_offset, adc2_offset, adc3_offset;
    float i_a, i_b, i_c;
    int adc1_raw, adc2_raw, adc3_raw;
    
public:
    CurrentSensor(float i_scale);
    void zeroCurrent();
    void currentSample();
    float getCurrentIa();
    float getCurrentIb();
    float getCurrentIc();
		int getAdc1Offset();
		int getAdc2Offset();
};

#endif

/*
 * Current Version: 1.0
 * Author: Fang Yun
 * Create Time: 2022/1/21
 */

