/*
 * File Name: CurrentSensor.cpp
 * Description: Current senssor sampling
 */
#include "mbed.h"
#include "CurrentSensor.h"
#include "user_config.h"

CurrentSensor::CurrentSensor(float i_scale)
{
    _i_scale = i_scale;
}

/*
 * Calibrate the zero current of the current sensor
 */
void CurrentSensor::zeroCurrent()
{
    adc1_offset = 0;
    adc2_offset = 0;
    int n = 1024;

    // Average n samples of the ADC
    for (int i = 0; i < n; i++)
    {
        TIM1->CCR1 = PWM_ARR >> 1;
        TIM1->CCR2 = PWM_ARR >> 1;
        TIM1->CCR3 = PWM_ARR >> 1;
        // Begin sample and conversion
        ADC1->CR2  |= 0x40000000;
        wait(.001);
        adc2_offset += ADC2->DR;
        adc1_offset += ADC1->DR;
    }

    adc1_offset = adc1_offset / n;
    adc2_offset = adc2_offset / n;
}

/*
 * Current sampling
 */
void CurrentSensor::currentSample()
{
    adc1_raw = ADC1->DR;
    adc2_raw = ADC2->DR;
    adc3_raw = ADC3->DR;

    if(PHASE_ORDER)                                                                           // Check current sensor ordering
    {
        i_b = _i_scale * (float)(adc2_raw - adc2_offset);  // Calculate phase currents from ADC readings
        i_c = _i_scale * (float)(adc1_raw - adc1_offset);
    }
    else
    {
        i_b = _i_scale * (float)(adc1_raw - adc1_offset);
        i_c = _i_scale * (float)(adc2_raw - adc2_offset);
    }

    i_a = -i_b - i_c;
}

/*
 * Get current of phase A
 */
float CurrentSensor::getCurrentIa()
{
    return i_a;
}

/*
 * Get current of phase B
 */
float CurrentSensor::getCurrentIb()
{
    return i_b;
}

/*
 * Get current of phase C
 */
float CurrentSensor::getCurrentIc()
{
    return i_c;
}

int CurrentSensor :: getAdc1Offset()
{
    return adc1_offset;
}

int CurrentSensor :: getAdc2Offset()
{
    return adc2_offset;
}
/*
 * Current Version: 1.0
 * Author: Fang Yun
 * Create Time: 2022/1/21
 */
