#ifndef HW_SETUP_H
#define HW_SETUP_H

#include "mbed.h"
#include "FastPWM.h"
typedef struct{
    DigitalOut *enable;
    DigitalOut *led;
    FastPWM *pwm_u, *pwm_v, *pwm_w;
    } GPIOStruct;
   
void Init_PWM(GPIOStruct *gpio);
void Init_ADC(void);
void Init_DAC(void);
void Init_All_HW(GPIOStruct *gpio);

#endif
