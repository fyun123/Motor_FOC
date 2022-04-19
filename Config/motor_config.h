#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#define R_PHASE 0.156f           //Ohms
#define L_D 0.000053f            //Henries
#define L_Q 0.000053f            //


#define KT .091f                 //N-m per peak phase amp, = WB*NPP*3/2
#define NPP 14               		//Number of pole pairs
#define GR 9.1f                 //Gear ratio
#define KT_OUT 0.32f            //KT*GR 0.091*6
#define WB 0.00167f              //Flux linkage, Webers.  
#define R_TH 1.25f              //Kelvin per watt
#define INV_M_TH 0.03125f       //Kelvin per joule



#endif
