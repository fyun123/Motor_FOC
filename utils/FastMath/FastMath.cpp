#include "FastMath.h"
#include "LUT.h"

const float Multiplier = 81.4873308631f;
float angle_LUT[16] = {0.785398163397448,	0.463647609000806,	0.244978663126864,	0.124354994546761,
0.0624188099959574,	0.0312398334302683,	0.0156237286204768,	0.00781234106010111,
0.00390623013196697,	0.00195312251647882,	0.000976562189559320,	0.000488281211194898,
0.000244140620149362,	0.000122070311893670,	0.0000610351561742088,	0.0000305175781155261};
float FastMath::FastSin(float theta){
    while (theta < 0.0f) theta += 6.28318530718f;
    while (theta >= 6.28318530718f) theta -= 6.28318530718f;    
    return SinTable[(int) (Multiplier*theta)] ;
    }
    
float FastMath::FastCos(float theta){
    return FastSin(1.57079632679f - theta);
    }
float FastMath::FastAtan(float tan){
  float x = 1.0;
	float y = tan;
	float angle_accumulate = 0;
	float radian_out = 0;
	float phase_shift = 0;
	float d = 1.0;
	float x_temp = x;
	float y_temp = y;
	float angle_accumulate_temp = angle_accumulate;
	if(x!=0 || y!=0){
		if(x > 0){
			phase_shift = 0;
		}else if(y < 0){
			phase_shift = -PI;
		}else{
			phase_shift = PI;
		}
		for(int i = 0; i < 16; i++){
			if(y < 0){
				d = -1.0;
			}else{
				d = 1.0;
			}
			x = x_temp + d * y_temp/(1<<i);
			y = y_temp - d * x_temp/(1<<i);
			angle_accumulate = angle_accumulate_temp + d * angle_LUT[i];
			radian_out = angle_accumulate + phase_shift;
			x_temp = x;
			y_temp = y;
			angle_accumulate_temp = angle_accumulate;
		}
	}
	return radian_out;	
}
