/*
 * File Name: Calibration.cpp
 * Description: including checks phase order, open loop calibrate and close loop calibrate
 */
#include "mbed.h"
#include "calibration.h"
#include "user_config.h"
#include "motor_config.h"
#include "FastMath.h"

/*
 * Initialize attribute of class Calibration
 */
Calibration::Calibration(float i_d, float v_d)
{
    _i_d = i_d;
    _i_q = 0.0f;
    _v_d = v_d;
    _v_q = 0.0f;
}

/*
* Checks phase order, to ensure that positive Q current produces
*/
void Calibration ::order_phases(PositionSensor *ps, CurrentController *cc)
{
    printf("\r\n Checking phase ordering\r\n");
    float theta_ref = 0;
		float theta_start = 0;;
    float theta_actual = 0;
    float u_alpha, u_beta = 0;
    float dtc_u, dtc_v, dtc_w = 0.5f;
    int sample_counter = 0;

    // Set voltage angle to zero, wait for rotor position to settle
//    cc->abc(theta_ref, _v_d, _v_q, &v_u, &v_v, &v_w); // inverse dq0 transform on voltages
//    cc->svm(v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);   // space vector modulation
		cc->inversePark(theta_ref, _v_d, _v_q, &u_alpha,  &u_beta);
		cc->svpwm(V_BUS, u_alpha, u_beta, &dtc_u, &dtc_v, &dtc_w); 
    for (int i = 0; i < 20000; i++)
    {
        TIM1->CCR1 = (PWM_ARR << 1) * dtc_u; // Set duty cycles
        TIM1->CCR2 = (PWM_ARR << 1) * dtc_v;
        TIM1->CCR3 = (PWM_ARR << 1) * dtc_w;
        wait_us(100);
    }
		// sample position sensor
    ps->Sample(DT);
    wait_us(1000);
    // Rotate voltage angle, rotate for 2 electrical cycles
    while (theta_ref < 4 * PI)
    {
//        cc->abc(theta_ref, _v_d, _v_q, &v_u, &v_v, &v_w); // inverse dq0 transform on voltages
//        cc->svm(v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);   // space vector modulation
				cc->inversePark(theta_ref, _v_d, _v_q, &u_alpha,  &u_beta);
				cc->svpwm(V_BUS, u_alpha, u_beta, &dtc_u, &dtc_v, &dtc_w); 
        wait_us(100);
				// Set duty cycles
        TIM1->CCR1 = (PWM_ARR << 1) * dtc_u; 
        TIM1->CCR2 = (PWM_ARR << 1) * dtc_v;
        TIM1->CCR3 = (PWM_ARR << 1) * dtc_w;
        ps->Sample(DT); 
        theta_actual = ps->GetMechPositionFixed();
        if (theta_ref == 0)
        {
            theta_start = theta_actual;
        }
				// Once print every 200 cycles.
        if (sample_counter > 200)
        {
            sample_counter = 0;
            printf("%.4f   %.4f\r\n", theta_ref / (NPP), theta_actual);
        }
        sample_counter++;
        theta_ref += 0.001f;
    }
    float theta_end = ps->GetMechPositionFixed();
    int direction = (theta_end - theta_start) > 0;
    printf("Theta Start:%f\t\tTheta End:%f\r\n", theta_start, theta_end);
    printf("Direction:%d\r\n", direction);
    if (direction)
    {
        printf("Phasing correct\r\n");
    }
    else if (!direction)
    {
        printf("Phasing incorrect.Swapping phases V and W\r\n");
    }
    PHASE_ORDER = direction;
}

/*
 * Calibrate electric offset in current close loop.
 */
void Calibration::closeLoopCalibrate(CurrentSensor *cs, PositionSensor *ps, CurrentController *cc)
{
    
    float id, iq = 0;
		float v_d, v_q = 0;
    float u_alpha = 0;
    float u_beta = 0;
    float dtc_u = 0;
    float dtc_v = 0;
    float dtc_w = 0;
    for (int i = 0; i < 400; i++)
    {
				// current sample
        cs->currentSample();
        cc->dq0(0.0, cs->getCurrentIa(), cs->getCurrentIb(), cs->getCurrentIc(), &id, &iq);
        v_d = cc->_pidRegulator.computePID(_i_d, id);
        v_q = cc->_pidRegulator.computePID(_i_q, iq);
        limit_norm(&v_d, &v_q, V_BUS);
				cc->inversePark(0.0, v_d, v_q, &u_alpha,  &u_beta);
				cc->svpwm(V_BUS, u_alpha, u_beta, &dtc_u, &dtc_v, &dtc_w); 
        TIM1->CCR1 = (PWM_ARR << 1) * dtc_u;
				if(PHASE_ORDER)
				{
					TIM1->CCR2 = (PWM_ARR << 1) * dtc_v;
					TIM1->CCR3 = (PWM_ARR << 1) * dtc_w;
				}
				else
				{
					TIM1->CCR3 = (PWM_ARR << 1) * dtc_v;
					TIM1->CCR2 = (PWM_ARR << 1) * dtc_w;
				}
        wait(100);
    }
    ps->Sample(DT);
		elec_offset = fmod(ps->GetMechPosition()*NPP, 2*PI);
		ps->SetElecOffset(elec_offset);
}

/*
 * Calibrate electric offset in current open loop.
 */
void Calibration::openLoopCalibrate(CurrentController *cc, PositionSensor *ps, PreferenceWriter *prefs)
{
		/// Measures the electrical angle offset of the position sensor
    /// and (in the future) corrects nonlinearity due to position sensor eccentricity
		printf("Starting calibration procedure\r\n");
		float u_alpha, u_beta = 0;
    float * error_f;
    float * error_b;
    int * lut;
    int * raw_f;
    int * raw_b;
    float * error;
    float * error_filt;
    
    const int n = 128*NPP;                            // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
    const int n2 = 40;                                  // increments between saved samples (for smoothing motion)
    float delta = 2*PI*NPP/(n*n2);                     // change in angle between samples
    error_f = new float[n]();                           // error vector rotating forwards
    error_b = new float[n]();                            // error vector rotating backwards
    const int  n_lut = 128;
    lut = new int[n_lut]();                             // clear any old lookup table before starting.
    
    error = new float[n]();
    const int window = 128;
    error_filt = new float[n]();
    
    ps->WriteLUT(lut); 
    raw_f = new int[n]();
    raw_b = new int[n]();
    float theta_ref = 0;
    float theta_actual = 0;
    float dtc_u, dtc_v, dtc_w = .5f;
    
        
    ///Set voltage angle to zero, wait for rotor position to settle
//       abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);
//       svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);
    cc->inversePark(theta_ref, _v_d, _v_q, &u_alpha,  &u_beta);
		cc->svpwm(V_BUS, u_alpha, u_beta, &dtc_u, &dtc_v, &dtc_w); 
    for(int i = 0; i<40000; i++){
        TIM1->CCR1 = (PWM_ARR << 1) * dtc_u;                                        // Set duty cycles
        if(PHASE_ORDER){                                   
            TIM1->CCR2 = (PWM_ARR << 1) * dtc_v;
            TIM1->CCR3 = (PWM_ARR << 1) * dtc_w;
            }
        else{
            TIM1->CCR3 = (PWM_ARR << 1) * dtc_v;
            TIM1->CCR2 = (PWM_ARR << 1) * dtc_w;
            }
        wait_us(100);
        }
    ps->Sample(DT);   
    printf(" Current Angle : Rotor Angle : Raw Encoder \r\n\r\n");
    for(int i = 0; i<n; i++){                                                   // rotate forwards
       for(int j = 0; j<n2; j++){   
					theta_ref += delta;
//       abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);
//       svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);
					cc->inversePark(theta_ref, _v_d, _v_q, &u_alpha,  &u_beta);
					cc->svpwm(V_BUS, u_alpha, u_beta, &dtc_u, &dtc_v, &dtc_w); 
					TIM1->CCR1 = (PWM_ARR << 1) * dtc_u;
					if(PHASE_ORDER){                                                        // Check phase ordering
							TIM1->CCR2 = (PWM_ARR << 1) * dtc_v;                                    // Set duty cycles
							TIM1->CCR3 = (PWM_ARR << 1) * dtc_w;
					}
					else{
							TIM1->CCR3 = (PWM_ARR << 1) * dtc_v;
							TIM1->CCR2 = (PWM_ARR << 1) * dtc_w;
					}
					wait_us(100);
					ps->Sample(DT);
			}
			ps->Sample(DT);
			theta_actual = ps->GetMechPositionFixed();
			error_f[i] = theta_ref/NPP - theta_actual;
			raw_f[i] = ps->GetRawPosition();
			printf("%.4f\t   %.4f\t    %d\r\n", theta_ref/(NPP), theta_actual, raw_f[i]);
       //theta_ref += delta;
    }
    
    for(int i = 0; i<n; i++){                                                   // rotate backwards
       for(int j = 0; j<n2; j++){
					theta_ref -= delta;
//       abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);
//       svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);
					cc->inversePark(theta_ref, _v_d, _v_q, &u_alpha,  &u_beta);
					cc->svpwm(V_BUS, u_alpha, u_beta, &dtc_u, &dtc_v, &dtc_w); 
					TIM1->CCR1 = (PWM_ARR << 1) * dtc_u;
					if(PHASE_ORDER){                                                        // Check phase ordering
							TIM1->CCR2 = (PWM_ARR << 1) * dtc_v;                                    // Set duty cycles
							TIM1->CCR3 = (PWM_ARR << 1) * dtc_w;
					}
					else{
							TIM1->CCR3 = (PWM_ARR << 1) * dtc_v;
							TIM1->CCR2 = (PWM_ARR << 1) * dtc_w;
					}
					wait_us(100);
					ps->Sample(DT);
			}
			 ps->Sample(DT);                                                            // sample position sensor
			 theta_actual = ps->GetMechPositionFixed();                                    // get mechanical position
			 error_b[i] = theta_ref/NPP - theta_actual;
			 raw_b[i] = ps->GetRawPosition();
			 printf("%.4f\t   %.4f\t    %d\r\n", theta_ref/(NPP), theta_actual, raw_b[i]);
				 //theta_ref -= delta;
    }    
        
		float offset = 0;                                  
		for(int i = 0; i<n; i++){
				offset += (error_f[i] + error_b[n-1-i])/(2.0f*n);                   // calclate average position sensor offset
		}
		offset = fmod(offset*NPP, 2*PI);                                        // convert mechanical angle to electrical angle
		
				
		ps->SetElecOffset(offset);                                              // Set position sensor offset
		__float_reg[0] = offset;
		E_OFFSET = offset;
        
		/// Perform filtering to linearize position sensor eccentricity
		/// FIR n-sample average, where n = number of samples in one electrical cycle
		/// This filter has zero gain at electrical frequency and all integer multiples
		/// So cogging effects should be completely filtered out.

        
        float mean = 0;
        for (int i = 0; i<n; i++){                                              //Average the forward and back directions
            error[i] = 0.5f*(error_f[i] + error_b[n-i-1]);
            }
        for (int i = 0; i<n; i++){
            for(int j = 0; j<window; j++){
                int ind = -window/2 + j + i;                                    // Indexes from -window/2 to + window/2
                if(ind<0){
                    ind += n;}                                                  // Moving average wraps around
                else if(ind > n-1) {
                    ind -= n;}
                error_filt[i] += error[ind]/(float)window;
                }
            //printf("%.4f   %4f    %.4f   %.4f\r\n", error[i], error_filt[i], error_f[i], error_b[i]);
            mean += error_filt[i]/n;
            }
        int raw_offset = (raw_f[0] + raw_b[n-1])/2;                             //Insensitive to errors in this direction, so 2 points is plenty
        
        
        printf("\r\n Encoder non-linearity compensation table\r\n");
        printf(" Sample Number : Lookup Index : Lookup Value\r\n\r\n");
        for (int i = 0; i<n_lut; i++){                                          // build lookup table
            int ind = (raw_offset>>7) + i;
            if(ind > (n_lut-1)){ 
                ind -= n_lut;
                }
            lut[ind] = (int) ((error_filt[i*NPP] - mean)*(float)(ps->GetCPR())/(2.0f*PI));
            printf("%d\t   %d\t   %d \r\n", i, ind, lut[ind]);
            wait(.001);
            }
            
        ps->WriteLUT(lut);                                                      // write lookup table to position sensor object
        //memcpy(controller->cogging, cogging_current, sizeof(controller->cogging));  //compensation doesn't actually work yet....
        memcpy(&ENCODER_LUT, lut, 128*4);                                 // copy the lookup table to the flash array
        printf("\r\nEncoder Electrical Offset (rad) %f\r\n",  offset);
        
        if (!prefs->ready()) prefs->open();
        prefs->flush();                                                         // write offset and lookup table to flash
        prefs->close();
        
				delete[] error_filt;
				delete[] error;		
        delete[] error_f;       //gotta free up that ram
        delete[] error_b;
        delete[] lut;
        delete[] raw_f;
        delete[] raw_b;

}


/*
 * Get electric offset
 */
float Calibration::getElecOffset()
{
    return elec_offset;
}
/*
 * Current Version: 1.0
 * Author: Fang Yun
 * Create Time: 2022/1/21
 */

