/*
 * File Name: CurrentController.h
 * Description: Inherit from Controller class
 */

#ifndef CURRENT_CONTROLLER_H
#define CURRENT_CONTROLLER_H

#include "mbed.h"
#include "Controller.h"
#include "CurrentSensor.h"
#include "current_controller_config.h"
#include "hw_config.h"
class CurrentController : public Controller
{
public:
    CurrentController(PIDRegulator pidRegulator);
    virtual void initController();
    void dq0(float theta, float a, float b, float c, float *d, float *q);
		void inversePark(float theta, float d, float q, float *alpha, float *beta);
		void svpwm(float v_bus, float u_alpha, float u_beta, float *dtc_u, float *dtc_v, float *dtc_w);
    void setIdRef(float i_d_reference);
    void setIqRef(float i_q_reference);
		void doOpenLoop(float v_d_open, float v_q_open,float delta_theta_elec);
		void doHalfCloseLoop(CurrentSensor *cs, float theta);
    void doFoc(CurrentSensor *cs, float theta);
		float getId();
		float getIq();
		int timeout;
private:
    float i_d_ref, i_q_ref;
    float i_d, i_q, i_d_filt, i_q_filt, i_d_int, i_q_int;
    float v_d, v_q, v_ref;
    float v_u, v_v, v_w;
		float v_alpha, v_beta;
    float dtc_u, dtc_v, dtc_w;
		float virtual_theta_elec, virtual_theta_elec_1;
};

#endif

/*
 * Current Version: 1.0
 * Author: Fang Yun
 * Create Time: 2022/1/21
 */
