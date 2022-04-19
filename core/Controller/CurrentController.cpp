/*
 * File Name: CurrentController.cpp
 * Description: Controller of Current Loop.
 */

#include "CurrentController.h"
#include "FastMath.h"
#include "math_ops.h"

using namespace FastMath;

/*
 * Call the constructor of the parent class.
 */
CurrentController::CurrentController(PIDRegulator pidRegulator) : Controller(pidRegulator) {};

/*
 * DQ0 Transform
 */
void CurrentController::dq0(float theta, float a, float b, float c, float *d, float *q)
{
    float cf = FastCos(theta);
    float sf = FastSin(theta);

    *d = 0.6666667f * (cf * a + (0.86602540378f * sf - .5f * cf) * b + (-0.86602540378f * sf - .5f * cf) * c); /// Faster DQ0 Transform
    *q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - .5f * sf) * b - (0.86602540378f * cf - .5f * sf) * c);
}

/*
 * inverse Park Transform
 */
void CurrentController:: inversePark(float theta, float d, float q, float *alpha, float *beta)
{
    float cf = FastCos(theta);
    float sf = FastSin(theta);
    *alpha = cf * d - sf * q;
    *beta  = sf * d + cf * q;
}


/*
 * svpwm
 */
void CurrentController::svpwm(float v_bus, float u_alpha, float u_beta, float *dtc_u, float *dtc_v, float *dtc_w)
{
    float u_ref1 = u_beta;
    float u_ref2 = 0.86602540378f * u_alpha - 0.5f * u_beta;
    float u_ref3 = -0.86602540378f * u_alpha - 0.5f * u_beta;
    int N = 0;
    float X = 1.7320508f * u_beta / v_bus;
    float Y = 1.7320508f * ( 0.86602540378f * u_alpha - 0.5f * u_beta) / v_bus;
    float Z = 1.7320508f * (-0.86602540378f * u_alpha - 0.5f * u_beta) / v_bus;
    float Tx = 0;
    float Ty = 0;
    float Txy_sum = 0;

    if(u_ref1 > 0)
    {
        N += 1;
    }

    if(u_ref2 > 0)
    {
        N += 2;
    }

    if(u_ref3 > 0)
    {
        N += 4;
    }

    switch(N)
    {
        case 1:
            Tx = -Y;
            Ty = -Z;
            Txy_sum = Tx + Ty;

            if (Txy_sum > 1)
            {
                Tx = Tx / Txy_sum;
                Ty = Ty / Txy_sum;
            }

            *dtc_v = (1 - Tx - Ty) * 0.25f;
            *dtc_u = *dtc_v + Tx * 0.5f;
            *dtc_w = *dtc_u + Ty * 0.5f;
            break;

        case 2:
            Tx = -Z;
            Ty = -X;
            Txy_sum = Tx + Ty;

            if (Txy_sum > 1)
            {
                Tx = Tx / Txy_sum;
                Ty = Ty / Txy_sum;
            }

            *dtc_u = (1 - Tx - Ty) * 0.25f;
            *dtc_w = *dtc_u + Tx * 0.5f;
            *dtc_v = *dtc_w + Ty * 0.5f;
            break;

        case 3:
            Tx = Y;
            Ty = X;
            Txy_sum = Tx + Ty;

            if (Txy_sum > 1)
            {
                Tx = Tx / Txy_sum;
                Ty = Ty / Txy_sum;
            }

            *dtc_u = (1 - Tx - Ty) * 0.25f;
            *dtc_v = *dtc_u + Tx * 0.5f;
            *dtc_w = *dtc_v + Ty * 0.5f;
            break;

        case 4:
            Tx = -X;
            Ty = -Y;
            Txy_sum = Tx + Ty;

            if (Txy_sum > 1)
            {
                Tx = Tx / Txy_sum;
                Ty = Ty / Txy_sum;
            }

            *dtc_w = (1 - Tx - Ty) * 0.25f;
            *dtc_v = *dtc_w + Tx * 0.5f;
            *dtc_u = *dtc_v + Ty * 0.5f;
            break;

        case 5:
            Tx = X;
            Ty = Z;
            Txy_sum = Tx + Ty;

            if (Txy_sum > 1)
            {
                Tx = Tx / Txy_sum;
                Ty = Ty / Txy_sum;
            }

            *dtc_v = (1 - Tx - Ty) * 0.25f;
            *dtc_w = *dtc_v + Tx  * 0.5f;
            *dtc_u = *dtc_w + Ty  * 0.5f;
            break;

        case 6:
            Tx = Z;
            Ty = Y;
            Txy_sum = Tx + Ty;

            if (Txy_sum > 1)
            {
                Tx = Tx / Txy_sum;
                Ty = Ty / Txy_sum;
            }

            *dtc_w = (1 - Tx - Ty) * 0.25f;
            *dtc_u = *dtc_w + Tx  * 0.5f;
            *dtc_v = *dtc_u + Ty  * 0.5f;
            break;

        default:
            break;
    }
}

/*
 * Initialize attribute of CurrentController
 */
void CurrentController::initController()
{
    TIM1->CCR1 = PWM_ARR >> 1;
    TIM1->CCR2 = PWM_ARR >> 1;
    TIM1->CCR3 = PWM_ARR >> 1;
    i_d = 0;
    i_q = 0;
    i_d_ref = 0;
    i_q_ref = 0;
    i_d_filt = 0;
    i_q_filt = 0;
    i_d_int = 0; // d Current error integrals
    i_q_int = 0; // q Current error integrals
    v_d = 0;
    v_q = 0;
    v_alpha = 0;
    v_beta = 0;
    virtual_theta_elec = 0;
    virtual_theta_elec_1 = 0;
}

/*
 * Set current reference of d-axis
 */
void CurrentController::setIdRef(float i_d_reference)
{
    i_d_ref = i_d_reference;
}

/*
 * Set current reference of q-axis
 */
void CurrentController::setIqRef(float i_q_reference)
{
    i_q_ref = i_q_reference;
}

/*
 * Open Current Loop Control
 */
void CurrentController::doOpenLoop(float v_d_open, float v_q_open, float delta_theta_elec)
{
    inversePark(virtual_theta_elec, v_d_open, v_q_open, &v_alpha,  &v_beta);
    svpwm(V_BUS, v_alpha, v_beta, &dtc_u, &dtc_v, &dtc_w);
    TIM1->CCR1 = (PWM_ARR << 1) * dtc_u;

    if (PHASE_ORDER)
    {
        TIM1->CCR2 = (PWM_ARR << 1) * dtc_v;
        TIM1->CCR3 = (PWM_ARR << 1) * dtc_w;
    }
    else
    {
        TIM1->CCR3 = (PWM_ARR << 1) * dtc_v;
        TIM1->CCR2 = (PWM_ARR << 1) * dtc_w;
    }

    virtual_theta_elec += delta_theta_elec;
}

/*
 * Half Close Current Loop Control. Current amplitude closed loop, phase open loop.
 */
void CurrentController::doHalfCloseLoop(CurrentSensor *cs, float delta_theta_elec)
{
    dq0(virtual_theta_elec_1, cs->getCurrentIa(), cs->getCurrentIb(), cs->getCurrentIc(), &i_d, &i_q);
    // id, iq filter
    i_d_filt = 0.95f * i_d_filt + 0.05f * i_d;
    i_q_filt = 0.95f * i_q_filt + 0.05f * i_q;

    v_d = _pidRegulator.computePID(i_d_ref, i_d_filt);
    v_q = _pidRegulator.computePID(i_q_ref, i_q_filt);

    inversePark(virtual_theta_elec_1, v_d, v_q, &v_alpha,  &v_beta);
    svpwm(V_BUS, v_alpha, v_beta, &dtc_u, &dtc_v, &dtc_w);
    TIM1->CCR1 = (PWM_ARR << 1) * dtc_u;

    if (PHASE_ORDER)
    {
        TIM1->CCR2 = (PWM_ARR << 1) * dtc_v;
        TIM1->CCR3 = (PWM_ARR << 1) * dtc_w;
    }
    else
    {
        TIM1->CCR3 = (PWM_ARR << 1) * dtc_v;
        TIM1->CCR2 = (PWM_ARR << 1) * dtc_w;
    }

    virtual_theta_elec_1 += delta_theta_elec;
}

/*
 * Close Current Loop Control.
 */
void CurrentController::doFoc(CurrentSensor *cs, float theta)
{
    dq0(theta, cs->getCurrentIa(), cs->getCurrentIb(), cs->getCurrentIc(), &i_d, &i_q);
    // id, iq filter
    i_d_filt = 0.95f * i_d_filt + 0.05f * i_d;
    i_q_filt = 0.95f * i_q_filt + 0.05f * i_q;

    // PI Controller //
    v_d = _pidRegulator.computePID(i_d_ref, i_d_filt);
    v_q = _pidRegulator.computePID(i_q_ref, i_q_filt);
    inversePark(theta, v_d, v_q, &v_alpha,  &v_beta);
    svpwm(V_BUS, v_alpha, v_beta, &dtc_u, &dtc_v, &dtc_w);
    limit(&dtc_u, DTC_MIN, DTC_MAX);
    limit(&dtc_v, DTC_MIN, DTC_MAX);
    limit(&dtc_w, DTC_MIN, DTC_MAX);
    TIM1->CCR1 = (PWM_ARR << 1) * dtc_u;

    if (PHASE_ORDER)
    {
        TIM1->CCR2 = (PWM_ARR << 1) * dtc_v;
        TIM1->CCR3 = (PWM_ARR << 1) * dtc_w;
    }
    else
    {
        TIM1->CCR3 = (PWM_ARR << 1) * dtc_v;
        TIM1->CCR2 = (PWM_ARR << 1) * dtc_w;
    }
}

float CurrentController :: getId()
{
    return i_d_ref;
}

float CurrentController :: getIq()
{
    return i_q_ref;
}

/*
 * Current Version: 1.0
 * Author: Fang Yun
 * Create Time: 2022/1/21
 */

