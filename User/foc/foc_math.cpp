#include "foc_math.h"
#include <cmath>
#include "svpwm.h"
void clark(float *Ia,float *Ib,float *Ic,float *alpha,float *beta){
    *alpha= *Ia + std::cos(static_cast<float>(2*PI / 3))* (*Ib + *Ic);
    *beta = std::sin(static_cast<float>(2*PI / 3))*( *Ib - *Ic);
}
void park(float *x1,float *y1,float *theta,float *x2,float *y2){
    *x2= *x1 * std::cos(*theta) + *y1*std::sin(*theta);
    *y2= -*x1*std::sin(*theta) +  *y1 * std::cos(*theta);
}

void calculate_Idq(float I_a, float I_b, float I_c,float &Id,float &Iq){
     
}

void calculate_foc(float I_a, float I_b, float I_c,float theta, uint32_t &tAout, uint32_t &tBout, uint32_t &tCout)
{       float I_alpha,I_beta,I_d,I_q;
        clark(&I_a,&I_b,&I_c,&I_alpha,&I_beta);
        park(&I_alpha,&I_beta,&theta,&I_d,&I_q);
        float temp = -theta;
        float U_alpha,U_beta;
        I_q = 0.1;
        I_d = 0;
        park(&I_d,&I_q,&temp,&U_alpha,&U_beta);
        uint32_t sector;
        foc_svm_normalized(U_alpha,U_beta,tAout,tBout,tCout,sector);
        
}

void foc_svm_normalized(float U_alpha,float U_beta, uint32_t &tA, uint32_t &tB, uint32_t &tC,uint32_t &sector){
    foc_svm(U_alpha*1.5f*ONE_BY_SQRT3, U_beta*1.5f*ONE_BY_SQRT3, 1,FOC_PWM_PERIOD_TICKS,&tA, &tB, &tC, &sector);

}

float calculate_PI(float error, float kp, float ki, float dt,
                   float output_limit, float &integral){
    const float limit = output_limit;
    if (!(dt > 0.0f) || !(limit > 0.0f)) {
        integral = 0.0f;
        return 0.0f;
    }

    // Re-clamp saved state if the voltage/output limit has been reduced.
    if (integral > limit) integral = limit;
    if (integral < -limit) integral = -limit;

    const float proportional = kp * error;
    float candidate = integral + ki * error * dt;
    if (candidate > limit) candidate = limit;
    if (candidate < -limit) candidate = -limit;

    // Do not accumulate an error that would push further into saturation.
    // Opposite-sign error can still unwind the integral immediately.
    const float requested = proportional + candidate;
    if (!((requested > limit && error > 0.0f) ||
          (requested < -limit && error < 0.0f))) {
        integral = candidate;
    }

    float output = proportional + integral;
    if (output > limit) output = limit;
    if (output < -limit) output = -limit;
    return output;
}
