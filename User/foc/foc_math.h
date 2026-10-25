#ifndef FOC_MATH_H
#define FOC_MATH_H
#define PI 3.14159265358979323846
#include "stdint.h"
#define POLES 7
#define ENCODER_COUNT_MAX 2048
#define FOC_PWM_PERIOD_TICKS 3400u
#define RESISTANCE (21.2f)
#define INDUCTANCE (0.0053f)

#ifdef __cplusplus
extern "C" {
#endif

/* Positional PI with output limiting and anti-windup; returns the output.
 * error = reference - feedback. dt is seconds; ki is an integral gain per second.
 * output_limit is the symmetric limit (+/-), in the same units as the output.
 * integral is persistent state: initialize/reset to 0; use one variable per axis.
 * Nonpositive dt or output_limit clears integral and returns 0.
 */
float calculate_PI(float error, float kp, float ki, float dt,
                   float output_limit, float &integral);
void clark(float *Ia,float *Ib,float *Ic,float *alpha,float *beta);
void park(float *x1,float *y1,float *theta,float *x2,float *y2);
void calculate_foc(float Ia,float Ib,float Ic,float theta,uint32_t &tAout,uint32_t &tBout,uint32_t &tCout);
void calculate_Idq(float I_a, float I_b, float I_c,float &Id,float &Iq);
/* Input magnitude must be <= 1; 1 means the full linear SVPWM voltage range.
 * Smaller magnitudes retain their amplitude (do not normalize them to unit length).
 */
void foc_svm_normalized(float U_alpha,float U_beta, uint32_t &tA, uint32_t &tB, uint32_t &tC,uint32_t  &sector);

#ifdef __cplusplus
}
#endif

#endif
