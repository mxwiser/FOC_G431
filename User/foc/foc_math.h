#ifndef FOC_MATH_H
#define FOC_MATH_H
#define PI 3.14159265358979323846
#include "stdint.h"
#define POLES 7
#define ENCODER_COUNT_MAX 2048
#define RESISTANCE (21.2f)
#define INDUCTANCE (0.0053f)

typedef struct 
{
    float input;         // Error: reference - feedback.
    float last_input;    // Error from the previous call.
    float integral;      // Accumulated integral contribution to output.
    float kp;
    float ki;            // Integral gain per second; calculate_PID multiplies by dt.
    float output;
    float dt;            // Sample period in seconds, e.g. 0.00004f at 25 kHz.
    float output_limit;  // Required symmetric limit (+/-); <= 0 disables output.
}PID;






#ifdef __cplusplus
extern "C" {
#endif

/* Positional PI controller (no derivative term). Zero-initialize before use.
 * Set input, kp, ki, dt and output_limit; read output after each call.
 * Reset integral and last_input to zero when restarting the controller.
 */
void calculate_PID(PID *pid);
void clark(float *Ia,float *Ib,float *Ic,float *alpha,float *beta);
void park(float *x1,float *y1,float *theta,float *x2,float *y2);
void calculate_foc(float Ia,float Ib,float Ic,float theta,uint32_t &tAout,uint32_t &tBout,uint32_t &tCout);
void calculate_Idq(float I_a, float I_b, float I_c,float &Id,float &Iq);
void foc_svm_normalized(float U_alpha,float U_beta, uint32_t &tA, uint32_t &tB, uint32_t &tC,uint32_t  &sector);

#ifdef __cplusplus
}
#endif

#endif
