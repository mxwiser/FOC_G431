#ifndef FOC_MATH_H
#define FOC_MATH_H
#define PI 3.14159265358979323846
#include "stdint.h"
#include "sensorless_config.hpp"
#define POLES FOC_POLE_PAIRS
#define ENCODER_COUNT_MAX 2048
#define FOC_PWM_PERIOD_TICKS 6800u
#define RESISTANCE (FOC_MOTOR_R)
#define INDUCTANCE (FOC_MOTOR_L)

typedef struct
{
    float input;         // Error: reference - feedback.
    float last_input;    // Error from the previous call.
    float integral;      // Persistent integral contribution to output.
    float kp;
    float ki;            // Integral gain per second; integration includes dt.
    float output;
    float dt;            // Sample period, seconds.
    float output_limit;  // Symmetric output limit (+/-); <= 0 disables output.
} PID;

#ifdef __cplusplus
extern "C" {
#endif

/* Positional PI with output limiting and anti-windup.
 * Zero-initialize once; set input, kp, ki, dt and output_limit, then read output.
 * Use a separate PID instance per axis; reset integral when restarting.
 * Nonpositive dt or output_limit clears integral, last_input and output.
 */
void calculate_PI(PID *pid);
void clark(float *Ia,float *Ib,float *Ic,float *alpha,float *beta);
void park(float *x1,float *y1,float *theta,float *x2,float *y2);
void calculate_foc(float Ia,float Ib,float Ic,float theta,uint32_t &tAout,uint32_t &tBout,uint32_t &tCout);
void calculate_Idq(float I_a, float I_b, float I_c,float,float &Id,float &Iq);
/* Input magnitude must be <= 1; 1 means the full linear SVPWM voltage range.
 * Smaller magnitudes retain their amplitude (do not normalize them to unit length).
 */
void foc_svm_normalized(float U_alpha,float U_beta, uint32_t &tA, uint32_t &tB, uint32_t &tC,uint32_t  &sector);

#ifdef __cplusplus
}
#endif

class Foc_motor{
    private:

        float I_a;      //current of phase A.
        float I_b;      //current of phase B.
        float I_c;      //current of phase C.
        float I_alpha;  //current of Alpha.
        float I_beta;   //current of Beta.
        float I_d;      //current of axis-d
        float I_q;      //current of axis-q
        float E_theta;  //angle of elecronic.
        float U_dc;     //Master Voltage.

        float target_I_d;
        float target_I_q;

        //[0,1] normalized
        float U_d;         
        float U_q;

    public:

        void init();
        void updateCurrent(float &I_a,float &I_b,float &I_c){
            this->I_a = I_a; this->I_b = I_b; this->I_c = I_c;
        };
        void updateUdc(float &udc){
            this->U_dc = udc;
        };
        void updateSvpwm(uint32_t &tA,uint32_t &tB,uint32_t &tC);
};


#endif
