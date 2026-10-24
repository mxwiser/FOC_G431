#ifndef FOC_MATH_H
#define FOC_MATH_H
#define PI 3.14159265358979323846

#define POLES 7
#define ENCODER_COUNT_MAX 2048
#define RESISTANCE (21.2f)
#define INDUCTANCE (0.0053f)

typedef struct 
{
    float input;
    float last_input;
    float integral;
    float kp;
    float ki;
    float output;
    float dt;
}PID;


typedef struct 
{
    float  theta;
    float  m_currents_adc[3];
    float  Ia;
    float  Ib;
    float  Ic;
    float  alpha;
    float  beta;
    float  max_mod;
    PID    dq[2];
}motor_state_t;


#ifdef __cplusplus
extern "C" {
#endif

void calculate_PID(PID *pid);
void clark(float *Ia,float *Ib,float *Ic,float *alpha,float *beta);
void park(float *x1,float *y1,float *theta,float *x2,float *y2);
void foc_loop(void);
void foc_init(void);

#ifdef __cplusplus
}
#endif

#endif
