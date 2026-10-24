#include "foc_math.h"
#include <cmath>
void clark(float *Ia,float *Ib,float *Ic,float *alpha,float *beta){
    *alpha= *Ia + std::cos(static_cast<float>(2*PI / 3))* (*Ib + *Ic);
    *beta = std::sin(static_cast<float>(2*PI / 3))*( *Ib - *Ic);
}
void park(float *x1,float *y1,float *theta,float *x2,float *y2){
    *x2= *x1 * std::cos(*theta) + *y1*std::sin(*theta);
    *y2= -*x1*std::sin(*theta) +  *y1 * std::cos(*theta);
}

motor_state_t motor;
void foc_init(){
   motor = motor_state_t();
}

void foc_loop(){

}
void calculate_PID(PID *pid){

}
