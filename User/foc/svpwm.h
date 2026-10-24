#ifndef SVPWM_H
#define SVPWM_H
#include <stdint.h>

// Constants
#define ONE_BY_SQRT3			(0.57735026919f)
#define TWO_BY_SQRT3			(2.0f * 0.57735026919f)
#define SQRT3_BY_2				(0.86602540378f)
#define COS_30_DEG				(0.86602540378f)
#define SIN_30_DEG				(0.5f)
#define COS_MINUS_30_DEG		(0.86602540378f)
#define SIN_MINUS_30_DEG		(-0.5f)
#define ONE_BY_SQRT2			(0.7071067811865475f)
#ifdef __cplusplus
extern "C" {
#endif

void foc_svm(float alpha, float beta, float max_mod, uint32_t PWMFullDutyCycle,
		uint32_t* tAout, uint32_t* tBout, uint32_t* tCout, uint32_t *svm_sector);
#ifdef __cplusplus
}
#endif

#endif
