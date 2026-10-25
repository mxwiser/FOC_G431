#include "svpwm.h"
#include <math.h>
#include <stdio.h>

static inline void utils_truncate_number_int(int *number, int min, int max) {
	if (*number > max) {
		*number = max;
	} else if (*number < min) {
		*number = min;
	}
}



/**
 * @brief svm Space vector modulation. Magnitude must not be larger than sqrt(3)/2, or 0.866 to avoid overmodulation.
 *        See https://github.com/vedderb/bldc/pull/372#issuecomment-962499623 for a full description.
 * 模归一化后的矢量U_alpha*1.5f*ONE_BY_SQRT3
 * @param alpha voltage
 * @param beta Park transformed and normalized voltage
 * @param PWMFullDutyCycle is the peak value of the PWM counter.
 * @param tAout PWM duty cycle phase A (0 = off all of the time, PWMFullDutyCycle = on all of the time)
 * @param tBout PWM duty cycle phase B
 * @param tCout PWM duty cycle phase C
 */
static inline uint32_t svm_sector_for(float alpha, float beta) {
	uint32_t sector;

	if (beta >= 0.0f) {
		if (alpha >= 0.0f) {
			//quadrant I
			if (ONE_BY_SQRT3 * beta > alpha) {
				sector = 2;
			} else {
				sector = 1;
			}
		} else {
			//quadrant II
			if (-ONE_BY_SQRT3 * beta > alpha) {
				sector = 3;
			} else {
				sector = 2;
			}
		}
	} else {
		if (alpha >= 0.0f) {
			//quadrant IV5
			if (-ONE_BY_SQRT3 * beta > alpha) {
				sector = 5;
			} else {
				sector = 6;
			}
		} else {
			//quadrant III
			if (ONE_BY_SQRT3 * beta > alpha) {
				sector = 4;
			} else {
				sector = 5;
			}
		}
	}

	return sector;
}

void foc_svm(float alpha, float beta, float max_mod, uint32_t PWMFullDutyCycle,
				uint32_t* tAout, uint32_t* tBout, uint32_t* tCout, uint32_t *svm_sector) {
	const uint32_t sector = svm_sector_for(alpha, beta);

	// PWM timings
	int tA, tB, tC;

	switch (sector) {

	// sector 1-2
	case 1: {
		// Vector on-times
		int t1 = (alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		int t2 = (TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;

		// PWM timings
		tA = (PWMFullDutyCycle + t1 + t2) / 2;
		tB = tA - t1;
		tC = tB - t2;

		break;
	}

	// sector 2-3
	case 2: {
		// Vector on-times
		int t2 = (alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		int t3 = (-alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

		// PWM timings
		tB = (PWMFullDutyCycle + t2 + t3) / 2;
		tA = tB - t3;
		tC = tA - t2;

		break;
	}

	// sector 3-4
	case 3: {
		// Vector on-times
		int t3 = (TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;
		int t4 = (-alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

		// PWM timings
		tB = (PWMFullDutyCycle + t3 + t4) / 2;
		tC = tB - t3;
		tA = tC - t4;

		break;
	}

	// sector 4-5
	case 4: {
		// Vector on-times
		int t4 = (-alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		int t5 = (-TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;

		// PWM timings
		tC = (PWMFullDutyCycle + t4 + t5) / 2;
		tB = tC - t5;
		tA = tB - t4;

		break;
	}

	// sector 5-6
	case 5: {
		// Vector on-times
		int t5 = (-alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		int t6 = (alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

		// PWM timings
		tC = (PWMFullDutyCycle + t5 + t6) / 2;
		tA = tC - t5;
		tB = tA - t6;

		break;
	}

	// sector 6-1
	case 6: {
		// Vector on-times
		int t6 = (-TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;
		int t1 = (alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

		// PWM timings
		tA = (PWMFullDutyCycle + t6 + t1) / 2;
		tC = tA - t1;
		tB = tC - t6;

		break;
	}
	}

	int t_max = PWMFullDutyCycle * (1.0f - (1.0f - max_mod) * 0.5f);
	utils_truncate_number_int(&tA, 0, t_max);
	utils_truncate_number_int(&tB, 0, t_max);
	utils_truncate_number_int(&tC, 0, t_max);

	*tAout = tA;
	*tBout = tB;
	*tCout = tC;
	*svm_sector = sector;
}


void svpwm_fast(float alpha, float beta, float max_mod, uint32_t PWMFullDutyCycle,
        uint32_t* tAout, uint32_t* tBout, uint32_t* tCout)
{
    float a,b,c,al,be,max,min,v0;
    al = alpha * 1.33333333f;
    be = beta * 1.15470053838f;

    a = al;
    b = -(al / 2.0f) + be;
    c = -(al / 2.0f) - be;

    // ARMCC5's default C++ math headers do not expose C99 fmaxf/fminf.
    max = (a > b) ? a : b;
    if (c > max) { max = c; }
    min = (a < b) ? a : b;
    if (c < min) { min = c; }

    // Center in [-1, 1]; the duty conversion below adds 0.5 exactly once.
    v0 = -(max + min) * 0.5f;
    a = a + v0;
    b = b + v0;
    c = c + v0;

    // Match foc_svm's tick units and upper limit, including overmodulation.
    int ta = static_cast<int>((a * 0.5f + 0.5f) * PWMFullDutyCycle);
    int tb = static_cast<int>((b * 0.5f + 0.5f) * PWMFullDutyCycle);
    int tc = static_cast<int>((c * 0.5f + 0.5f) * PWMFullDutyCycle);
    const int t_max = PWMFullDutyCycle * (1.0f - (1.0f - max_mod) * 0.5f);
    utils_truncate_number_int(&ta, 0, t_max);
    utils_truncate_number_int(&tb, 0, t_max);
    utils_truncate_number_int(&tc, 0, t_max);

    *tAout = ta;
    *tBout = tb;
    *tCout = tc;
    
}
