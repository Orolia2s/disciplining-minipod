#ifndef MINIPOD_PHASE_H
#define MINIPOD_PHASE_H

#include <oscillator-disciplining/oscillator-disciplining.h>

#include "algorithm_structs.h"

float filter_phase(struct kalman_parameters *kalman, float phase, float estimated_drift, int interval);
int compute_phase_error_mean(struct od_input *inputs, int length, float *mean_phase_error);

#endif /* MINIPOD_PHASE_H */
