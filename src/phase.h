#ifndef MINIPOD_PHASE_H
#define MINIPOD_PHASE_H

#include <oscillator-disciplining/oscillator-disciplining.h>

#include "algorithm_structs.h"
#include "utils.h"

float filter_phase(struct kalman_parameters *kalman, float phase, float estimated_drift, int interval);
int compute_phase_error_mean(struct algorithm_input *inputs, int length, float *mean_phase_error);
int compute_frequency_error(struct algorithm_input *inputs, int length, struct linear_func_param *func);

#endif /* MINIPOD_PHASE_H */
