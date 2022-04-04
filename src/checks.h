#ifndef MINIPOD_CHECKS_H
#define MINIPOD_CHECKS_H

#include <oscillator-disciplining/oscillator-disciplining.h>

#include "algorithm_structs.h"

bool check_gnss_valid_over_cycle(struct od_input *inputs, int length);
bool check_lock_over_cycle(struct od_input *inputs, int length);
bool check_max_drift(struct od_input *inputs, int length);
bool check_no_outlier(struct od_input *inputs, int length, float mean_phase_error, int ref_fluctuation_ns);

#endif /* MINIPOD_CHECKS_H */
