#include <math.h>

#include "checks.h"

#include "algorithm_structs.h"
#include "log.h"

#define GNSS_INVALID_MAX_STREAK 10

enum gnss_state check_gnss_valid_over_cycle(struct algorithm_input *inputs, int length)
{
	int i;
	int gnss_invalid_counter = 0;
	int gnss_invalid_max_streak = 0;
	int gnss_invalid_current_streak = 0;
	bool previous_gnss_state;

	if (inputs == NULL)
		return GNSS_KO;
	if (length <= 0)
		return GNSS_KO;

	if (!inputs[0].valid) {
		gnss_invalid_counter++;
		gnss_invalid_current_streak = 1;
		gnss_invalid_max_streak = 1;
	}
	previous_gnss_state = inputs[0].valid;

	for (i = 1; i < length; i ++) {
		if (!inputs[i].valid) {
			gnss_invalid_counter++;
			if (!previous_gnss_state) {
				gnss_invalid_current_streak++;
			} else {
				gnss_invalid_current_streak = 1;
			}
		}
		if (gnss_invalid_max_streak < gnss_invalid_current_streak) {
			gnss_invalid_max_streak = gnss_invalid_current_streak;
		}
		previous_gnss_state = inputs[i].valid;
	}

	log_trace("check_gnss_valid_over_cycle: gnss_invalid_counter: %d, gnss_invalid_max_streak: %d, ", gnss_invalid_counter, gnss_invalid_max_streak);

	if (gnss_invalid_counter == 0)
		return GNSS_OK;
	else if (gnss_invalid_counter < round(length / 3) && gnss_invalid_max_streak < GNSS_INVALID_MAX_STREAK)
		return GNSS_UNSTABLE;
	else
		return GNSS_KO;
}

bool check_lock_over_cycle(struct algorithm_input *inputs, int length)
{
	bool lock_valid = true;
	int i;

	if (inputs == NULL)
		return false;
	if (length <= 0)
		return false;

	for (i = 0; i < length; i ++)
		lock_valid = lock_valid & inputs[i].lock;
	return lock_valid;
}

bool check_max_drift(struct algorithm_input *inputs, int length)
{
	if (inputs == NULL)
		return false;
	if (length <= 0)
		return false;

	if (fabs(inputs[length - 1].phase_error - inputs[0].phase_error)
		> DRIFT_COEFFICIENT_ABSOLUTE_MAX * length
	) {
		log_warn("Phase error is drifting too fast, a coarse calibration is needed");
		return false;
	}
	return true;
}

bool check_no_outlier(struct algorithm_input *inputs, int length, float mean_phase_error, int ref_fluctuation_ns)
{
	int i;

	if (inputs == NULL)
		return false;
	if (length <= 0)
		return false;

	for (i = 0; i < length; i ++) {
		if (fabs(inputs[i].phase_error - mean_phase_error) > ref_fluctuation_ns) {
			log_warn("Outlier detected at index %d", i);
			return false;
		}
	}
	return true;
}
