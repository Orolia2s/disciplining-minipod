#include <math.h>

#include "checks.h"

#include "algorithm_structs.h"
#include "log.h"

bool check_gnss_valid_over_cycle(struct od_input *inputs, int length)
{
	bool gnss_valid = true;
	int i;

	if (inputs == NULL)
		return false;
	if (length <= 0)
		return false;

	for (i = 0; i < length; i ++)
		gnss_valid = gnss_valid & inputs[i].valid;
	return gnss_valid;
}

bool check_lock_over_cycle(struct od_input *inputs, int length)
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

bool check_max_drift(struct od_input *inputs, int length)
{
	if (inputs == NULL)
		return false;
	if (length <= 0)
		return false;

	if (fabs((inputs[length - 1].phase_error.tv_nsec + (float) inputs[length - 1].qErr / PS_IN_NS)
		- (inputs[0].phase_error.tv_nsec + (float) inputs[0].qErr / PS_IN_NS))
		> DRIFT_COEFFICIENT_ABSOLUTE_MAX * length
	) {
		log_warn("Phase error is drifting too fast, a coarse calibration is needed");
		return false;
	}
	return true;
}

bool check_no_outlier(struct od_input *inputs, int length, float mean_phase_error, int ref_fluctuation_ns)
{
	int i;

	if (inputs == NULL)
		return false;
	if (length <= 0)
		return false;

	for (i = 0; i < length; i ++) {
		if (fabs(inputs[i].phase_error.tv_nsec + (float) inputs[i].qErr / PS_IN_NS - mean_phase_error) > ref_fluctuation_ns) {
			log_warn("Outlier detected at index %d", i);
			return false;
		}
	}
	return true;
}
