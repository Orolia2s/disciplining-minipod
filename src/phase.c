#include <math.h>

#include "phase.h"

#include "log.h"

int compute_phase_error_mean(struct algorithm_input *inputs, int length, float *mean_phase_error)
{
	float sum_phase_error = 0.0;
	int i;

	if (inputs == NULL)
		return -1;
	if (length <= 0)
		return -1;

	for (i = 0; i < length; i ++)
		sum_phase_error += inputs[i].phase_error;
	*mean_phase_error = sum_phase_error / length;
	log_debug("Mean Phase error: %f", *mean_phase_error);

	return 0;
}

int compute_frequency_error(struct algorithm_input *inputs, int length, struct linear_func_param *func)
{
	if (!inputs || !func)
		return -1;

	float x[length];
	float y[length];
	int ret, i;

	for (i = 0; i < length; i++) {
		x[i] = (float) i;
		y[i] = inputs[i].phase_error;
	}

	ret = simple_linear_reg(x, y, length, func);
	if (ret != 0) {
		return -1;
	}
	return 0;
}
