#include <math.h>

#include "phase.h"

#include "log.h"

float filter_phase(
	struct kalman_parameters *kalman,
	float phase,
	float estimated_drift,
	int interval)
{
	/* Init Kalman phase */
	if (!kalman->Kphase_set) {
		kalman->Kphase = phase;
		kalman->Kphase_set = true;
	}

	/* Predict */
	kalman->Kphase += interval * estimated_drift;
	log_debug("kalman->Kphase += interval * estimated_drift = %f", kalman->Kphase);
	kalman->Ksigma += kalman->q;

	/* Square computing to do it once */
	float square_Ksigma = pow(kalman->Ksigma, 2);
	float square_r = pow(kalman->r, 2);

	/* Update */
	float gain = square_Ksigma / (square_Ksigma + square_r);
	float innovation = (phase - kalman->Kphase);
	kalman->Kphase = kalman->Kphase + gain * innovation;
	kalman->Ksigma = sqrt((square_r * square_Ksigma)
		/ (square_r + square_Ksigma)
	);

	return kalman->Kphase;
}

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
