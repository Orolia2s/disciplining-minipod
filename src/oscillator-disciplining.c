#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <stddef.h>

#include <oscillator-disciplining/oscillator-disciplining.h>

#include "config.h"
#include "parameters.h"
#include "utils.h"
#include "algorithm_structs.h"

#define MRO_FINE_STEP_SENSITIVITY -3.E-12
#define MRO_COARSE_STEP_SENSITIVITY 1.24E-9
#define SETTLING_TIME 5

#define COARSE_RANGE_MIN 0
#define COARSE_RANGE_MAX 4194303
#define FINE_RANGE_MIN 1600
#define FINE_RANGE_MAX 3200

struct od {
    struct algorithm_state state;
    struct parameters params;
    clockid_t clockid;
};

static int init_algorithm_state(struct od * od) {
	int ret;
	double interpolation_value;
	uint16_t diff_fine;

	struct algorithm_state *state = &od->state;
	struct parameters *params = &od->params;

	/* Constants */
	state->mRO_fine_step_sensitivity = MRO_FINE_STEP_SENSITIVITY;
	state->mRO_coarse_step_sensitivity = MRO_COARSE_STEP_SENSITIVITY;
	state->coarse_ctrl = false;
	state->invalid_ctrl = false;
	state->settling_time = SETTLING_TIME;
	state->calib = false;
	state->status = INIT;
	state->ctrl_range_coarse[0] = COARSE_RANGE_MIN;
	state->ctrl_range_coarse[1] = COARSE_RANGE_MAX;
	state->ctrl_range_fine[0] = FINE_RANGE_MIN;
	state->ctrl_range_fine[1] = FINE_RANGE_MAX;
	state->fine_mid = (uint16_t) (0.5 * (FINE_RANGE_MIN + FINE_RANGE_MAX));
	state->estimated_drift = 0;

	/* Kalman filter parameters */
	state->kalman.Ksigma = params->ref_fluctuations_ns;
	state->kalman.Kphase = 0;
	state->kalman.q = 1;
	state->kalman.r = 5;

	/*
	 * Init ctrl_points, which have the same size as
	 * ctrl_load_nodes and ctrl_drift_coeffs, which
	 * is ctrl_nodes_length
	 */
	if (params->ctrl_nodes_length <=0)
		return -EINVAL;

	state->ctrl_points = malloc(params->ctrl_nodes_length * sizeof(int));
	if (state->ctrl_points == NULL)
		return -ENOMEM;

	diff_fine = state->ctrl_range_fine[1] - state->ctrl_range_fine[0];
	for (int i = 0; i < params->ctrl_nodes_length; i++) {
		state->ctrl_points[i] = (uint16_t) state->ctrl_range_fine[0] + params->ctrl_load_nodes[i] * diff_fine;
	}

	/*
	 * Compute estimated equilibrium using linear interpolation
	 * First convert ctrl_points array into a double array
	 * to use in lin_interp func
	 */
	double ctrl_points_double[params->ctrl_nodes_length];
	for (int i = 0; i < params->ctrl_nodes_length; i++)
		ctrl_points_double[i] = (double) state->ctrl_points[i];

	ret = lin_interp(
		ctrl_points_double,
		params->ctrl_drift_coeffs,
		params->ctrl_nodes_length,
		X_INTERPOLATION,
		0,
		&interpolation_value
	);
	if (ret < 0) {
		printf("Error occured during lin_terp, err %d", -ret);
		return ret;
	}
	state->estimated_equilibrium = (uint32_t) interpolation_value;

	return 0;
}

struct od *od_new(clockid_t clockid)
{
	struct od *od;

	od = calloc(1, sizeof(*od));
	if (od == NULL)
		return NULL;
	od->clockid = clockid;

    printf("Od_new called \n");
	return od;
}

struct od *od_new_from_config(const char *path, char err_msg[OD_ERR_MSG_LEN])
{
	struct od *od;
	int ret;

	if (path == NULL || *path == '\0' || err_msg == NULL) {
		errno = EINVAL;
		return NULL;
	}

	od = calloc(1, sizeof(*od));
	if (od == NULL)
		return NULL;

	ret = fill_parameters(&od->params, path, err_msg);

	print_parameters(&od->params);

	od->clockid = CLOCK_REALTIME;

	ret = init_algorithm_state(od);
	if (ret < 0) {
		printf("Error occured during init_algorithm_state, err %d", ret);
	}
	printf("Od_new_from_config called \n");

	return od;
}

uint32_t od_get_dac_min(const struct od *od)
{
	if (od == NULL)
		return  UINT32_MAX;

	printf("Od_get_dac_min called \n");
	return 0;
}

uint32_t od_get_dac_max(const struct od *od)
{
	if (od == NULL)
		return 0;

	printf("Od_get_dac_max called \n");
	return 0;
}

int od_process(struct od *od, const struct od_input *input,
		struct od_output *output)
{
	printf("Od_process called \n");
	return 0;
}

void od_destroy(struct od **od)
{
	if (od == NULL || *od == NULL)
		return;
	free((*od)->params.ctrl_load_nodes);
	(*od)->params.ctrl_load_nodes = NULL;
	free((*od)->params.ctrl_drift_coeffs);
	(*od)->params.ctrl_drift_coeffs = NULL;
	free(*od);
	*od = NULL;
	printf("Od_destroy called \n");
}
