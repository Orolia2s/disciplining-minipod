/**
 * @file oscillator-disciplining.c
 * @brief liboscillator-disciplining's main file.
 *
 * liboscillator-disciplining is a small library responsible of abstracting
 * disciplining algorithms used for an oscillator for which we want to control
 * the frequency compared to a PPS reference.
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <stddef.h>
#include <math.h>

#include <oscillator-disciplining/oscillator-disciplining.h>

#include "config.h"
#include "parameters.h"
#include "utils.h"
#include "algorithm_structs.h"
#include "log.h"

/** mRO base fine step sensitivity */
#define MRO_FINE_STEP_SENSITIVITY -3.E-12
/** mRO base coarse step sensitivity */
#define MRO_COARSE_STEP_SENSITIVITY 1.24E-9

/** Minimum possible value of coarse control */
#define COARSE_RANGE_MIN 0
/** Maximum possible value of coarse control */
#define COARSE_RANGE_MAX 4194303
/** Minimum possible value of fine control */
#define FINE_RANGE_MIN 1600
/** Maximum possible value of fine control */
#define FINE_RANGE_MAX 3200

/**
 * @struct od
 * @brief Library context.
 */
struct od {
    /** Algorith state */
    struct algorithm_state state;
    /** Algorithm paramters */
    struct parameters params;
};

static int init_algorithm_state(struct od * od) {
	int ret;
	double interpolation_value;
	uint16_t diff_fine;

	struct algorithm_state *state = &od->state;
	struct parameters *params = &od->params;

	/* Init state variables */
	state->mRO_fine_step_sensitivity = MRO_FINE_STEP_SENSITIVITY;
	state->mRO_coarse_step_sensitivity = MRO_COARSE_STEP_SENSITIVITY;
	state->invalid_ctrl = false;
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
	state->kalman.Kphase = 0.0;
	state->kalman.q = 1.0;
	state->kalman.r = 5.0;
	state->kalman.Kphase_set = false;

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
		debug("control points number %d is at %d\n", i, state->ctrl_points[i]);
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
		Y_INTERPOLATION,
		0,
		&interpolation_value
	);
	if (ret < 0) {
		err("Error occured during lin_interp, err %d\n", -ret);
		return ret;
	}
	state->estimated_equilibrium = (uint32_t) interpolation_value;
	info("Initialization: Estimated equilibirum is %d\n", state->estimated_equilibrium);

	return 0;
}

/*
 * Check if fine control setpoint is within available range +/- a tolerance.
 * If not the coarse control value is changed in order to re-center the fine control.
 * init_control_mRO must be called after a coarse change.
 */
static bool control_check_mRO(struct od *od, const struct od_input *input, struct od_output *output) {
	struct algorithm_state* state = &(od->state);
	struct parameters *params = &(od->params);

	if (state->calib
		&& state->estimated_equilibrium >= (uint32_t) state->ctrl_range_fine[0] + params->fine_stop_tolerance
		&& state->estimated_equilibrium <= (uint32_t) state->ctrl_range_fine[1] - params->fine_stop_tolerance
	) {
		info("Estimated equilibrium is in tolerance range and no calibration is running\n");
		return true;
	} else if (state->calib) {
		info("Coarse alignment must be adjusted based on calibration\n");
		int32_t delta_mid_fine = (int32_t) state->estimated_equilibrium - state->fine_mid;
		int32_t delta_coarse = (int32_t) round(
			delta_mid_fine
			* state->mRO_fine_step_sensitivity
			/ state->mRO_coarse_step_sensitivity
		);
		info("Delta mid_fine is %d\n", delta_mid_fine);
		info("Delta coarse is %d\n", delta_coarse);

		if (abs(delta_coarse) > params->max_allowed_coarse) {
			info("Large coarse change %u can lead to the loss of LOCK!", delta_coarse);
			delta_coarse = delta_coarse > 0 ?
				params->max_allowed_coarse :
				-params->max_allowed_coarse;
		}

		info("Requesting a coarse alignement to value %d\n", output->setpoint);
		output->setpoint = input->coarse_setpoint + delta_coarse;
		output->action = ADJUST_COARSE;
		output->value_phase_ctrl = 0;
		return false;
	} else {
		/* mRO needs to be calibrated */
		output->action = CALIBRATE;
		return false;
	}
}

static double get_reactivity(double phase_ns, int sigma, int min, int max, int power) {
	double r = max * exp(-pow(phase_ns/sigma, power));
	return r > min ? r : min;
}

static double filter_phase(struct kalman_parameters *kalman, double phase, int interval, double estimated_drift) {
	/* Init Kalman phase */
	if (!kalman->Kphase_set) {
		kalman->Kphase = phase;
		kalman->Kphase_set = true;
	}

	/* Predict */
	kalman->Kphase += interval * estimated_drift;
	kalman->Ksigma += kalman->q;

	/* Square computing to do it once */
	double square_Ksigma = pow(kalman->Ksigma, 2);
	double square_r = pow(kalman->r, 2);

	/* Update */
	double gain = square_Ksigma / (square_Ksigma + square_r);
	double innovation = (phase - kalman->Kphase);
	kalman->Kphase = kalman->Kphase + gain * innovation;
	kalman->Ksigma = sqrt((square_r * square_Ksigma)
		/ (square_r + square_Ksigma)
	);

	return kalman->Kphase;
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
	if (ret != 0) {
		err("Error parsing config file !\n");
		return NULL;
	}

	log_enable_debug(od->params.debug);
	if (od->params.debug) {
		print_parameters(&od->params);
	}

	ret = init_algorithm_state(od);
	if (ret < 0) {
		err("Error occured during init_algorithm_state, err %d\n", ret);
	}
	return od;
}

int od_process(struct od *od, const struct od_input *input,
		struct od_output *output)
{
	int ret;
	if (od == NULL || input == NULL || output == NULL)
	{
		err("At least one input variable is NULL\n");
		return -EINVAL;
	}

	log_enable_debug(od->params.debug);
	debug("State is %d\n", od->state.status);
	struct algorithm_state *state = &(od->state);
	struct parameters *params = &(od->params);
	debug("valid is %d and lock is %d\n", input->valid, input->lock);


	if (input->valid && input->lock)
	{
		if (params->calibrate_first)
		{
			params->calibrate_first = false;
			output->action = CALIBRATE;
			return 0;
		}
		/** Invalid control value, need to check mRO control values
		 * If calibration has been done, we need to make a control check of the mRO
		 */
		if (state->invalid_ctrl || state->status == CALIBRATION)
		{
			if(!control_check_mRO(od, input, output))
			{
				/* Control check has not been passed.
				* Either a Coarse alignement or a calibration process
				* has been decided and prepared in output
				*/
				state->calib = false;
				debug("control_check_mro has not been passed !\n");
				return 0;
			}
			debug("Control check mRO has been passed !\n");
			state->calib = false;
			state->status = INIT;
		}

		if (state->status == INIT)
		{
			output->action = ADJUST_FINE;
			output->setpoint = state->estimated_equilibrium;
			state->status = PHASE_ADJUSTMENT;
			info("INITIALIZATION: Applying estimated equilibrium setpoint %d\n", state->estimated_equilibrium);
			return 0;
		}
		else
		{
			if (labs(input->phase_error.tv_nsec) < params->phase_jump_threshold_ns)
			{
				/* Call Main loop */
				double phase = input->phase_error.tv_nsec;
				double filtered_phase = filter_phase(
					&(state->kalman),
					phase,
					params->ref_fluctuations_ns,
					state->estimated_drift
				);
				double innovation = phase - filtered_phase;

				double x;
				if (fabs(phase) <= params->ref_fluctuations_ns
					&& fabs(innovation) <= params->ref_fluctuations_ns)
					x = filtered_phase;
				else
					x = phase;

				double r = get_reactivity(
					fabs(x),
					params->ref_fluctuations_ns,
					params->reactivity_min,
					params->reactivity_max,
					params->reactivity_power
				);
				double react_coeff = - x / r;
				info("get_recativity gives %f, react coeff is now %f\n", r, react_coeff);

				double ctrl_points_double[params->ctrl_nodes_length];
				for (int i = 0; i < params->ctrl_nodes_length; i++)
					ctrl_points_double[i] = (double) state->ctrl_points[i];
				double interp_value;

				ret = lin_interp(
					ctrl_points_double,
					params->ctrl_drift_coeffs,
					params->ctrl_nodes_length,
					Y_INTERPOLATION,
					react_coeff,
					&interp_value
				);
				if (ret < 0)
				{
					err("Error occured in lin_interp: %d\n", ret);
					return -1;
				}

				state->fine_ctrl_value = (uint16_t) round(interp_value);

				if (state->fine_ctrl_value >= state->ctrl_range_fine[0]
					&& state->fine_ctrl_value <= state->ctrl_range_fine[1])
				{
					state->estimated_drift = react_coeff;
					output->action = ADJUST_FINE;
					output->setpoint = state->fine_ctrl_value;
				}
				else
				{
					info("Control value %u is out of range! Decrease reactivity or allow a lower phase jump threshold for quicker convergence. If this persists consider recablibration\n",
						state->fine_ctrl_value
					);
				
					double stop_value;
					
					if (state->fine_ctrl_value < state->ctrl_range_fine[0])
						stop_value = state->ctrl_range_fine[0];
					else
						stop_value = state->ctrl_range_fine[1];
					
					double ctrl_points_double[params->ctrl_nodes_length];
					for (int i = 0; i < params->ctrl_nodes_length; i++)
						ctrl_points_double[i] = (double) state->ctrl_points[i];

					ret = lin_interp(
						ctrl_points_double,
						params->ctrl_drift_coeffs,
						params->ctrl_nodes_length,
						X_INTERPOLATION,
						stop_value,
						&state->estimated_drift
					);
					info("Estimated drift is now %f\n", state->estimated_drift);
					
					if (ret < 0)
					{
						err("Error occured in lin_interp: %d\n", ret);
						return -1;
					}

					output->action = ADJUST_FINE;
					output->setpoint = stop_value;
				}
				return 0;
			} else {
				/* Phase jump needed */
				output->action = PHASE_JUMP;
				output->value_phase_ctrl = input->phase_error.tv_nsec;
				return 0;
			}
		}
	} else {
		state->status = HOLDOVER;
		output->action = ADJUST_FINE;
		output->setpoint = state->estimated_equilibrium;
	}
	return 0;
}

struct calibration_parameters * od_get_calibration_parameters(struct od *od)
{
	if (od == NULL)
	{
		err("Library context is null\n");
		return NULL;
	}

	if (od->params.ctrl_nodes_length <= 0)
	{
		err("get_calibration_parameters: Length cannot be negative\n");
		return NULL;
	}
	
	struct calibration_parameters *calib_params = malloc(sizeof(struct calibration_parameters));
	if (calib_params == NULL)
	{
		err("Could not allocate memory to create calibration parameters data\n");
		return NULL;
	}

	calib_params->ctrl_points = malloc(od->params.ctrl_nodes_length * sizeof(uint16_t));
	if (calib_params->ctrl_points == NULL) {
		err("Could not allocate memory to create ctrl points in calibration parameters data\n");
		free(calib_params);
		calib_params = NULL;
		return NULL;
	}

	for (int i = 0; i < od->params.ctrl_nodes_length; i++)
	{
		calib_params->ctrl_points[i] = od->state.ctrl_points[i];
	}

	calib_params->length = od->params.ctrl_nodes_length;
	calib_params->nb_calibration = od->params.nb_calibration;
	calib_params->settling_time = od->params.settling_time;
	od->state.calib = true;
	return calib_params;
}

static void free_calibration(struct calibration_parameters *calib_params, struct calibration_results *calib_results)
{
	free(calib_params->ctrl_points);
	calib_params->ctrl_points = NULL;
	free(calib_params);
	calib_params = NULL;
	free(calib_results->measures);
	calib_results->measures = NULL;
	free(calib_results);
	calib_results = NULL;
	return;
}

static int update_config(char * path, double * drift_coeffs, int length)
{
	FILE * config;
	FILE * tmp_config;
	char * line = NULL;
	size_t len = 0;
	int read;
	int ret;
	char * temp_file_name;

	config = fopen(path, "r");
	if (config == NULL) {
		err("Could not open file %s\n", path);
		return -EINVAL;
	}

	/* create <configFileName>.tmp file to store new config s*/
	temp_file_name = malloc((strlen(path) + 5) * sizeof(char));
	if (temp_file_name == NULL) {
		err("Could not allocate memory for temp_file name !\n");
		return -ENOMEM;
	}
	ret = sprintf(temp_file_name, "%s.tmp", path);
	if (ret < 0) {
		err("Error ocurred while creating temp file");
		free(temp_file_name);
		return -1;
	}


	tmp_config = fopen(temp_file_name, "wb");
	if (tmp_config == NULL) {
		err("Could not create temp file %s\n", temp_file_name);
		free(temp_file_name);
		return -1;
	}

	/* Copy all config variables except ctrl_drift_coeffs that needs to be updated */
	while ((read = getline(&line, &len, config)) != -1) {
		if(strncmp(line, "ctrl_drift_coeffs=", sizeof("ctrl_drift_coeffs=") - 1) == 0) {
			fprintf(tmp_config, "ctrl_drift_coeffs=");
			for (int i = 0; i < length; i++) {
				fprintf(tmp_config, "%f", drift_coeffs[i]);
				if (i < length -1) {
					fprintf(tmp_config, ",");
				}
			}
			fprintf(tmp_config, "\r\n");
		} else {
			fputs(line, tmp_config);
		}
	}

	fclose(config);
	fclose(tmp_config);
	if (line)
		free(line);

	/* Replace old config */
	ret = rename(temp_file_name, path);
	if(ret < 0) {
		err("Error occured when trying to overwrite config file\n");
	}
	free(temp_file_name);
	return 0;
}

void od_calibrate(struct od *od, struct calibration_parameters *calib_params, struct calibration_results *calib_results)
{
	int ret;
	int length;

	if (od == NULL || calib_params == NULL || calib_results == NULL)
	{
		err("od_calibration: at least one input parameter is null\n");
		return;
	}

	od->state.status = CALIBRATION;

	if (calib_params->length != od->params.ctrl_nodes_length || calib_params->length != calib_results->length)
	{
		err("od_calibrate: length mismatch\n");
		free_calibration(calib_params, calib_results);
		return;
	}
	length = od->params.ctrl_nodes_length;

	if (calib_params->nb_calibration != calib_results->nb_calibration)
	{
		err("od_calibrate: nb_calibration mismatch \n");
		free_calibration(calib_params, calib_results);
		return;
	}

	/* Create array representing all the integers between 0 and n_calibration */
	double x[calib_params->nb_calibration];
	for (int i = 0; i < calib_params->nb_calibration; i++)
	{
		x[i] = (double) i;
	}

	/* Update drift coefficients */
	for (int i = 0; i < od->params.ctrl_nodes_length; i++)
	{
		double v[calib_params->nb_calibration];
		for(int j = 0; j < calib_params->nb_calibration; j++)
		{

			struct timespec * current_measure = calib_results->measures + i * calib_params->nb_calibration + j;
			v[j] = (double) current_measure->tv_nsec;
		}

		struct linear_func_param func_params;

		ret = simple_linear_reg(
			x,
			v,
			calib_params->nb_calibration,
			&func_params
		);


		if (ret < 0)
		{
			err("od_calibrate: error occured in simple_linear_reg, err %d\n", ret);
			free_calibration(calib_params, calib_results);
			return;
		}
		od->params.ctrl_drift_coeffs[i] = func_params.a;
	}

	/* Save new drift coeffs in config file */
	ret = update_config(od->params.path, od->params.ctrl_drift_coeffs, od->params.ctrl_nodes_length);
	if (ret < 0) {
		err("Error updating configuration with new drift coefficients ! Calibration must be done again next time \n");
	}

	double interp_value;
	double ctrl_points_double[length];
	for (int i = 0; i < length; i++)
		ctrl_points_double[i] = (double) od->state.ctrl_points[i];
	ret = lin_interp(
		ctrl_points_double,
		od->params.ctrl_drift_coeffs,
		length,
		Y_INTERPOLATION,
		0,
		&interp_value
	);
	if(ret < 0)
	{
		err("od_calibrate: error occured in lin_interp, err %d\n", ret);
	}

	od->state.estimated_equilibrium = (uint32_t) interp_value;

	if (od->state.ctrl_points[length - 1] - od->state.ctrl_points[0] == 0)
	{
		err("Control points cannot have the same value !\n");
		free_calibration(calib_params, calib_results);
		return;
	}

	od->state.mRO_fine_step_sensitivity = 1E-9
		* ( od->params.ctrl_drift_coeffs[length - 1] - od->params.ctrl_drift_coeffs[0])
		/ ( od->state.ctrl_points[length - 1] - od->state.ctrl_points[0] );

	free_calibration(calib_params, calib_results);
	return;
}

void od_destroy(struct od **od)
{
	if (od == NULL || *od == NULL)
		return;
	free((*od)->params.ctrl_load_nodes);
	(*od)->params.ctrl_load_nodes = NULL;
	free((*od)->params.ctrl_drift_coeffs);
	(*od)->params.ctrl_drift_coeffs = NULL;
	free((*od)->params.path);
	(*od)->params.path = NULL;
	free(*od);
	*od = NULL;
}
