/*
 * liboscillator-disciplining: Disciplining Algorithm for Orolia's mRO50.
 * Copyright (C) 2021  Spectracom SAS

 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

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
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <oscillator-disciplining/oscillator-disciplining.h>

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
#define FINE_RANGE_MIN 0
/** Maximum possible value of fine control */
#define FINE_RANGE_MAX 4800
/** Minimum possible value of fine control used for calibration*/
#define FINE_MID_RANGE_MIN 1600
/** Maximum possible value of fine control used for calibration*/
#define FINE_MID_RANGE_MAX 3200
/**
 * Smooth exponential factor for estimated equilibrium
 * used during holdover phase
 */
#define ALPHA_ES 0.01
/**
 * Smooth exponential factor for estimated equilibrium
 * used during tracking phase
 */
#define ALPHA_ES_TRACKING 0.018
/**
 * Maximum drift coefficient
 * (Fine mid value * abs(mRO base fine step sensitivity) in s/s)
 */
#define DRIFT_COEFFICIENT_ABSOLUTE_MAX 7.2

#define PS_IN_NS 1000

#define PS_IN_NS 1000


/**
 * @struct od
 * @brief Library context.
 */
struct od {
	/** Algorith state */
	struct algorithm_state state;
	/** Algorithm parameters */
	struct minipod_config minipod_config;
	struct disciplining_parameters dsc_parameters;
};

static int init_ctrl_points(struct algorithm_state *state, float *load_nodes, float *drift_coeffs, uint8_t length) {
	uint16_t diff_fine;

	/*
	* Init ctrl_points, which have the same size as
	* ctrl_load_nodes and ctrl_drift_coeffs, which
	* is ctrl_nodes_length
	*/
	if (length <=0)
		return -EINVAL;

	state->ctrl_points = malloc(length * sizeof(float));
	if (state->ctrl_points == NULL)
		return -ENOMEM;

	state->ctrl_drift_coeffs = malloc(length * sizeof(float));

	diff_fine = state->ctrl_range_fine[1] - state->ctrl_range_fine[0];
	log_debug("Control points used:");
	for (int i = 0; i < length; i++) {
		state->ctrl_points[i] = (float) state->ctrl_range_fine[0] + load_nodes[i] * diff_fine;
		state->ctrl_drift_coeffs[i] = drift_coeffs[i];
		log_debug("\t%d: %f -> %f", i, state->ctrl_points[i], state->ctrl_drift_coeffs[i]);
	}
	state->ctrl_points_length = length;
	return 0;
}

static int init_algorithm_state(struct od * od) {
	int ret;
	float interpolation_value;

	struct algorithm_state *state = &od->state;
	struct disciplining_parameters *dsc_parameters = &od->dsc_parameters;
	struct minipod_config *config = &od->minipod_config;

	/* Init state variables */
	state->mRO_fine_step_sensitivity = MRO_FINE_STEP_SENSITIVITY;
	state->mRO_coarse_step_sensitivity = MRO_COARSE_STEP_SENSITIVITY;
	state->invalid_ctrl = false;
	state->calib = false;
	state->status = INIT;
	state->ctrl_range_coarse[0] = COARSE_RANGE_MIN;
	state->ctrl_range_coarse[1] = COARSE_RANGE_MAX;
	state->ctrl_range_fine[0] = FINE_MID_RANGE_MIN;
	state->ctrl_range_fine[1] = FINE_MID_RANGE_MAX;
	state->fine_mid = (uint16_t) (0.5 * (FINE_MID_RANGE_MIN + FINE_MID_RANGE_MAX));
	state->estimated_drift = 0;
	state->fine_ctrl_value = 0;
	state->od_process_count = 0;

	/* Kalman filter parameters */
	state->kalman.Ksigma = config->ref_fluctuations_ns;
	state->kalman.Kphase = 0.0;
	state->kalman.q = 1.0;
	state->kalman.r = 5.0;
	state->kalman.Kphase_set = false;

	/*
	 * Check wether nominal parameters should be used or factory ones
	 */
	if (config->oscillator_factory_settings || !dsc_parameters->calibration_valid) {
		if (!dsc_parameters->calibration_valid) {
			log_warn("Calibration parameters are not valid for this card. Factory settings will be used.");
			log_warn("Please calibrate your card on a stable gnss reference if you want to use calibration parameters");
		}
		log_debug("Using factory settings");
		ret = init_ctrl_points(
			state,
			dsc_parameters->ctrl_load_nodes_factory,
			dsc_parameters->ctrl_drift_coeffs_factory,
			dsc_parameters->ctrl_nodes_length_factory
		);
	} else {
		ret = init_ctrl_points(
			state,
			dsc_parameters->ctrl_load_nodes,
			dsc_parameters->ctrl_drift_coeffs,
			dsc_parameters->ctrl_nodes_length
		);
	}
	if (ret != 0)
		return ret;

	ret = lin_interp(
		state->ctrl_points,
		state->ctrl_drift_coeffs,
		state->ctrl_points_length,
		Y_INTERPOLATION,
		0,
		&interpolation_value
	);

	if (ret < 0) {
		log_error("Error occured during lin_interp, err %d", -ret);
		return ret;
	}
	state->estimated_equilibrium = (uint32_t) interpolation_value;
	log_info("Initialization: Estimated equilibirum is %d", state->estimated_equilibrium);
	state->estimated_equilibrium_ES = state->estimated_equilibrium;

	return 0;
}

/*
 * Check if fine control setpoint is within available range +/- a tolerance.
 * If not the coarse control value is changed in order to re-center the fine control.
 * init_control_mRO must be called after a coarse change.
 */
static bool control_check_mRO(struct od *od, const struct od_input *input, struct od_output *output) {
	struct algorithm_state *state = &(od->state);
	struct minipod_config *config = &(od->minipod_config);
	struct disciplining_parameters *dsc_parameters = &(od->dsc_parameters);
	int i;

	log_debug("Control Check mRO:");
	/* Check estimated equilibrium is in tolerance range */
	if (state->calib
		&& state->estimated_equilibrium >= (uint32_t) state->ctrl_range_fine[0] + config->fine_stop_tolerance
		&& state->estimated_equilibrium <= (uint32_t) state->ctrl_range_fine[1] - config->fine_stop_tolerance
		&& state->ctrl_drift_coeffs[0] >= 0.0 && state->ctrl_drift_coeffs[state->ctrl_points_length - 1] <= 0.0
	) {
		log_info("Estimated equilibrium is in tolerance range, saving calibration in config file");
		for(i = 0; i < state->ctrl_points_length - 2; i++) {
			if (state->ctrl_drift_coeffs[i] < state->ctrl_drift_coeffs[i+1]) {
				log_debug("ctrl_drift_coeffs[%d] = %f is greater than ctrl_drift_coeffs[%d] = %f",
					i, state->ctrl_drift_coeffs[i], i + 1, state->ctrl_drift_coeffs[i+1]);
				log_warn("Calibration coefficients are not descending with fine values");
				output->action = CALIBRATE;
				return false;
			}
		}
		log_debug("Calibration coefficients are descending with fine values");
		for(i = 0; i < state->ctrl_points_length; i++) {
			if (fabs(state->ctrl_drift_coeffs[i]) > DRIFT_COEFFICIENT_ABSOLUTE_MAX) {
				log_warn("ctrl_drift_coeffs[%d] coefficient is greater than %f in absolute: %f",
					i, DRIFT_COEFFICIENT_ABSOLUTE_MAX, state->ctrl_drift_coeffs[i]);
				output->action = CALIBRATE;
				return false;
			}
		}
		log_debug("All coefficients are inferior to %f in absolute value", DRIFT_COEFFICIENT_ABSOLUTE_MAX);

		dsc_parameters->coarse_equilibrium = input->coarse_setpoint;
		dsc_parameters->calibration_valid = true;
		return true;
	} else if (state->calib) {
		log_info("Coarse alignment must be adjusted based on calibration");
		int32_t delta_mid_fine = (int32_t) state->estimated_equilibrium - state->fine_mid;
		int32_t delta_coarse = (int32_t) round(
			delta_mid_fine
			* state->mRO_fine_step_sensitivity
			/ state->mRO_coarse_step_sensitivity
		);
		log_debug("Deltas: mid_fine is %d, coarse is %d", delta_mid_fine, delta_coarse);

		if (abs(delta_coarse) > config->max_allowed_coarse) {
			log_info("Large coarse change %u can lead to the loss of LOCK!", delta_coarse);
			delta_coarse = delta_coarse > 0 ?
				config->max_allowed_coarse :
				-config->max_allowed_coarse;
		}

		output->setpoint = input->coarse_setpoint + delta_coarse;
		output->action = ADJUST_COARSE;
		output->value_phase_ctrl = 0;
		log_info("Requesting a coarse alignement to value %d", output->setpoint);
		return false;
	} else {
		/* mRO needs to be calibrated */
		output->action = CALIBRATE;
		return false;
	}
}

static float get_reactivity(float phase_ns, int sigma, int min, int max, int power) {
	float r = max * exp(-pow(phase_ns/sigma, power));
	return r > min ? r : min;
}

static float filter_phase(struct kalman_parameters *kalman, float phase, int interval, float estimated_drift) {
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

static int compute_phase_error_mean(struct od_input *inputs, int length, float *mean_phase_error)
{
	float sum_phase_error = 0.0;
	int i;

	if (inputs == NULL)
		return -1;
	if (length <= 0)
		return -1;

	for (i = 0; i < length; i ++)
		sum_phase_error += inputs[i].phase_error.tv_nsec + (float) inputs[i].qErr / PS_IN_NS;
	*mean_phase_error = sum_phase_error / length;
	return 0;
}

static bool check_gnss_valid_over_cycle(struct od_input *inputs, int length)
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

static bool check_lock_over_cycle(struct od_input *inputs, int length)
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

static bool check_max_drift(struct od_input *inputs, int length)
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

static bool check_no_outlier(struct od_input *inputs, int length, float mean_phase_error, int ref_fluctuation_ns)
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

static void print_inputs(struct od_input inputs[7])
{
	log_info(
		"Inputs: [%f, %f, %f, %f, %f, %f, %f]",
		(float) inputs[0].phase_error.tv_nsec + (float) inputs[0].qErr / PS_IN_NS,
		(float) inputs[1].phase_error.tv_nsec + (float) inputs[1].qErr / PS_IN_NS,
		(float) inputs[2].phase_error.tv_nsec + (float) inputs[2].qErr / PS_IN_NS,
		(float) inputs[3].phase_error.tv_nsec + (float) inputs[3].qErr / PS_IN_NS,
		(float) inputs[4].phase_error.tv_nsec + (float) inputs[4].qErr / PS_IN_NS,
		(float) inputs[5].phase_error.tv_nsec + (float) inputs[5].qErr / PS_IN_NS,
		(float) inputs[6].phase_error.tv_nsec + (float) inputs[6].qErr / PS_IN_NS
	);
}

struct od *od_new_from_config(struct minipod_config *minipod_config, struct disciplining_parameters *disciplining_config, char err_msg[OD_ERR_MSG_LEN])
{
	struct od *od;
	int ret;

	od = calloc(1, sizeof(struct od));
	if (od == NULL)
		return NULL;

	memcpy(&od->minipod_config, minipod_config, sizeof(struct minipod_config));

	log_set_level(
		od->minipod_config.debug >= 0 ?
		od->minipod_config.debug :
		LOG_INFO
	);
	print_minipod_config(&od->minipod_config);

	memcpy(&od->dsc_parameters, disciplining_config, sizeof(struct disciplining_parameters));

	print_disciplining_parameters(&od->dsc_parameters);

	ret = init_algorithm_state(od);
	if (ret < 0) {
		log_error("Error occured during init_algorithm_state, err %d", ret);
	}
	return od;
}

int od_process(struct od *od, const struct od_input *input,
		struct od_output *output)
{
	int ret;
	if (od == NULL || input == NULL || output == NULL)
	{
		log_error("At least one input variable is NULL");
		return -EINVAL;
	}

	log_set_level(
		od->minipod_config.debug >= 0 ?
		od->minipod_config.debug :
		LOG_INFO
	);

	struct algorithm_state *state = &(od->state);
	struct disciplining_parameters *dsc_parameters = &(od->dsc_parameters);
	struct minipod_config *config = &(od->minipod_config);
	output->action = NO_OP;

	log_debug("OD_PROCESS: State is %d, gnss valid is %d and mRO lock is %d",
		od->state.status, input->valid, input->lock);
	memcpy(&(state->inputs[state->od_process_count]), input, sizeof(struct od_input));

	if (state->od_process_count == 6) {
		state->od_process_count = 0;
		print_inputs(state->inputs);

		if (check_gnss_valid_over_cycle((struct od_input *) &state->inputs, 7)
			&& check_lock_over_cycle((struct od_input *) &state->inputs, 7))
		{
			if (od->state.status != CALIBRATION
				&& (
					config->calibrate_first
					|| input->calibration_requested
				)
			)
			{
				config->calibrate_first = false;
				output->action = CALIBRATE;
				od->state.status = CALIBRATION;
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
					log_debug("Control_check_mro has not been passed !");
					return 0;
				}
				log_debug("Control check mRO has been passed !");
				state->calib = false;
				state->status = INIT;

				/* Request coarse value to be saved in mRO50 memory */
				output->action = SAVE_DISCIPLINING_PARAMETERS;
				return 0;
			}

			/* Initialization */
			if (state->status == INIT)
			{
				if (config->oscillator_factory_settings && dsc_parameters->coarse_equilibrium_factory >= 0 && input->coarse_setpoint != dsc_parameters->coarse_equilibrium_factory) {
					output->action = ADJUST_COARSE;
					output->setpoint = dsc_parameters->coarse_equilibrium_factory;
					log_info("INITIALIZATION: Applying factory coarse equilibrium setpoint %d", dsc_parameters->coarse_equilibrium);
				} else if (!config->oscillator_factory_settings && dsc_parameters->coarse_equilibrium >= 0 && input->coarse_setpoint != dsc_parameters->coarse_equilibrium) {
					output->action = ADJUST_COARSE;
					output->setpoint = dsc_parameters->coarse_equilibrium;
					log_info("INITIALIZATION: Applying coarse equilibrium setpoint %d", dsc_parameters->coarse_equilibrium);
				} else {
					if (dsc_parameters->coarse_equilibrium < 0)
						log_warn("Unknown coarse_equilibrium, using value saved in oscillator,"
							"consider calibration if disciplining is not efficient");
					output->action = ADJUST_FINE;
					output->setpoint = state->estimated_equilibrium;
					state->status = PHASE_ADJUSTMENT;
					log_info("INITIALIZATION: Applying estimated fine equilibrium setpoint %d", state->estimated_equilibrium);

				}
				return 0;
			}
			/* Holdover and valid flag switched to valid,
			* We wait for one cycle to start disciplining again
			*/
			else if (state->status == HOLDOVER)
			{
				state->status = PHASE_ADJUSTMENT;
				output->action = ADJUST_FINE;
				output->setpoint = state->estimated_equilibrium;
				log_info("HOLDOVER: Gnss flag valid again, waiting one cycle before restarting disciplining");
			}
			else
			{
				/* Compute mean phase error over cycle */
				float mean_phase_error;
				ret = compute_phase_error_mean((struct od_input *) state->inputs, 7, &mean_phase_error);
				if (ret != 0) {
					log_error("Mean phase error could be computed");
					state->status = HOLDOVER;
					output->action = ADJUST_FINE;
					output->setpoint = state->estimated_equilibrium_ES;
					return 0;
				}
				log_debug("Mean Phase error: %f", mean_phase_error);
				/* Check phase error is below threshold configured */
				if (fabs(mean_phase_error) < (float) config->phase_jump_threshold_ns)
				{
					/* Call Main loop */
					if (!check_no_outlier((struct od_input *) state->inputs, 7, mean_phase_error, config->ref_fluctuations_ns)) {
						log_warn("Outlier detected ! entering holdover");
						state->status = HOLDOVER;
						output->action = ADJUST_FINE;
						output->setpoint = state->estimated_equilibrium_ES;
						return 0;
					}

					/* Check phase error is not exceeding maximal drift and that there is not outlier value in cycla */
					// if (!check_max_drift((struct od_input *) state->inputs, 7)) {
					// 	output->action = CALIBRATE;
					// 	od->state.status = CALIBRATION;
					// 	return 0;
					// }

					/* Legacy code used for debug */
					float filtered_phase = filter_phase(
						&(state->kalman),
						mean_phase_error,
						SETTLING_TIME,
						state->estimated_drift
					);
					log_info("Filtered phase is %f", filtered_phase);

					if (fabs(mean_phase_error) < config->ref_fluctuations_ns
						&& (state->fine_ctrl_value >= state->ctrl_range_fine[0]
						&& state->fine_ctrl_value <= state->ctrl_range_fine[1])
						&& fabs((state->inputs[6].phase_error.tv_nsec + (float) state->inputs[6].qErr / PS_IN_NS)
						- (state->inputs[0].phase_error.tv_nsec + (float) state->inputs[0].qErr / PS_IN_NS))
						< (float) config->ref_fluctuations_ns)
					{
						state->estimated_equilibrium_ES =
							round((ALPHA_ES_TRACKING * state->fine_ctrl_value
							+ (1.0 - ALPHA_ES_TRACKING) * state->estimated_equilibrium_ES));
						log_info("Estimated equilibrium with exponential smooth is %d",
							state->estimated_equilibrium_ES);
					}

					float r = get_reactivity(
						fabs(mean_phase_error),
						config->ref_fluctuations_ns,
						config->reactivity_min,
						config->reactivity_max,
						config->reactivity_power
					);
					float react_coeff = - mean_phase_error / r;
					log_info("get_reactivity gives %f, react coeff is now %f", r, react_coeff);

					float interp_value;

					ret = lin_interp(
						state->ctrl_points,
						state->ctrl_drift_coeffs,
						state->ctrl_points_length,
						Y_INTERPOLATION,
						react_coeff,
						&interp_value
					);
					if (ret < 0)
					{
						log_error("Error occured in lin_interp: %d", ret);
						return -1;
					}
					/* If linear interpretation returns a negative value, consider value found is 0 before conversion to integers */
					if (interp_value <= 0.0) {
						log_warn("fine control value found is negative (%f), setting to 0.0", interp_value);
						interp_value = 0.0;
					}

					state->fine_ctrl_value = (uint16_t) round(interp_value);

					if (state->fine_ctrl_value >= FINE_RANGE_MIN + config->fine_stop_tolerance
						&& state->fine_ctrl_value <= FINE_RANGE_MAX - config->fine_stop_tolerance)
					{
						state->estimated_drift = react_coeff;
						output->action = ADJUST_FINE;
						output->setpoint = state->fine_ctrl_value;
					}
					else
					{
						log_warn("Control value is out of range, if convergence is not reached"
							" consider recalibration or other reactivity parameters");

						float stop_value;

						if (state->fine_ctrl_value < FINE_RANGE_MIN + config->fine_stop_tolerance)
							stop_value = FINE_RANGE_MIN + config->fine_stop_tolerance;
						else
							stop_value = FINE_RANGE_MAX - config->fine_stop_tolerance;

						ret = lin_interp(
							state->ctrl_points,
							state->ctrl_drift_coeffs,
							state->ctrl_points_length,
							X_INTERPOLATION,
							stop_value,
							&state->estimated_drift
						);
						log_info("Estimated drift is now %f", state->estimated_drift);

						if (ret < 0)
						{
							log_error("Error occured in lin_interp: %d", ret);
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
			log_warn("HOLDOVER activated: GNSS data is not valid and/or oscillator's lock has been lost");
			log_info("Applying estimated equilibirum until going out of holdover");
			state->status = HOLDOVER;
			output->action = ADJUST_FINE;
			output->setpoint = state->estimated_equilibrium_ES;
		}
	} else {
		state->od_process_count++;
		output->action = NO_OP;
	}
	return 0;
}

int od_get_disciplining_parameters(struct od *od, struct disciplining_parameters* disciplining_parameters) {
	if (od == NULL) {
		log_error("Library context is null");
		return -1;
	}
	memcpy(disciplining_parameters, &od->dsc_parameters, sizeof(struct disciplining_parameters));
	return 0;
}

struct calibration_parameters * od_get_calibration_parameters(struct od *od)
{
	if (od == NULL)
	{
		log_error("Library context is null");
		return NULL;
	}

	if (od->dsc_parameters.ctrl_nodes_length <= 0)
	{
		log_error("get_calibration_parameters: Length cannot be negative");
		return NULL;
	}

	struct calibration_parameters *calib_params = malloc(sizeof(struct calibration_parameters));
	if (calib_params == NULL)
	{
		log_error("Could not allocate memory to create calibration parameters data");
		return NULL;
	}

	calib_params->ctrl_points = malloc(od->dsc_parameters.ctrl_nodes_length * sizeof(uint16_t));
	if (calib_params->ctrl_points == NULL) {
		log_error("Could not allocate memory to create ctrl points in calibration parameters data");
		free(calib_params);
		calib_params = NULL;
		return NULL;
	}

	for (int i = 0; i < od->dsc_parameters.ctrl_nodes_length; i++)
	{
		calib_params->ctrl_points[i] = od->state.ctrl_points[i];
	}

	calib_params->length = od->dsc_parameters.ctrl_nodes_length;
	calib_params->nb_calibration = od->minipod_config.nb_calibration;
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

void od_calibrate(struct od *od, struct calibration_parameters *calib_params, struct calibration_results *calib_results)
{
	int ret;
	int length;
	struct disciplining_parameters *dsc_parameters;
	struct algorithm_state *state;

	if (od == NULL || calib_params == NULL || calib_results == NULL)
	{
		log_error("od_calibration: at least one input parameter is null");
		return;
	}
	dsc_parameters = &od->dsc_parameters;
	state = &od->state;

	if (calib_params->length != dsc_parameters->ctrl_nodes_length || calib_params->length != calib_results->length)
	{
		log_error("od_calibrate: length mismatch");
		free_calibration(calib_params, calib_results);
		return;
	}
	length = dsc_parameters->ctrl_nodes_length;

	if (calib_params->nb_calibration != calib_results->nb_calibration)
	{
		log_error("od_calibrate: nb_calibration mismatch");
		free_calibration(calib_params, calib_results);
		return;
	}

	/* Create array representing all the integers between 0 and n_calibration */
	float x[calib_params->nb_calibration];
	for (int i = 0; i < calib_params->nb_calibration; i++)
		x[i] = (float) i;

	/* Update drift coefficients */
	log_debug("CALIBRATION: Updating drift coefficients:");
	for (int i = 0; i < length; i++)
	{
		float v[calib_params->nb_calibration];
		for(int j = 0; j < calib_params->nb_calibration; j++)
			v[j] = *(calib_results->measures + i * calib_params->nb_calibration + j);

		struct linear_func_param func_params;

		ret = simple_linear_reg(
			x,
			v,
			calib_params->nb_calibration,
			&func_params
		);


		if (ret < 0)
		{
			log_error("od_calibrate: error occured in simple_linear_reg, err %d", ret);
			free_calibration(calib_params, calib_results);
			return;
		}
		dsc_parameters->ctrl_drift_coeffs[i] = func_params.a;
		log_debug("\t[%d]: %f", i, dsc_parameters->ctrl_drift_coeffs[i]);
	}

	float interp_value;
	for (int i = 0; i < length; i++)
		state->ctrl_drift_coeffs[i] = dsc_parameters->ctrl_drift_coeffs[i];
	ret = lin_interp(
		state->ctrl_points,
		state->ctrl_drift_coeffs,
		length,
		Y_INTERPOLATION,
		0,
		&interp_value
	);
	if(ret < 0)
		log_error("od_calibrate: error occured in lin_interp, err %d", ret);

	state->estimated_equilibrium = (uint32_t) interp_value;
	log_debug("Estimated equilibrium is now %d", state->estimated_equilibrium);
	log_debug("Resetting estimated equiblibrium exponential smooth");
	state->estimated_equilibrium_ES = state->estimated_equilibrium;

	if (state->ctrl_points[length - 1] - state->ctrl_points[0] == 0)
	{
		log_error("Control points cannot have the same value !");
		free_calibration(calib_params, calib_results);
		return;
	}

	state->mRO_fine_step_sensitivity = 1E-9
		* ( dsc_parameters->ctrl_drift_coeffs[length - 1] - dsc_parameters->ctrl_drift_coeffs[0])
		/ ( state->ctrl_points[length - 1] - state->ctrl_points[0] );

	free_calibration(calib_params, calib_results);
	return;
}

void od_destroy(struct od **od)
{
	if (od == NULL || *od == NULL)
		return;
	free((*od)->state.ctrl_points);
	(*od)->state.ctrl_points = NULL;
	free((*od)->state.ctrl_drift_coeffs);
	(*od)->state.ctrl_drift_coeffs = NULL;
	free(*od);
	*od = NULL;
}

int od_get_status(struct od *od) {
	if (od == NULL)
		return -1;
	return od->state.status;
}
