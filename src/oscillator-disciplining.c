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

#include "algorithm_structs.h"
#include "checks.h"
#include "log.h"
#include "parameters.h"
#include "phase.h"
#include "utils.h"

#define SETTLING_TIME_MRO50 6
#define WINDOW_TRACKING 6
#define WINDOW_LOCK_LOW_RESOLUTION 66
#define LOCK_LOW_RESOLUTION_PHASE_CONVERGENCE_REACTIVITY 10 * WINDOW_LOCK_LOW_RESOLUTION
#define WINDOW_LOCK_HIGH_RESOLUTION 606
#define LOCK_HIGH_RESOLUTION_PHASE_CONVERGENCE_REACTIVITY 10 * WINDOW_LOCK_HIGH_RESOLUTION

uint16_t state_windows[NUM_STATES] = {
	WINDOW_TRACKING,
	WINDOW_TRACKING,
	WINDOW_TRACKING,
	WINDOW_TRACKING,
	WINDOW_LOCK_LOW_RESOLUTION,
	WINDOW_LOCK_HIGH_RESOLUTION
};

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

static void set_state(struct algorithm_state *state, enum Disciplining_State new_state)
{
	state->status = new_state;
	state->od_inputs_for_state = state_windows[new_state];
	state->od_inputs_count = 0;
	state->current_phase_convergence_count = 0;
}

static void set_output(struct od_output *output, enum output_action action, uint32_t setpoint, int32_t value_phase_ctrl)
{
	output->action = action;
	output->setpoint = setpoint;
	output->value_phase_ctrl = value_phase_ctrl;
	return;
}

static int init_ctrl_points(struct algorithm_state *state, float *load_nodes, float *drift_coeffs, uint8_t length) {
	uint16_t diff_fine;
	int i;

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
	for (i = 0; i < length; i++) {
		state->ctrl_points[i] = (float) state->ctrl_range_fine[0] + load_nodes[i] * diff_fine;
		state->ctrl_drift_coeffs[i] = drift_coeffs[i];
		log_debug("\t%d: %f -> %f", i, state->ctrl_points[i], state->ctrl_drift_coeffs[i]);
	}
	state->ctrl_points_length = length;
	return 0;
}

static int compute_fine_value(struct algorithm_state *state, float react_coeff, uint16_t *fine_ctrl_value)
{
	float interpolation_value;
	int ret;
	ret = lin_interp(
		state->ctrl_points,
		state->ctrl_drift_coeffs,
		state->ctrl_points_length,
		Y_INTERPOLATION,
		react_coeff,
		&interpolation_value
	);
	if (ret < 0)
	{
		log_error("Error occured in lin_interp: %d", ret);
		return -1;
	}
	/* If linear interpretation returns a negative value, consider value found is 0 before conversion to integers */
	if (interpolation_value <= 0.0) {
		log_warn("fine control value found is negative (%f), setting to 0.0", interpolation_value);
		interpolation_value = 0.0;
	}

	*fine_ctrl_value = (uint16_t) round(interpolation_value);
	return 0;
}

static void print_inputs(struct algorithm_input *inputs, int length)
{
	int i;
	if (!inputs || length <= 0)
		return;
	char * str = calloc((length + 2) * 16, sizeof(char));
	strcat(str, "Inputs: [");
	for (i = 0; i < length; i++) {
		char float_num[14];
		sprintf(float_num, "%.3f", inputs[i].phase_error);
		strcat(str, float_num);
		if (i < length -1)
		strcat(str, ", ");
	}
	strcat(str, "]");
	log_info(str);
	free(str);
	str = NULL;
}

static int init_algorithm_state(struct od * od) {
	int ret;

	struct algorithm_state *state = &od->state;
	struct disciplining_parameters *dsc_parameters = &od->dsc_parameters;
	struct minipod_config *config = &od->minipod_config;

	/* Init state variables */
	set_state(state, INIT);
	state->calib = false;

	state->mRO_fine_step_sensitivity = MRO_FINE_STEP_SENSITIVITY;
	state->mRO_coarse_step_sensitivity = MRO_COARSE_STEP_SENSITIVITY;

	state->ctrl_range_coarse[0] = COARSE_RANGE_MIN;
	state->ctrl_range_coarse[1] = COARSE_RANGE_MAX;
	state->ctrl_range_fine[0] = FINE_MID_RANGE_MIN;
	state->ctrl_range_fine[1] = FINE_MID_RANGE_MAX;

	state->fine_mid = (uint16_t) (0.5 * (FINE_MID_RANGE_MIN + FINE_MID_RANGE_MAX));
	state->fine_ctrl_value = 0;

	state->estimated_drift = 0;
	state->current_phase_convergence_count = 0;
	state->previous_freq_error = 0.0;

	/* Kalman filter parameters */
	state->kalman.Ksigma = config->ref_fluctuations_ns;
	state->kalman.Kphase_set = false;
	state->kalman.Kphase = 0.0;
	state->kalman.q = 1.0;
	state->kalman.r = 5.0;

	/* Init Alpha Equilibrium smooth */
	state->alpha_es_tracking = 0.012; //od->minipod_config.alpha_global * WINDOW_TRACKING;
	state->alpha_es_lock_low_res = 0.06; //od->minipod_config.alpha_global * WINDOW_LOCK_LOW_RESOLUTION;
	state->alpha_es_lock_high_res = 0.3; //od->minipod_config.alpha_global * WINDOW_LOCK_HIGH_RESOLUTION;

	/* Allocate memory for algorithm inputs */
	state->inputs = (struct algorithm_input*) malloc(WINDOW_LOCK_HIGH_RESOLUTION * sizeof(struct algorithm_input));
	if (!state->inputs) {
		log_error("Could not allocate memory for algorithm inputs !");
		return -1;
	}

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

	ret = compute_fine_value(state, 0, &state->estimated_equilibrium);
	if (ret != 0) {
		log_error("Error computing fine value from control drift coefficients");
		return ret;
	}
	if (dsc_parameters->estimated_equilibrium_ES != 0)
		state->estimated_equilibrium_ES = dsc_parameters->estimated_equilibrium_ES;
	else
		state->estimated_equilibrium_ES = state->estimated_equilibrium;
	log_info("Initialization: Estimated equilibrium is %d and estimated equilibrium ES is %d", state->estimated_equilibrium, state->estimated_equilibrium_ES);
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
				set_output(output, CALIBRATE, 0, 0);
				return false;
			}
		}
		log_debug("Calibration coefficients are descending with fine values");
		for(i = 0; i < state->ctrl_points_length; i++) {
			if (fabs(state->ctrl_drift_coeffs[i]) > DRIFT_COEFFICIENT_ABSOLUTE_MAX) {
				log_warn("ctrl_drift_coeffs[%d] coefficient is greater than %f in absolute: %f",
					i, DRIFT_COEFFICIENT_ABSOLUTE_MAX, state->ctrl_drift_coeffs[i]);
				set_output(output, CALIBRATE, 0, 0);
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

		set_output(output, ADJUST_FINE, input->coarse_setpoint + delta_coarse, 0);
		log_info("Requesting a coarse alignement to value %d", output->setpoint);
		return false;
	} else {
		/* mRO needs to be calibrated */
		set_output(output, CALIBRATE, 0, 0);
		return false;
	}
}

static float get_reactivity(float phase_ns, int sigma, int min, int max, int power) {
	float r = max * exp(-pow(phase_ns/sigma, power));
	return r > min ? r : min;
}

static void add_input_to_algorithm(struct algorithm_input *algorithm_input, const struct od_input *input)
{
	algorithm_input->phase_error = input->phase_error.tv_nsec + (float) input->qErr / PS_IN_NS;
	algorithm_input->valid = input->valid;
	algorithm_input->lock = input->lock;
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
	set_output(output, NO_OP, 0, 0);

	/* Add new algorithm input */
	add_input_to_algorithm(&state->inputs[state->od_inputs_count], input);
	log_debug("input: phase_error: %d, qErr: %d", input->phase_error.tv_nsec,input->qErr);
	log_debug("INPUT[%d] = %f",
		state->od_inputs_count,
		state->inputs[state->od_inputs_count].phase_error
	);
	state->od_inputs_count++;

	log_debug("OD_PROCESS: State is %s, Conv. Step %u, (%u/%u), GNSS valid: %s and mRO lock: %s",
		od->state.status == INIT ? "INIT" :
		od->state.status == TRACKING ? "TRACKING" :
		od->state.status == HOLDOVER ? "HOLDOVER" :
		od->state.status == CALIBRATION ? "CALIBRATION" :
		od->state.status == LOCK_LOW_RESOLUTION ? "LOCK_LOW_RESOLUTION":
		od->state.status == LOCK_HIGH_RESOLUTION ?"LOCK_HIGH_RESOLUTION":
		"UNKNOWN_STATE",
		state->current_phase_convergence_count,
		state->od_inputs_count,
		state->od_inputs_for_state,
		input->valid ? "True" : "False", input->lock ? "True" : "False");


	if (state->od_inputs_count == state->od_inputs_for_state) {
		state->od_inputs_count = 0;

		if (check_gnss_valid_over_cycle(state->inputs, state->od_inputs_for_state)
			&& check_lock_over_cycle(state->inputs, state->od_inputs_for_state))
		{
			if (od->state.status != CALIBRATION
				&& (
					config->calibrate_first
					|| input->calibration_requested
				)
			)
			{
				config->calibrate_first = false;
				set_output(output, CALIBRATE, 0, 0);
				/* FIXME !*/
				od->state.status = CALIBRATION;
				return 0;

			}

			float mean_phase_error;
			switch(state->status) {
			/* Calibration: calibration data is available and control check must be done */
			case CALIBRATION:
				/** Need to check mRO control values
				 * If calibration has been done, we need to make a control check of the mRO
				 */
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
				set_state(state, INIT);

				/* Request coarse value to be saved in mRO50 memory */
				set_output(output, SAVE_DISCIPLINING_PARAMETERS, 0, 0);
				return 0;
				break;

			/* Initialization */
			case INIT:
				if (config->oscillator_factory_settings
					&& dsc_parameters->coarse_equilibrium_factory >= 0
					&& input->coarse_setpoint != dsc_parameters->coarse_equilibrium_factory)
				{
					set_output(output, ADJUST_COARSE, dsc_parameters->coarse_equilibrium_factory, 0);
					log_info("INITIALIZATION: Applying factory coarse equilibrium setpoint %d", dsc_parameters->coarse_equilibrium);
				} else if (!config->oscillator_factory_settings
					&& dsc_parameters->coarse_equilibrium >= 0
					&& input->coarse_setpoint != dsc_parameters->coarse_equilibrium)
				{
					set_output(output, ADJUST_COARSE, dsc_parameters->coarse_equilibrium, 0);
					log_info("INITIALIZATION: Applying coarse equilibrium setpoint %d", dsc_parameters->coarse_equilibrium);
				} else {
					if (dsc_parameters->coarse_equilibrium < 0)
						log_warn("Unknown coarse_equilibrium, using value saved in oscillator,"
							"consider calibration if disciplining is not efficient");
					set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
					set_state(state, TRACKING);
					log_info("INITIALIZATION: Applying estimated fine equilibrium setpoint %d", state->estimated_equilibrium);
				}
				return 0;
				break;

			/* Holdover and valid flag switched to valid,
			* We wait for one cycle to start disciplining again
			*/
			case HOLDOVER:
				set_state(state, TRACKING);
				set_output(output, ADJUST_FINE, state->estimated_equilibrium, 0);
				log_info("HOLDOVER: Gnss flag valid again, waiting one cycle before restarting disciplining");
				break;

			case TRACKING:
				print_inputs(state->inputs, WINDOW_TRACKING);
				/* Compute mean phase error over cycle */
				ret = compute_phase_error_mean(state->inputs, state->od_inputs_for_state, &mean_phase_error);
				if (ret != 0) {
					log_error("Mean phase error could be computed");
					set_state(state, HOLDOVER);
					set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
					return 0;
				}
				/* Check phase error is below threshold configured */
				if (fabs(mean_phase_error) < (float) config->phase_jump_threshold_ns)
				{
					/* Call Main loop */
					if (!check_no_outlier(state->inputs, state->od_inputs_for_state,
						mean_phase_error, config->ref_fluctuations_ns))
					{
						log_warn("Outlier detected ! entering holdover");
						set_state(state, HOLDOVER);
						set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
						return 0;
					}

					/* Check phase error is not exceeding maximal drift and that there is not outlier value in cycla */
					// if (!check_max_drift((struct od_input *) state->inputs, 7)) {
					// 	output->action = CALIBRATE;
					// 	od->state.status = CALIBRATION;
					// 	return 0;
					// }

					/* Legacy code used for debug */
					// float filtered_phase = filter_phase(
					// 	&(state->kalman),
					// 	mean_phase_error,
					// 	SETTLING_TIME,
					// 	state->estimated_drift
					// );
					// log_info("Filtered phase is %f", filtered_phase);

					/* Phase error is below reference and control value in midrange */
					if (fabs(mean_phase_error) < config->ref_fluctuations_ns
						&& (state->fine_ctrl_value >= state->ctrl_range_fine[0]
						&& state->fine_ctrl_value <= state->ctrl_range_fine[1])
						&& fabs(state->inputs[WINDOW_TRACKING].phase_error - state->inputs[0].phase_error)
						< (float) config->ref_fluctuations_ns)
					{
						if (state->current_phase_convergence_count <= round(1.0 / state->alpha_es_tracking)) {
							log_debug("fast smoothing convergence : 2.0 * %f applied", state->alpha_es_tracking);
							state->estimated_equilibrium_ES =
								round((2.0 * state->alpha_es_tracking * state->fine_ctrl_value
								+ (1.0 - (2.0 * state->alpha_es_tracking)) * state->estimated_equilibrium_ES));
						} else {
							state->estimated_equilibrium_ES =
								round((state->alpha_es_tracking * state->fine_ctrl_value
								+ (1.0 - state->alpha_es_tracking) * state->estimated_equilibrium_ES));
						}
						state->current_phase_convergence_count++;
						if (state->current_phase_convergence_count  == UINT16_MAX)
							state->current_phase_convergence_count = round(6.0 / state->alpha_es_tracking);
					}
					log_info("Estimated equilibrium with exponential smooth is %d",
						state->estimated_equilibrium_ES);
					log_debug("convergence_count: %d", state->current_phase_convergence_count);

					float r = get_reactivity(
						fabs(mean_phase_error),
						config->ref_fluctuations_ns,
						config->reactivity_min,
						config->reactivity_max,
						config->reactivity_power
					);
					float react_coeff = - mean_phase_error / r;
					log_info("get_reactivity gives %f, react coeff is now %f", r, react_coeff);

					ret = compute_fine_value(state, react_coeff, &state->fine_ctrl_value);
					if (ret != 0) {
						log_error("Error computing fine value");
						return ret;
					}
					log_debug("New fine control value: %u", state->fine_ctrl_value);

					if (state->current_phase_convergence_count > round(6.0 / state->alpha_es_tracking)
						&& state->estimated_equilibrium_ES >= (uint32_t) FINE_MID_RANGE_MIN + config->fine_stop_tolerance
						&& state->estimated_equilibrium_ES <= (uint32_t) FINE_MID_RANGE_MAX - config->fine_stop_tolerance)
					{
						/* Update estimated equilibrium ES in discplining parameters */
						od->dsc_parameters.estimated_equilibrium_ES = state->estimated_equilibrium_ES;
						/* Smooth convergence reached, adjust to estimated equilibrium smooth */
						log_info("Smoothing convergence reached");
						state->estimated_drift = react_coeff;
						set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);

						/* Switch to LOCK_LOW_RESOLUTION_STATE */
						set_state(state, LOCK_LOW_RESOLUTION);
						return 0;
					} else if (state->current_phase_convergence_count > 2 * round(6.0 / state->alpha_es_tracking)) {
						log_warn("Estimated equilibrium is out of range !");
						uint32_t new_coarse = input->coarse_setpoint;
						if (state->estimated_equilibrium_ES < (uint32_t) FINE_MID_RANGE_MIN + config->fine_stop_tolerance)
							new_coarse = input->coarse_setpoint + 1;
						else if (state->estimated_equilibrium_ES > (uint32_t) FINE_MID_RANGE_MAX - config->fine_stop_tolerance)
							new_coarse = input->coarse_setpoint - 1;
						log_info("Adjusting coarse value to %u", new_coarse);
						set_output(output, ADJUST_COARSE, new_coarse, 0);

						/* Reset Tracking state */
						set_state(state, TRACKING);

						/* Update estimated equilibrium ES to initial guess */
						if (dsc_parameters->estimated_equilibrium_ES != 0)
							state->estimated_equilibrium_ES = dsc_parameters->estimated_equilibrium_ES;
						else
							state->estimated_equilibrium_ES = state->estimated_equilibrium;

						if (config->oscillator_factory_settings)
							dsc_parameters->coarse_equilibrium_factory = new_coarse;
						else
							dsc_parameters->coarse_equilibrium = new_coarse;
						return 0;
					}
					
					/* current_phase_convergence_count is below current_phase_convergence_count_threshold*/
					if (state->fine_ctrl_value >= FINE_RANGE_MIN + config->fine_stop_tolerance
						&& state->fine_ctrl_value <= FINE_RANGE_MAX - config->fine_stop_tolerance)
					{
						state->estimated_drift = react_coeff;
						set_output(output, ADJUST_FINE, state->fine_ctrl_value, 0);
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
						set_output(output, ADJUST_FINE, stop_value, 0);
					}
					return 0;
				} else {
					if (state->current_phase_convergence_count <= round(6.0 / state->alpha_es_tracking)) {
						/* Phase jump needed */
						set_output(output, PHASE_JUMP, 0, input->phase_error.tv_nsec);
					} else {
						set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
					}
					return 0;
				}
				break;
			case LOCK_LOW_RESOLUTION:
			{
				state->current_phase_convergence_count++;
				log_debug("convergence_count: %d", state->current_phase_convergence_count);
				if (state->current_phase_convergence_count  == UINT16_MAX)
					state->current_phase_convergence_count = round(6.0 / state->alpha_es_lock_low_res);
				print_inputs(&(state->inputs[SETTLING_TIME_MRO50]), WINDOW_LOCK_LOW_RESOLUTION - SETTLING_TIME_MRO50);

				/* Check that estimated equilibrium is within acceptable range */
				if (state->estimated_equilibrium_ES < (uint32_t) FINE_MID_RANGE_MIN + config->fine_stop_tolerance ||
					state->estimated_equilibrium_ES > (uint32_t) FINE_MID_RANGE_MAX - config->fine_stop_tolerance) {
					log_warn("Estimated equilibrium is out of range !");
					uint32_t new_coarse = input->coarse_setpoint;
					if (state->estimated_equilibrium_ES < (uint32_t) FINE_MID_RANGE_MIN + config->fine_stop_tolerance)
						new_coarse = input->coarse_setpoint + 1;
					else if (state->estimated_equilibrium_ES > (uint32_t) FINE_MID_RANGE_MAX - config->fine_stop_tolerance)
						new_coarse = input->coarse_setpoint - 1;
					log_info("Adjusting coarse value to %u", new_coarse);
					set_output(output, ADJUST_COARSE, new_coarse, 0);

					/* Switch to Tracking state */
					set_state(state, TRACKING);

					/* Update estimated equilibrium ES to initial guess*/
					if (dsc_parameters->estimated_equilibrium_ES != 0)
						state->estimated_equilibrium_ES = dsc_parameters->estimated_equilibrium_ES;
					else
						state->estimated_equilibrium_ES = state->estimated_equilibrium;

					if (config->oscillator_factory_settings)
						dsc_parameters->coarse_equilibrium_factory = new_coarse;
					else
						dsc_parameters->coarse_equilibrium = new_coarse;
					return 0;
				}


				/* Compute mean phase error over cycle */
				ret = compute_phase_error_mean(
					&(state->inputs[SETTLING_TIME_MRO50]),
					WINDOW_LOCK_LOW_RESOLUTION - SETTLING_TIME_MRO50,
					&mean_phase_error
				);
				if (ret != 0) {
					log_error("Mean phase error could not be computed");
					set_state(state, HOLDOVER);
					set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
					return 0;
				}

				if (!check_no_outlier(state->inputs, state->od_inputs_for_state,
					mean_phase_error, config->ref_fluctuations_ns))
				{
					log_warn("Outlier detected ! Adjust to equilibrium");
					set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
					return 0;
				}
				/* Compute frequency error */
				struct linear_func_param func;
				ret = compute_frequency_error(
					&(state->inputs[SETTLING_TIME_MRO50]),
					WINDOW_LOCK_LOW_RESOLUTION - SETTLING_TIME_MRO50,
					&func
				);
				if (ret != 0) {
					log_error("Error computing frequency_error and standard deviation");
				}
				double R2 = func.R2;
				double t0 = func.t0;
				double frequency_error = func.a;
				double frequency_error_std = func.a_std;
				log_debug("Frequency Error: %f, STD: %f, R2: %f, t0: %f", frequency_error, frequency_error_std, R2, t0);
				float current_freq_error = frequency_error;
				// t-test threshold for 99% confidence level null slope with 60-2 degrees of freedom
				// must be changed if lock windows size changes or used from a table
				float t9995_ndf58 = 3.467;

				if ((R2 > R2_THRESHOLD_LOW_RESOLUTION) || (t0 < t9995_ndf58)) {
					log_debug("Current frequency estimate is %f +/- %f", frequency_error, frequency_error_std);
					if (fabs(frequency_error) > LOCK_LOW_RES_FREQUENCY_ERROR_MAX) {
						log_warn("Strong drift detected");

						/* We authorize such strong drift at first step of the phase */
						if (state->current_phase_convergence_count > 1
							&& state->current_phase_convergence_count < round(6.0 / state->alpha_es_lock_low_res)) {
							log_warn("Applying estimated equilibrium");
							set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
							return 0;
						} else if (state->current_phase_convergence_count > round(6.0 / state->alpha_es_lock_low_res) && mean_phase_error > 2 * config->ref_fluctuations_ns) {
							set_state(state, TRACKING);
							set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
							return 0;
						}
					}

					float coeff = 0.0;
					/* Compensate pure frequency error only */
					if (frequency_error_std < fabs(frequency_error) && fabs(frequency_error) > fabs((MRO_FINE_STEP_SENSITIVITY * 1.E9))){
						coeff = 1.0 - fabs(frequency_error_std/frequency_error);
						coeff = coeff > 0.9 ? 0.9 : coeff;
					}
					log_debug("Pure frequency coefficients: %f", coeff);
					int16_t delta_fine = -round(coeff * frequency_error / (MRO_FINE_STEP_SENSITIVITY * 1.E9));

					/* Compensate phase error */
					float frequency_error_pcorr = 0.0;
					int16_t delta_fine_pcorr = 0;
					if (fabs(mean_phase_error) >= config->ref_fluctuations_ns) {
						frequency_error_pcorr = - mean_phase_error / (LOCK_LOW_RESOLUTION_PHASE_CONVERGENCE_REACTIVITY);
						if (fabs(frequency_error_pcorr) > fabs((MRO_FINE_STEP_SENSITIVITY * 1.E9)))
							delta_fine_pcorr = round(frequency_error_pcorr / (MRO_FINE_STEP_SENSITIVITY * 1.E9)); // check sign !!
					}
					log_debug("frequency_error_pcorr: %f", frequency_error_pcorr);

					log_debug("delta_fine (pure frequency): %d, delta_fine_pcorr: %d", delta_fine, delta_fine_pcorr);
					delta_fine += delta_fine_pcorr;
					log_debug("Sum delta fine: %d", delta_fine);

					if (abs(delta_fine) > LOCK_LOW_RES_FINE_DELTA_MAX) {
						delta_fine = delta_fine < 0 ?
							-LOCK_LOW_RES_FINE_DELTA_MAX :
							LOCK_LOW_RES_FINE_DELTA_MAX;
					}

					if ((delta_fine > 0) &&
						(fabs(state->previous_freq_error) > fabs((MRO_FINE_STEP_SENSITIVITY * 1.E9))) &&
						(fabs(current_freq_error) > fabs((MRO_FINE_STEP_SENSITIVITY * 1.E9))) &&
						(current_freq_error * state->previous_freq_error < 0) &&
						(fabs(mean_phase_error) < config->ref_fluctuations_ns)

					) {
						log_debug("frequency sign change since last cycle (%f, %f), 0.5*delta_fine" , state->previous_freq_error, current_freq_error);
						delta_fine = round(0.5*delta_fine);
					}
					state->previous_freq_error = current_freq_error;

					uint16_t new_fine;
					if (input->fine_setpoint + delta_fine < FINE_MID_RANGE_MIN)
						new_fine = FINE_MID_RANGE_MIN;
					else if(input->fine_setpoint + delta_fine > FINE_MID_RANGE_MAX)
						new_fine = FINE_MID_RANGE_MAX;
					else
						new_fine = input->fine_setpoint + delta_fine;
					log_debug("NEW FINE value: %u", new_fine);

					uint16_t new_fine_from_calib;
					ret = compute_fine_value(state, coeff*frequency_error, &new_fine_from_calib);
					if (ret != 0) {
						log_error("Could not compute fine value for log");
					} else {
						log_debug("new_fine_from_calib: %u", new_fine_from_calib);
					}

					/* Apply computed fine  */
					set_output(output, ADJUST_FINE, new_fine, 0);
					/* Update estimated equilibrium */
					state->estimated_equilibrium_ES =
						round((state->alpha_es_lock_low_res * new_fine
						+ (1.0 - state->alpha_es_lock_low_res) * state->estimated_equilibrium_ES));
					log_info("Estimated equilibrium with exponential smooth is %d",
						state->estimated_equilibrium_ES);
					/* Update estimated equilibrium ES in discplining parameters */
					od->dsc_parameters.estimated_equilibrium_ES = state->estimated_equilibrium_ES;

					/* Check wether high resolution has been reached */
					if (fabs(frequency_error) < LOCK_LOW_RES_FREQUENCY_ERROR_MIN &&
						abs(delta_fine) <= LOCK_LOW_RES_FREQUENCY_ERROR_MIN / fabs((MRO_FINE_STEP_SENSITIVITY * 1.E9)) &&
						state->current_phase_convergence_count > round(6.0 / state->alpha_es_lock_low_res) && fabs(mean_phase_error) < 1.5*config->ref_fluctuations_ns ) {
						log_info("Low frequency error reached, entering LOCK_HIGH_RESOLUTION");
						set_state(state, LOCK_HIGH_RESOLUTION);
						return 0;
					}

					if (state->current_phase_convergence_count > 5 * round(6.0 / state->alpha_es_lock_low_res)) {
						log_warn("No high resolution convergence reached after %d cycles", state->current_phase_convergence_count);
					}


				} else {
					log_warn("Low linear fit quality, applying estimated equilibrium");
					set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
				}
				break;
			}
			case LOCK_HIGH_RESOLUTION:
			{   
				state->current_phase_convergence_count++;
				log_debug("convergence_count: %d", state->current_phase_convergence_count);
				if (state->current_phase_convergence_count  == UINT16_MAX)
					state->current_phase_convergence_count = round(6.0 / state->alpha_es_lock_high_res);
				print_inputs(&(state->inputs[SETTLING_TIME_MRO50]), WINDOW_LOCK_HIGH_RESOLUTION - SETTLING_TIME_MRO50);

				/* Check that estimated equilibrium is within acceptable range */
				if (state->estimated_equilibrium_ES < (uint32_t) FINE_MID_RANGE_MIN + config->fine_stop_tolerance ||
					state->estimated_equilibrium_ES > (uint32_t) FINE_MID_RANGE_MAX - config->fine_stop_tolerance) {
					log_warn("Estimated equilibrium is out of range !");
					uint32_t new_coarse = input->coarse_setpoint;
					if (state->estimated_equilibrium_ES < (uint32_t) FINE_MID_RANGE_MIN + config->fine_stop_tolerance)
						new_coarse = input->coarse_setpoint + 1;
					else if (state->estimated_equilibrium_ES > (uint32_t) FINE_MID_RANGE_MAX - config->fine_stop_tolerance)
						new_coarse = input->coarse_setpoint - 1;
					log_info("Adjusting coarse value to %u", new_coarse);
					set_output(output, ADJUST_COARSE, new_coarse, 0);

					/* Switch to Tracking state */
					set_state(state, TRACKING);

					/* Update estimated equilibrium ES to initial guess*/
					if (dsc_parameters->estimated_equilibrium_ES != 0)
						state->estimated_equilibrium_ES = dsc_parameters->estimated_equilibrium_ES;
					else
						state->estimated_equilibrium_ES = state->estimated_equilibrium;

					if (config->oscillator_factory_settings)
						dsc_parameters->coarse_equilibrium_factory = new_coarse;
					else
						dsc_parameters->coarse_equilibrium = new_coarse;
					return 0;
				}

				/* Compute mean phase error over cycle */
				ret = compute_phase_error_mean(
					&(state->inputs[SETTLING_TIME_MRO50]),
					WINDOW_LOCK_HIGH_RESOLUTION - SETTLING_TIME_MRO50,
					&mean_phase_error
				);
				if (ret != 0) {
					log_error("Mean phase error could not be computed");
					set_state(state, HOLDOVER);
					set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
					return 0;
				}

				if (!check_no_outlier(state->inputs, state->od_inputs_for_state,
					mean_phase_error, config->ref_fluctuations_ns))
				{
					log_warn("Outlier detected ! Adjust to equilibrium");
					set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
					return 0;
				}

				/* Compute frequency error */
				struct linear_func_param func;
				ret = compute_frequency_error(
					&(state->inputs[SETTLING_TIME_MRO50]),
					WINDOW_LOCK_HIGH_RESOLUTION - SETTLING_TIME_MRO50,
					&func
				);
				if (ret != 0) {
					log_error("Error computing frequency_error and standard deviation");
					/* FIXME */
				}
				double R2 = func.R2;
				double t0 = func.t0;
				double frequency_error = func.a;
				double frequency_error_std = func.a_std;
				log_debug("Frequency Error: %f, STD: %f, R2: %f, t0: %f", frequency_error, frequency_error_std, R2, t0);

				// t-test threshold for 99% confidence level null slope with 600-2 degrees of freedom
				// must be changed if lock windows size changes or used from a table
				float t995_ndf598 = 3.39;

				if ((R2 > R2_THRESHOLD_HIGH_RESOLUTION) || (t0 < t995_ndf598)) {
					log_debug("Current frequency estimate is %f +/- %f", frequency_error, frequency_error_std);
					float current_freq_error = frequency_error;
					if (fabs(frequency_error) > LOCK_HIGH_RES_FREQUENCY_ERROR_MAX) {
						log_warn("Strong drift detected");

						/* We authorize such strong drift at first step of the phase */
						if (state->current_phase_convergence_count > 1) {
							/* TODO: More elaborate exit conditions depending on mean phase error*/
							log_warn("Applying estimated equilibrium");
							set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
							return 0;
						} else if (fabs(mean_phase_error) > 2.5 * config->ref_fluctuations_ns) {
							set_state(state, LOCK_LOW_RESOLUTION);
							set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
							return 0;
						}
					}

					float coeff = 0.0;
					/* Compensate pure frequency error only */
					if (frequency_error_std < fabs(frequency_error) && fabs(frequency_error) > fabs((MRO_FINE_STEP_SENSITIVITY * 1.E9))) {
						coeff = 1.0 - fabs(frequency_error_std/frequency_error);
						coeff = coeff > 0.9 ? 0.9 : coeff;
					}
					log_debug("Pure frequency coefficients: %f", coeff);
					int16_t delta_fine = -round(coeff * frequency_error / (MRO_FINE_STEP_SENSITIVITY * 1.E9));

					/* Compensate phase error */
					float frequency_error_pcorr = 0.0;
					int16_t delta_fine_pcorr = 0;
					if (fabs(mean_phase_error) >= config->ref_fluctuations_ns) {
						frequency_error_pcorr = - mean_phase_error / (LOCK_HIGH_RESOLUTION_PHASE_CONVERGENCE_REACTIVITY);
						if (fabs(frequency_error_pcorr) > fabs((MRO_FINE_STEP_SENSITIVITY * 1.E9)))
							delta_fine_pcorr = round(frequency_error_pcorr / (MRO_FINE_STEP_SENSITIVITY * 1.E9));
					}
					log_debug("frequency_error_pcorr: %f", frequency_error_pcorr);

					log_debug("delta_fine (pure frequency): %d, delta_fine_pcorr: %d", delta_fine, delta_fine_pcorr);
					delta_fine += delta_fine_pcorr;
					log_debug("Sum delta fine: %d", delta_fine);

					if (abs(delta_fine) > LOCK_HIGH_RES_FINE_DELTA_MAX) {
						delta_fine = delta_fine < 0 ?
							-LOCK_HIGH_RES_FINE_DELTA_MAX :
							LOCK_HIGH_RES_FINE_DELTA_MAX;
					}

					if ((delta_fine > 0) &&
						(fabs(state->previous_freq_error) > fabs((MRO_FINE_STEP_SENSITIVITY * 1.E9))) &&
						(fabs(current_freq_error) > fabs((MRO_FINE_STEP_SENSITIVITY * 1.E9))) &&
						(current_freq_error * state->previous_freq_error < 0) &&
						(fabs(mean_phase_error) < config->ref_fluctuations_ns)

					) {
						log_debug("frequency sign change since last cycle (%f, %f), 0.5*delta_fine" , state->previous_freq_error, current_freq_error);
						delta_fine = round(0.5*delta_fine);
					}
					state->previous_freq_error = current_freq_error;

					uint16_t new_fine;
					if (input->fine_setpoint + delta_fine < FINE_MID_RANGE_MIN)
						new_fine = FINE_MID_RANGE_MIN;
					else if(input->fine_setpoint + delta_fine > FINE_MID_RANGE_MAX)
						new_fine = FINE_MID_RANGE_MAX;
					else
						new_fine = input->fine_setpoint + delta_fine;
					log_debug("NEW FINE value: %u", new_fine);

					uint16_t new_fine_from_calib;
					ret = compute_fine_value(state, coeff*frequency_error, &new_fine_from_calib);
					if (ret != 0) {
						log_error("Could not compute fine value for log");
					} else {
						log_debug("new_fine_from_calib: %u", new_fine_from_calib);
					}

					/* Apply computed fine  */
					set_output(output, ADJUST_FINE, new_fine, 0);
					/* Update estimated equilibrium */
					state->estimated_equilibrium_ES =
						round((state->alpha_es_lock_high_res * new_fine
						+ (1.0 - state->alpha_es_lock_high_res) * state->estimated_equilibrium_ES));
					log_info("Estimated equilibrium with exponential smooth is %d",
						state->estimated_equilibrium_ES);
					/* Update estimated equilibrium ES in discplining parameters */
					od->dsc_parameters.estimated_equilibrium_ES = state->estimated_equilibrium_ES;
				} else {
					log_warn("Low linear fit quality, applying estimated equilibrium");
					set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
				}
				break;
			}
			default:
				log_error("Unhandled state %d", state->status);
				return -1;
			}
		} else {
			log_warn("HOLDOVER activated: GNSS data is not valid and/or oscillator's lock has been lost");
			log_info("Applying estimated equilibrium until going out of holdover");
			set_state(state, HOLDOVER);
			set_output(output, ADJUST_FINE, state->estimated_equilibrium_ES, 0);
		}
	} else {
		set_output(output, NO_OP, 0, 0);
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
	int i;

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

	for (i = 0; i < od->dsc_parameters.ctrl_nodes_length; i++)
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
	struct disciplining_parameters *dsc_parameters;
	struct algorithm_state *state;
	int length;
	int i, j;
	int ret;

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
	for (i = 0; i < calib_params->nb_calibration; i++)
		x[i] = (float) i;

	/* Update drift coefficients */
	log_debug("CALIBRATION: Updating drift coefficients:");
	for (i = 0; i < length; i++)
	{
		float v[calib_params->nb_calibration];
		for(j = 0; j < calib_params->nb_calibration; j++)
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

	for (i = 0; i < length; i++)
		state->ctrl_drift_coeffs[i] = dsc_parameters->ctrl_drift_coeffs[i];

	ret = compute_fine_value(state, 0, &state->estimated_equilibrium);
	if (ret != 0) {
		log_error("Error computing fine value");
		return;
	}
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
	free((*od)->state.inputs);
	(*od)->state.inputs = NULL;
	free(*od);
	*od = NULL;
}

int od_get_status(struct od *od) {
	if (od == NULL)
		return -1;
	return od->state.status;
}
