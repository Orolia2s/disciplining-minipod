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
/** Smooth exponential factor for estimated equilibrium
 * used during holdover phase
 */
#define ALPHA_ES 0.1

/**
 * @struct od
 * @brief Library context.
 */
struct od {
    /** Algorith state */
    struct algorithm_state state;
    /** Algorithm paramters */
    struct parameters params;
    struct config config;
};

static int update_config(struct od *od);

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
	state->fine_ctrl_value = 0;

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
	log_debug("Control points used:");
	for (int i = 0; i < params->ctrl_nodes_length; i++) {
		state->ctrl_points[i] = (uint16_t) state->ctrl_range_fine[0] + params->ctrl_load_nodes[i] * diff_fine;
		log_debug("\t%d: %d", i, state->ctrl_points[i]);
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
	struct algorithm_state* state = &(od->state);
	struct parameters *params = &(od->params);
	log_debug("Control Check mRO:");
	if (state->calib
		&& state->estimated_equilibrium >= (uint32_t) state->ctrl_range_fine[0] + params->fine_stop_tolerance
		&& state->estimated_equilibrium <= (uint32_t) state->ctrl_range_fine[1] - params->fine_stop_tolerance
	) {
		log_info("Estimated equilibrium is in tolerance range, saving calibration in config file");
		params->coarse_equilibrium = input->coarse_setpoint;
		/* Save new drift coeffs in config file */
		int ret = update_config(od);
		if (ret < 0) {
			log_error("Error updating configuration with new drift coefficients ! Calibration must be done again next time");
		}

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

		if (abs(delta_coarse) > params->max_allowed_coarse) {
			log_info("Large coarse change %u can lead to the loss of LOCK!", delta_coarse);
			delta_coarse = delta_coarse > 0 ?
				params->max_allowed_coarse :
				-params->max_allowed_coarse;
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

	ret = fill_parameters(&od->config, &od->params, path, err_msg);
	if (ret != 0) {
		log_error("Error parsing config file !");
		return NULL;
	}

	log_set_level(
		od->params.debug >= 0 ?
		od->params.debug :
		LOG_INFO
	);
	print_parameters(&od->params);

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
		od->params.debug >= 0 ?
		od->params.debug :
		LOG_INFO
	);

	struct algorithm_state *state = &(od->state);
	struct parameters *params = &(od->params);
	log_debug("OD_PROCESS: State is %d, gnss valid is %d and mRO lock is %d",
		od->state.status, input->valid, input->lock);

	if (input->valid && input->lock)
	{
		if (params->calibrate_first || input->calibration_requested)
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
				log_debug("Control_check_mro has not been passed !");
				return 0;
			}
			log_debug("Control check mRO has been passed !");
			state->calib = false;
			state->status = INIT;

			/* Request coarse value to be saved in mRO50 memory */
			output->action = SAVE_COARSE;
			return 0;
		}

		/* Initialization */
		if (state->status == INIT)
		{
			if (params->coarse_equilibrium >= 0 && input->coarse_setpoint != params->coarse_equilibrium) {
				output->action = ADJUST_COARSE;
				output->setpoint = params->coarse_equilibrium;
				log_info("INITIALIZATION: Applying coarse equilibrium setpoint %d", params->coarse_equilibrium);
			} else {
				if (params->coarse_equilibrium < 0)
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
				log_debug("Filtered phase is %f", filtered_phase);

				double x;
				if (fabs(phase) <= params->ref_fluctuations_ns
					&& fabs(innovation) <= params->ref_fluctuations_ns)
					x = filtered_phase;
				else
					x = phase;

				if (abs(x) < params->ref_fluctuations_ns
					&& (state->fine_ctrl_value >= state->ctrl_range_fine[0]
					|| state->fine_ctrl_value <= state->ctrl_range_fine[1]))
				{
					state->estimated_equilibrium_ES =
						(int) (ALPHA_ES * state->fine_ctrl_value
						+ (1 - ALPHA_ES) * state->estimated_equilibrium_ES);
					log_info("Estimated equilibrium with exponential smooth is %d",
						state->estimated_equilibrium_ES);
				}

				double r = get_reactivity(
					fabs(x),
					params->ref_fluctuations_ns,
					params->reactivity_min,
					params->reactivity_max,
					params->reactivity_power
				);
				double react_coeff = - x / r;
				log_info("get_reactivity gives %f, react coeff is now %f", r, react_coeff);

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
					log_error("Error occured in lin_interp: %d", ret);
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
					log_warn("Control value is out of range, if convergence is not reached"
						"consider recalibration or other reactivity parameters");

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
		state->status = HOLDOVER;
		output->action = ADJUST_FINE;
		output->setpoint = state->estimated_equilibrium_ES;
	}
	return 0;
}

struct calibration_parameters * od_get_calibration_parameters(struct od *od)
{
	if (od == NULL)
	{
		log_error("Library context is null");
		return NULL;
	}

	if (od->params.ctrl_nodes_length <= 0)
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

	calib_params->ctrl_points = malloc(od->params.ctrl_nodes_length * sizeof(uint16_t));
	if (calib_params->ctrl_points == NULL) {
		log_error("Could not allocate memory to create ctrl points in calibration parameters data");
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

static int update_config(struct od *od)
{
	FILE __attribute__((cleanup(file_cleanup)))*config= NULL;
	FILE __attribute__((cleanup(file_cleanup)))*eeprom= NULL;
	char __attribute__((cleanup(string_cleanup)))*coefs_str = NULL;
	char *path = od->params.path;
	const char *eeprom_path;
	int length = od->params.ctrl_nodes_length;
	char strce[20];
	char data[1024]; // size of eeprom
	int i;
	int o;
	int ret;

	// (10*length + length-1 commas) + 1 ('\0') = 11
	// ... but 16 to be safe :) 
	coefs_str = malloc(16*length);
	o = 0; 
	for (i = 0; i < length ; i++) {
		ret = snprintf(coefs_str + o, 10, "%.6f",
			       od->params.ctrl_drift_coeffs[i]);
		if (ret >= 10) {
			log_error("float str too large\n");
			return -EINVAL;
		}
		o += ret;
		if (i != length - 1)
			coefs_str[o] = ',';
		o+=1;
	};

	sprintf(strce, "%d", od->params.coarse_equilibrium);
	config_set(&od->config, "coarse_equilibrium", strce);
	config_set(&od->config, "ctrl_drift_coeffs", coefs_str);
	config_set(&od->config, "calibrate_first", "false");
	config_dump(&od->config, data, sizeof(data));

	config = fopen(path, "w+");
	if (config == NULL) {
		log_error("Could not open file %s", path);
		return -EINVAL;
	}
	fwrite(data, 1, strlen(data)+1, config);
	fwrite("\n", 1, 1, config);

	eeprom_path = config_get(&od->config, "eeprom");
	if (eeprom_path == NULL)
		return 0;

	eeprom = fopen(eeprom_path, "w+");
	if (eeprom == NULL) {
		log_error("Could not open file %s", eeprom_path);
		return -EINVAL;
	}
	fwrite(data, 1, strlen(data)+1, eeprom);
	fwrite("\n", 1, 1, eeprom);

	return 0;
}

void od_calibrate(struct od *od, struct calibration_parameters *calib_params, struct calibration_results *calib_results)
{
	int ret;
	int length;

	if (od == NULL || calib_params == NULL || calib_results == NULL)
	{
		log_error("od_calibration: at least one input parameter is null");
		return;
	}

	od->state.status = CALIBRATION;

	if (calib_params->length != od->params.ctrl_nodes_length || calib_params->length != calib_results->length)
	{
		log_error("od_calibrate: length mismatch");
		free_calibration(calib_params, calib_results);
		return;
	}
	length = od->params.ctrl_nodes_length;

	if (calib_params->nb_calibration != calib_results->nb_calibration)
	{
		log_error("od_calibrate: nb_calibration mismatch");
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
	log_debug("CALIBRATION: Updating drift coefficients:");
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
			log_error("od_calibrate: error occured in simple_linear_reg, err %d", ret);
			free_calibration(calib_params, calib_results);
			return;
		}
		od->params.ctrl_drift_coeffs[i] = func_params.a;
		log_debug("\t[%d]: %f", i, od->params.ctrl_drift_coeffs[i]);
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
		log_error("od_calibrate: error occured in lin_interp, err %d", ret);
	}

	od->state.estimated_equilibrium = (uint32_t) interp_value;
	log_debug("Estimated equilibrium is now %d", od->state.estimated_equilibrium);

	if (od->state.ctrl_points[length - 1] - od->state.ctrl_points[0] == 0)
	{
		log_error("Control points cannot have the same value !");
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
	config_cleanup(&((*od)->config));
	free(*od);
	*od = NULL;
}

int od_get_status(struct od *od) {
	if (od == NULL)
		return -1;
	return od->state.status;
}
