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

#ifndef PARAMETERS_H
#define PARAMETERS_H
/**
 * @file parameters.h
 * @brief Functions to print config and/or parameters of the disciplining library.
 */

#include <stdio.h>

#include <oscillator-disciplining/oscillator-disciplining.h>
#include "algorithm_structs.h"
#include "log.h"

static inline void print_minipod_config(struct minipod_config *config) {
	log_debug("Minipod Config:");
	log_debug("\t- debug: %s", config->debug ? "true" : "false");
	log_debug("\t- ref_fluctuations_ns: %d", config->ref_fluctuations_ns);
	log_debug("\t- phase_jump_threshold_ns: %d", config->phase_jump_threshold_ns);
	log_debug("\t- phase_resolution_ns: %d", config->phase_resolution_ns);
	log_debug("\t- reactivity_min: %d", config->reactivity_min);
	log_debug("\t- reactivity_max: %d", config->reactivity_max);
	log_debug("\t- reactivity_power: %d", config->reactivity_power);
	log_debug("\t- nb_calibration: %d", config->nb_calibration);
	log_debug("\t- fine_stop_tolerance: %d", config->fine_stop_tolerance);
	log_debug("\t- calibrate_first: %s", config->calibrate_first ? "true" : "false");
	log_debug("\t- oscillator_factory_settings: %s", config->oscillator_factory_settings ? "true" : "false");
	log_debug("\t- tracking_only: %s", config->tracking_only ? "true" : "false");
}

static inline void print_disciplining_parameters(struct disciplining_parameters *params)
{
	int i = 0;
	log_debug("Discipining Parameters:");
	log_debug("\t- ctrl_nodes_length: %d", params->ctrl_nodes_length);

	log_debug("\t- ctrl_load_nodes:");
	for (i = 0; i < params->ctrl_nodes_length; i++)
		log_debug("\t\t- [%d]: %f", i, params->ctrl_load_nodes[i]);

	log_debug("\t- ctrl_drift_coeffs]:");
	for (i = 0; i < params->ctrl_nodes_length; i++)
		log_debug("\t\t- [%d]: %f", i, params->ctrl_drift_coeffs[i]);

	log_debug("\t- coarse_equilibrium: %d", params->coarse_equilibrium);


	log_debug("\t- ctrl_nodes_length_factory: %d", params->ctrl_nodes_length_factory);

	log_debug("\t- ctrl_load_nodes_factory:");
	for (i = 0; i < params->ctrl_nodes_length_factory; i++)
		log_debug("\t\t- [%d]: %f", i, params->ctrl_load_nodes_factory[i]);
	log_debug("\t- ctrl_drift_coeffs_factory:");
	for (i = 0; i < params->ctrl_nodes_length_factory; i++)
		log_debug("\t\t- [%d]: %f", i, params->ctrl_drift_coeffs_factory[i]);
	log_debug("\t- coarse_equilibrium_factory: %d", params->coarse_equilibrium_factory);

	log_debug("\t- calibration_valid: %s", params->calibration_valid ? "true" : "false");
}


#endif /* PARAMETERS_H */
