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
 * @brief Parse parameters from the config file.
 *
 * Provides one function to parse all parameters define in algorithm parameters
 * Uses internal functions to parse different types of parameters (double, int,
 * boolean, char, double array)
 *
 */

#include <stdio.h>

#include "algorithm_structs.h"
#include "log.h"

enum value_type {
	VALUE_TYPE_DOUBLE,
	VALUE_TYPE_INT,
	VALUE_TYPE_BOOL,
	VALUE_TYPE_CHAR,
	VALUE_TYPE_DOUBLE_ARRAY,
};

#define CONFIG_ENTRY(key, t) { \
.name = #key, \
.offset = offsetof(struct parameters, key), \
.type = VALUE_TYPE_##t, \
}

struct config_key {
	const char *name;
	size_t offset;
	enum value_type type;
};


static const struct config_key config_keys[] = {
	/* base param structure fields */
	CONFIG_ENTRY(ref_fluctuations_ns, INT),
	CONFIG_ENTRY(phase_jump_threshold_ns, INT),
	CONFIG_ENTRY(phase_resolution_ns, INT),
	CONFIG_ENTRY(ctrl_nodes_length, INT),
	CONFIG_ENTRY(ctrl_load_nodes, DOUBLE_ARRAY),
	CONFIG_ENTRY(ctrl_drift_coeffs, DOUBLE_ARRAY),
	CONFIG_ENTRY(debug, INT),
	CONFIG_ENTRY(reactivity_min, INT),
	CONFIG_ENTRY(reactivity_max, INT),
	CONFIG_ENTRY(reactivity_power, INT),
	CONFIG_ENTRY(nb_calibration, INT),
	CONFIG_ENTRY(fine_stop_tolerance, INT),
	CONFIG_ENTRY(max_allowed_coarse, INT),
	CONFIG_ENTRY(calibrate_first, BOOL),
	CONFIG_ENTRY(coarse_equilibrium, INT),
};

static inline void print_parameters(struct parameters *params) {
	log_debug("Configuration variables set:");
	log_debug("\tConfig file path is %s", params->path);
	log_debug("\tdebug %s", params->debug ? "true" : "false");
	log_debug("\tref_fluctuations_ns: %d", params->ref_fluctuations_ns);
	log_debug("\tphase_jump_threshold_ns: %d", params->phase_jump_threshold_ns);
	log_debug("\tphase_resolution_ns: %d", params->phase_resolution_ns);
	log_debug("\tctrl_nodes_length: %d", params->ctrl_nodes_length);
	for (int i = 0; i < params->ctrl_nodes_length; i++) {
		log_debug("\tctrl_load_nodes[%d]: is %f", i, params->ctrl_load_nodes[i]);
	}
	for (int i = 0; i < params->ctrl_nodes_length; i++) {
		log_debug("\tctrl_drift_coeffs[%d]: is %f", i, params->ctrl_drift_coeffs[i]);
	}
	log_debug("\treactivity_min: %d", params->reactivity_min);
	log_debug("\treactivity_max: %d", params->reactivity_max);
	log_debug("\treactivity_power: %d", params->reactivity_power);
	log_debug("\tnb_calibration: %d", params->nb_calibration);
	log_debug("\tfine_stop_tolerance: %d", params->fine_stop_tolerance);
	log_debug("\tCalibrate_first %s", params->calibrate_first ? "true" : "false");
	log_debug("\tCoarse equilibrium: %d", params->coarse_equilibrium);
}

int fill_parameters(struct parameters *p, const char *path, char err_msg[]);


#endif /* PARAMETERS_H */
