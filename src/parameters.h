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
	CONFIG_ENTRY(debug, BOOL),
	CONFIG_ENTRY(reactivity_min, INT),
	CONFIG_ENTRY(reactivity_max, INT),
	CONFIG_ENTRY(reactivity_power, INT),
	CONFIG_ENTRY(nb_calibration, INT),
	CONFIG_ENTRY(fine_stop_tolerance, INT),
	CONFIG_ENTRY(settling_time, INT),
	CONFIG_ENTRY(max_allowed_coarse, INT),
	CONFIG_ENTRY(calibrate_first, BOOL),
};

static inline void print_parameters(struct parameters *params) {
	debug("Config file path is %s\n", params->path);
	debug("debug %s\n", params->debug ? "true" : "false");
	debug("ref_fluctuations_ns: %d\n", params->ref_fluctuations_ns);
	debug("phase_jump_threshold_ns: %d\n", params->phase_jump_threshold_ns);
	debug("phase_resolution_ns: %d\n", params->phase_resolution_ns);
	debug("ctrl_nodes_length: %d\n", params->ctrl_nodes_length);
	for (int i = 0; i < params->ctrl_nodes_length; i++) {
		debug("ctrl_load_nodes[%d]: is %f\n", i, params->ctrl_load_nodes[i]);
	}
	for (int i = 0; i < params->ctrl_nodes_length; i++) {
		debug("ctrl_drift_coeffs[%d]: is %f\n", i, params->ctrl_drift_coeffs[i]);
	}
	debug("reactivity_min: %d \n", params->reactivity_min);
	debug("reactivity_max: %d \n", params->reactivity_max);
	debug("reactivity_power: %d \n", params->reactivity_power);
	debug("nb_calibration: %d \n", params->nb_calibration);
	debug("fine_stop_tolerance: %d \n", params->fine_stop_tolerance);
	debug("settling_time %d\n", params->settling_time);
	debug("Calibrate_first %s\n", params->calibrate_first ? "true" : "false");
}

int fill_parameters(struct parameters *p, const char *path, char err_msg[]);


#endif /* PARAMETERS_H */
