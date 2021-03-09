#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <stdio.h>

struct parameters {
	int ref_fluctuations_ns;
	int phase_jump_threshold_ns;
	int phase_resolution_ns;
	double ctrl_load_node_0;
	double ctrl_load_node_1;
	double ctrl_load_node_2;
	double ctrl_drift_coeffs_0;
	double ctrl_drift_coeffs_1;
	double ctrl_drift_coeffs_2;
};

enum value_type {
	VALUE_TYPE_DOUBLE,
	VALUE_TYPE_INT,
	VALUE_TYPE_BOOL,
	VALUE_TYPE_CHAR,
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
	CONFIG_ENTRY(ctrl_load_node_0, DOUBLE),
	CONFIG_ENTRY(ctrl_load_node_1, DOUBLE),
	CONFIG_ENTRY(ctrl_load_node_2, DOUBLE),
	CONFIG_ENTRY(ctrl_drift_coeffs_0, DOUBLE),
	CONFIG_ENTRY(ctrl_drift_coeffs_1, DOUBLE),
	CONFIG_ENTRY(ctrl_drift_coeffs_2, DOUBLE),
};

static inline void print_parameters(struct parameters *params) {
	printf("ref_fluctuations_ns: %d\n", params->ref_fluctuations_ns);
	printf("phase_jump_threshold_ns: %d\n", params->phase_jump_threshold_ns);
	printf("phase_resolution_ns: %d\n", params->phase_resolution_ns);
	printf("ctrl_load_node_0: %f\n", params->ctrl_load_node_0);
	printf("ctrl_load_node_1: %f\n", params->ctrl_load_node_1);
	printf("ctrl_load_node_2: %f\n", params->ctrl_load_node_2);
	printf("ctrl_drift_coeffs_0: %f\n", params->ctrl_drift_coeffs_0);
	printf("ctrl_drift_coeffs_1: %f\n", params->ctrl_drift_coeffs_1);
	printf("ctrl_drift_coeffs_2: %f\n", params->ctrl_drift_coeffs_2);
}

int fill_parameters(struct parameters *p, const char *path, char err_msg[]);


#endif /* PARAMETERS_H */
