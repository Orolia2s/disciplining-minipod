#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <stdio.h>

struct parameters {
	int ref_fluctuations_ns;
	int phase_jump_threshold_ns;
	int phase_resolution_ns;
	int ctrl_nodes_length;
	double *ctrl_load_nodes;
	double *ctrl_drift_coeffs;
};

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
};

static inline void print_parameters(struct parameters *params) {
	printf("ref_fluctuations_ns: %d\n", params->ref_fluctuations_ns);
	printf("phase_jump_threshold_ns: %d\n", params->phase_jump_threshold_ns);
	printf("phase_resolution_ns: %d\n", params->phase_resolution_ns);
	printf("ctrl_nodes_length: %d\n", params->ctrl_nodes_length);
	for (int i = 0; i < params->ctrl_nodes_length; i++) {
		printf("ctrl_load_nodes[%d]: is %f\n", i, params->ctrl_load_nodes[i]);
	}
	for (int i = 0; i < params->ctrl_nodes_length; i++) {
		printf("ctrl_drift_coeffs[%d]: is %f\n", i, params->ctrl_drift_coeffs[i]);
	}
}

int fill_parameters(struct parameters *p, const char *path, char err_msg[]);


#endif /* PARAMETERS_H */
