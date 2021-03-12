#ifndef ALGORITHM_STRUCTS_H
#define ALGORITHM_STRUCTS_H

#include <stdint.h>

struct parameters {
	int ref_fluctuations_ns;
	int phase_jump_threshold_ns;
	int phase_resolution_ns;
	int ctrl_nodes_length;
	double *ctrl_load_nodes;
	double *ctrl_drift_coeffs;
};

struct kalman_parameters {
	double Ksigma;
	double Kphase;
	float q;
	float r;
	bool Kphase_set;
};

enum State {
	INIT,
	PHASE_ADJUSTMENT,
	HOLDOVER,
};

struct algorithm_state {
	enum State status;
	double mRO_fine_step_sensitivity;
	double mRO_coarse_step_sensitivity;
	bool coarse_ctrl;
	bool invalid_ctrl;
	int settling_time;
	bool calib;
	uint32_t ctrl_range_coarse[2];
	uint16_t ctrl_range_fine[2];
	uint16_t *ctrl_points;
	uint16_t fine_mid;
	uint32_t coarse_ctrl_value;
	uint16_t fine_ctrl_value;
	uint32_t estimated_equilibrium;
	double estimated_drift;
	struct kalman_parameters kalman;
};

#endif /* ALGORITHM_STRUCTS_H */
