/**
 * @file algorithm_structs.h
 * @brief Header containing all algorithm's structures.
 */
#ifndef ALGORITHM_STRUCTS_H
#define ALGORITHM_STRUCTS_H

#include <stdint.h>
/**
 * @struct parameters
 * @brief Structure containing all variables fetched from
 * configuration file.
 */
struct parameters {
	/** Config file path */
	char * path;
	/** Used to filter phase */
	int ref_fluctuations_ns;
	/** Threshold above which as phase jump is requested */
	int phase_jump_threshold_ns;
	/** Phasemeter's resolution in ns */
	int phase_resolution_ns;
	/** Number of control nodes in ctrl_load_nodes */
	int ctrl_nodes_length;
	/** Equilibrium Coarse value define during calibration */
	uint32_t coarse_equilibrium;
	/**
	 * Array containing the control node, in percentage
	 * value of the control range.
	 * Array must be of size ctrl_node_length.
	 */
	double *ctrl_load_nodes;
	/**
	 * Array of drift coefficients for each control node.
	 * Array must be of size ctrl_node_length.
	 */
	double *ctrl_drift_coeffs;
	/** Enable debug logs */
	bool debug;
	/** Minimal reactivity */
	int reactivity_min;
	/** Maximal reactivity */
	int reactivity_max;
	/** Power used in reactivity computation */
	int reactivity_power;
	/** number of phase error measures for one control node
	 * when doing a calibration
	 */
	int nb_calibration;
	/**
	 * Set tolerance range to check if fine equilibrium is inside this range
	 * to validate calibration.
	 * After a calibration fine equilibrium point must be between
	 * ctrl_range_fine[0] + fine_stop_tolerance and
	 * ctrl_range_fine[1] - fine_stop_tolerance.
	 */
	int fine_stop_tolerance;
	/** Number of seconds to wait after setting fine or coarse value to oscillator */
	int settling_time;
	/** Maxium difference allowed when changin coarse value */
	int max_allowed_coarse;
	/** Triggers calibration when starting the program */
	bool calibrate_first;
};

/**
 * @struct kalman_parameters
 * @brief Kalman filter parameters
 */
struct kalman_parameters {
	double Ksigma;
	double Kphase;
	float q;
	float r;
	bool Kphase_set;
};

/**
 * @enum State
 * @brief Algorithm state value
 */
enum State {
	/** Initialization State */
	INIT,
	/** Phase adjustement State, nominal one */
	PHASE_ADJUSTMENT,
	/** Holdover state, when gnss data is not valid */
	HOLDOVER,
	/** Calibration state, when drift coefficients are computed */
	CALIBRATION,
};

/**
 * @struct algorithm_state
 * @brief Algorithm data stored in od context
 */
struct algorithm_state {
	/** State value */
	enum State status;
	/** Frequency adjustement for one value on the fine control */
	double mRO_fine_step_sensitivity;
	/** Frequency adjustement for one value on the coarse control */
	double mRO_coarse_step_sensitivity;
	/** Indicate ctrl values (either fine or coarse) is invalid and
	 * that a mro check must be done
	 */
	bool invalid_ctrl;
	/** Indicate a calibration has been requested either by software or
	 * user and is on going
	 */
	bool calib;
	/** Range of possible coarse values applicable to the oscillator */
	uint32_t ctrl_range_coarse[2];
	/** Range of possible fine values applicable to the oscillator */
	uint16_t ctrl_range_fine[2];
	/** Fine values for which a drift coefficient is computed */
	uint16_t *ctrl_points;
	/** Median value of the fine range */
	uint16_t fine_mid;
	/** Fine control value computed by the algorithm */
	uint16_t fine_ctrl_value;
	/** Estimated equilibrium value of the fine control */
	uint32_t estimated_equilibrium;
	/** Estimated drift based on last fine control value and drift coefficients */
	double estimated_drift;
	/** Kalman filter paramters */
	struct kalman_parameters kalman;
};

#endif /* ALGORITHM_STRUCTS_H */
