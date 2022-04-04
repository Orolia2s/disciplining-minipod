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
 * @file algorithm_structs.h
 * @brief Header containing all algorithm's structures.
 */
#ifndef ALGORITHM_STRUCTS_H
#define ALGORITHM_STRUCTS_H

#include <stdint.h>
#include <oscillator-disciplining/oscillator-disciplining.h>

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
 * @struct algorithm_state
 * @brief Algorithm data stored in od context
 */
struct algorithm_state {
	struct od_input inputs[7];
	/** State value */
	enum Disciplining_State status;
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
	/** Number of ctrl_points */
	int ctrl_points_length;
	/** Fine values for which a drift coefficient is computed */
	float *ctrl_points;
	/** Ctrl drift coeffs */
	float *ctrl_drift_coeffs;
	/** Median value of the fine range */
	uint16_t fine_mid;
	/** Fine control value computed by the algorithm */
	uint16_t fine_ctrl_value;
	/** Estimated equilibrium value of the fine control */
	uint16_t estimated_equilibrium;
	/** Exponential Smooth of the estimated equilibrium */
	uint16_t estimated_equilibrium_ES;
	/** Estimated drift based on last fine control value and drift coefficients */
	float estimated_drift;
	/** Kalman filter paramters */
	struct kalman_parameters kalman;
	/** Counts how many times od_process has been called */
	int od_process_count;
	/** Counter of number of cycles where phase error is below reference during tracking phase */
	uint16_t tracking_phase_convergence_count;
	/** Basic count threshold tracking_phase_convergence count */
	uint16_t tracking_phase_convergence_count_threshold;
};

#endif /* ALGORITHM_STRUCTS_H */
