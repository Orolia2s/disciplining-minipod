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
	uint32_t estimated_equilibrium;
	/** Exponential Smooth of the estimated equilibrium */
	uint32_t estimated_equilibrium_ES;
	/** Estimated drift based on last fine control value and drift coefficients */
	float estimated_drift;
	/** Kalman filter paramters */
	struct kalman_parameters kalman;
};

#endif /* ALGORITHM_STRUCTS_H */
