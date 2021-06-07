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

#include "utils.h"
#include "log.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>

static double sum(double values[], int length) {
	double sum = 0.0;

	if (length <= 0) {
		return 0.0;
	}

	for(int i = 0; i < length; i ++) {
		sum += values[i];
	}

	return sum;
}

/*
 * Return the index where to insert item x in list a, assuming a is sorted.
 * The return value i is such that all e in a[:i] have e <= x, and all e in
 * a[i:] have e > x.
 */
static int bisect_right(double values[], int length, double x) {
	int low = 0;
	int step = 0;
	int hi = length;
	if (hi < 0) {
		log_error("bisect_right: length cannot be negative!");
		return -EINVAL;
	}

	while(low < hi) {
		step = (int) (low + hi) / 2;
		if (x < values[step])
			hi = step;
		else
			low = step + 1;
	}
	return low;
}

int simple_linear_reg(double x[], double y[], int length, struct linear_func_param * func_params) {
	double mean_x;
	double mean_y;
	
	if (length <= 0) {
		log_error("simple_linear_red: length cannot be negative");
		return -EINVAL;
	}

	mean_x = sum(x, length) / length;
	mean_y = sum(y, length) / length;

	double ss_x[length];
	double ss_y[length];
	for (int i = 0; i < length; i++) {
		ss_x[i] = x[i] * (y[i] - mean_y);
		ss_y[i] = x[i] * (x[i] - mean_x);

		if (ss_x[i] == HUGE_VAL || ss_y[i] == HUGE_VAL) {
			log_error("HUGE_VAL detected ! ss_x[%d] is %f and ss_y[%d] is %f", i, ss_x[i], i, ss_y[i]);
			return -ERANGE;
		}
	}
	double sum_ss_y = sum(ss_y, length);
	if (sum_ss_y == 0.0) {
		log_error("sum_ss_y is equal to 0");
		return -EINVAL;
	}
	func_params->a = sum(ss_x, length) / sum_ss_y;
	func_params->b = mean_y - func_params->a * mean_x;

	return 0;
}

/* piece-wise linear interpolation/extrapolation for given x and y array in ascending order
 * x_interp: allows to choose between x interpolation or y one.
 */
int lin_interp(double x[], double y[], int length, bool x_interp, double interp_value, double *interp_result) {
	double slopes[length - 1];
	int index;
	if (length < 0) {
		log_error("lin_interp: length cannot be negative (value is %d)", length);
		return -EINVAL;
	}

	for (int i = 0; i < length -1; i++) {
		if (x[i+1] - x[i] == 0.0) {
			log_error("difference between x values at %d and %d is null", i, i+1);
			return -EINVAL;
		}
		else
			slopes[i] = (y[i+1] - y[i]) / (x[i+1] - x[i]);
	}

	if (x_interp) {
		/* X interpolation */
		if (interp_value >= x[length - 1])
			index = length - 2;
		else {
			index = bisect_right(x, length, interp_value) - 1;
			if (index <= 0)
				index = 0;
		}
		
		*interp_result = y[index] + slopes[index] * (interp_value - x[index]);
	} else {
		/* Y interpolation */
		if (interp_value >= y[length - 1])
			index = length - 2;
		else {
			index = bisect_right(y, length, interp_value) - 1;
			if (index <= 0)
				index = 0;
		}

		if (slopes[index] == 0.0)
			*interp_result = x[index];
		else
			*interp_result = x[index] + (interp_value - y[index]) / slopes[index];
	}

	return 0;
}
