#include "utils.h"

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

int simple_linear_reg(double x[], double y[], int length, struct linear_func_param * func_params) {
	double mean_x;
	double mean_y;
	
	if (length <= 0) {
		return -EINVAL;
	}

	mean_x = sum(x, length) / length;
	mean_y = sum(y, length) / length;

	double ss_x[length];
	double ss_y[length];
	for (int i = 0; i < length; i++) {
		ss_x[i] = x[i] * (y[i] - mean_y);
		ss_y[i] = x[i] * (x[i] - mean_x);

		if (ss_x[i] == HUGE_VAL || ss_y[i] == HUGE_VAL)
			return -ERANGE;
	}
	double sum_ss_y = sum(ss_y, length);
	if (sum_ss_y == 0.0) {
		return -EINVAL;
	}
	func_params->a = sum(ss_x, length) / sum_ss_y;
	func_params->b = mean_y - func_params->a * mean_x;

	return 0;
}
