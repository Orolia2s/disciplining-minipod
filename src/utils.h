#ifndef UTILS_H
#define UTILS_H
/**
 * @file utils.h
 * @brief Provide mathematical functions to compute linear regression and linear interpolation.
 *
 */

#include <stdbool.h>

#define X_INTERPOLATION true
#define Y_INTERPOLATION false
/* parameters of a linear function such as y = a*x + b */
struct linear_func_param {
	double a;
	double b;
};

int simple_linear_reg(double x[], double y[], int length, struct linear_func_param * func_params);
int lin_interp(double x[], double y[], int length, bool x_interp, double interp_value, double *interp_result);

#endif /* UTILS_H */
