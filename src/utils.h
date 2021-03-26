#ifndef UTILS_H
#define UTILS_H
/**
 * @file utils.h
 * @brief Provide mathematical functions to compute linear regression and linear interpolation.
 *
 */

#include <stdbool.h>

/** Define to use for interp_value of lin_interp function */
#define X_INTERPOLATION true
/** Define to use for interp_value of lin_interp function */
#define Y_INTERPOLATION false

/**
 * @struct linear_func_param
 * @brief Description of a linear function such as y = a*x + b
 */
struct linear_func_param {
	double a;
	double b;
};

/**
 * @brief Simple linear regression
 * @param x Array of values corresponding to the abscissa
 * @param y Array of values corresponding to ordinate
 * @param length Length of both arrays
 * @param func_params Output structure of the linear regression
 * @return 0 in case of success, else errno
 */
int simple_linear_reg(double x[], double y[], int length, struct linear_func_param * func_params);
/**
 * @brief Linear interpolation
 * @param x Array of values corresponding to the abscissa
 * @param y Array of values corresponding to ordinate
 * @param length Length of both arrays
 * @param x_interp If true, do X interpolation, else a Y interpolation
 * @param interp_value Depending on x_interp, Y value for which to find the X value, or vice versa
 * @param interp_result output value of the linear interpolation
 * @return 0 in case of success, else errno
 */

int lin_interp(double x[], double y[], int length, bool x_interp, double interp_value, double *interp_result);

#endif /* UTILS_H */
