#ifndef UTILS_H
#define UTILS_H

/* parameters of a linear function such as y = a*x + b */
struct linear_func_param {
	double a;
	double b;
};

int simple_linear_reg(double x[], double y[], int length, struct linear_func_param * func_params);

#endif /* UTILS_H */
