#ifndef _APR2UTM_H
#define _APR2UTM_H


#include <gsl/gsl_errno.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_multiroots.h>


int quadratic_system(const gsl_vector *x, void *params, gsl_vector *f);
//std::array<double, 4> calculate(std::size_t tags_num, double x, double y, double distance);

#endif
