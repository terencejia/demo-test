#include <gsl/gsl_multifit.h>
#include <gsl/gsl_errno.h>
#include "fs_log.hpp"
#include "math_ext.hpp"

namespace mathext {

void multifit_robust(const std::vector<point_t> & points, std::size_t degree, std::vector<double> & r_coefficient)
{
	gsl_set_error_handler_off();

	const std::size_t n = points.size();
	const std::size_t p = degree + 1;

	gsl_matrix * X = gsl_matrix_alloc(n, p);
	gsl_vector * y = gsl_vector_alloc(n);
	gsl_vector * c = gsl_vector_alloc(p);
	gsl_matrix * cov = gsl_matrix_alloc(p, p);

	/* generate linear dataset */
	for (std::size_t i = 0; i < n; i++)
	{
		gsl_vector_set(y, i, points[i].y);
	}

	/* construct design matrix X for linear fit */
	for (std::size_t i = 0; i < n; ++i) {
		double xij = 1.0;
		for (std::size_t j = 0; j < p; ++j) {
			gsl_matrix_set(X, i, j, xij);
			xij *= points[i].x;
		}
	}

	/* perform robust fit */
	gsl_multifit_robust_workspace * work
		= gsl_multifit_robust_alloc(gsl_multifit_robust_bisquare, X->size1, X->size2);
	int err = gsl_multifit_robust(X, y, c, cov, work);
	if (err) {
		//const char * gslerrstr = gsl_strerror(err);

	};

	gsl_multifit_robust_free(work);

	r_coefficient.resize(c->size);
	for (std::size_t i = 0; i < c->size; ++i) {
		r_coefficient[i] = gsl_vector_get(c, i);
	}
#if 0
	log_debug("best fit: y = %f + %fx + %fx^2 + %fx^3 ", gsl_vector_get(c, 0),
			degree < 1 ? 0 : gsl_vector_get(c, 1),
			degree < 2 ? 0 : gsl_vector_get(c, 2),
			degree < 3 ? 0 : gsl_vector_get(c, 3));
#endif
	///\todo draw a pic by plotutil

	gsl_matrix_free (cov);
	gsl_vector_free (c);
	gsl_vector_free (y);
	gsl_matrix_free (X);

	/**
	* \note here we trust the gsl error will result in core dump by default,
	* so if we disabled that, we should check error code by ourselves.
	* please see gsl function:
	* const char * gsl_strerror (const int gsl_errno)
	*/
}

} /* namespace mathext */
