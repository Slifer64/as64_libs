#ifndef gmp_TEST_UTILS_H
#define gmp_TEST_UTILS_H

#include <vector>
#include <memory>
#include <armadillo>
#include <gmp_lib/gmp/gmp_.h>
#include <gmp_lib/gmp/gmp_pos.h>
#include <gmp_lib/gmp/gmpo.h>
#include <gmp_lib/gmp/gmp_eo.h>
#include <gmp_lib/CanonicalClock/CanonicalClock.h>
#include <gmp_lib/math/quaternions.h>

using namespace as64_;

void simulategmp(std::vector<std::shared_ptr<gmp_::gmp_>> &gmp, const arma::vec &y0,
                 const arma::vec &yg, double T, double dt,
                 arma::mat &Time, arma::mat &y_data, arma::mat &dy_data, arma::mat &ddy_data);


void simulategmp(std::shared_ptr<gmp_::gmp_> &gmp, double y0, double yg, double T, double dt,
                 arma::rowvec &Time, arma::rowvec &y_data, arma::rowvec &dy_data, arma::rowvec &ddy_data);

#endif // gmp_TEST_UTILS_H