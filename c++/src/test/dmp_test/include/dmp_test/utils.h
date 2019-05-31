#ifndef DMP_TEST_UTILS_H
#define DMP_TEST_UTILS_H

#include <vector>
#include <memory>
#include <armadillo>
#include <dmp_lib/DMP/DMP_.h>
#include <dmp_lib/CanonicalClock/CanonicalClock.h>

using namespace as64_;

void simulateDMP(std::vector<std::shared_ptr<dmp_::DMP_>> &dmp, const arma::vec &y0,
                 const arma::vec &yg, double T, double dt,
                 arma::mat &Time, arma::mat &y_data, arma::mat &dy_data, arma::mat &ddy_data);


void simulateDMP(std::shared_ptr<dmp_::DMP_> &dmp, double y0, double yg, double T, double dt,
                 arma::rowvec &Time, arma::rowvec &y_data, arma::rowvec &dy_data, arma::rowvec &ddy_data);

#endif // DMP_TEST_UTILS_H