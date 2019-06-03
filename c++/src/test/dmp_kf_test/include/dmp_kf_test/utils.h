#ifndef DMP_KF_TEST_UTILS_H
#define DMP_KF_TEST_UTILS_H

#include <vector>
#include <memory>
#include <armadillo>
#include <dmp_lib/DMP/DMP_.h>
#include <dmp_lib/DMP/DMP_pos.h>
#include <dmp_lib/DMP/DMP_eo.h>
#include <dmp_lib/CanonicalClock/CanonicalClock.h>

using namespace as64_;

void simulatePosOrientDMP(std::shared_ptr<dmp_::DMP_pos> &dmp_p,
                          std::shared_ptr<dmp_::DMP_eo> &dmp_o,
                          const arma::vec &P0, const arma::vec &Q0,
                          const arma::vec &Pg, const arma::vec &Qg, double T, double dt,
                          arma::rowvec &Time, arma::mat &P_data, arma::mat &dP_data, arma::mat &ddP_data,
                          arma::mat &Q_data, arma::mat &vRot_data, arma::mat &dvRot_data);

#endif // DMP_KF_TEST_UTILS_H