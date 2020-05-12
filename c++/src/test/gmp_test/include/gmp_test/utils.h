#ifndef GMP_TEST_UTILS_H
#define GMP_TEST_UTILS_H

#include <vector>
#include <memory>
#include <armadillo>
#include <gmp_lib/gmp_lib.h>
#include <gmp_lib/math/quaternions.h>
#include <io_lib/io_utils.h>

using namespace as64_;

void simulateGMP(std::vector<std::shared_ptr<gmp_::GMP>> &gmp, const arma::vec &y0,
                 const arma::vec &yg, double T, double dt,
                 arma::mat &Time, arma::mat &y_data, arma::mat &dy_data, arma::mat &ddy_data);


void simulateGMP(std::shared_ptr<gmp_::GMP> gmp, double y0, double yg, double T, double dt,
                 arma::rowvec &Time, arma::rowvec &y_data, arma::rowvec &dy_data, arma::rowvec &ddy_data);

void simulateGMP_nDoF(std::shared_ptr<gmp_::GMP_nDoF> &gmp, const arma::vec &y0,
                      const arma::vec &yg, double T, double dt,
                      arma::mat &Time, arma::mat &Y_data, arma::mat &dY_data, arma::mat &ddY_data);


void simulateGMPo_in_log_space(std::shared_ptr<gmp_::GMPo> &gmp_o, const arma::vec &Q0, const arma::vec &Qg,
                               double T, double dt, arma::rowvec &Time, arma::mat &Q_data, arma::mat &rotVel_data, arma::mat &rotAccel_data);


void simulateGMPo_in_quat_space(std::shared_ptr<gmp_::GMPo> &gmp_o, const arma::vec &Q0, const arma::vec &Qg,
                              double T, double dt, arma::rowvec &Time, arma::mat &Q_data, arma::mat &rotVel_data, arma::mat &rotAccel_data);

#endif // GMP_TEST_UTILS_H
