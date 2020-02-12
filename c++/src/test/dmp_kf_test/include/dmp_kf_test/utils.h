#ifndef DMP_KF_TEST_UTILS_H
#define DMP_KF_TEST_UTILS_H

#include <vector>
#include <memory>
#include <armadillo>
#include <dmp_lib/DMP/DMP_.h>
#include <dmp_lib/DMP/DMP_pos.h>
#include <dmp_lib/DMP/DMP_eo.h>
#include <dmp_lib/DMP/DMPo.h>
#include <dmp_lib/CanonicalClock/CanonicalClock.h>

using namespace as64_;

void simulatePosOrientDMP(std::shared_ptr<dmp_::DMP_pos> &dmp_p,
                          std::shared_ptr<dmp_::DMP_eo> &dmp_o,
                          const arma::vec &P0, const arma::vec &Q0,
                          const arma::vec &Pg, const arma::vec &Qg, double T, double dt,
                          arma::rowvec &Time, arma::mat &P_data, arma::mat &dP_data, arma::mat &ddP_data,
                          arma::mat &Q_data, arma::mat &vRot_data, arma::mat &dvRot_data);

void saveDMPdata(const std::string &dmp_data_file,
                 const std::shared_ptr<dmp_::DMP_pos> &dmp_p,
                 const std::shared_ptr<dmp_::DMP_eo> &dmp_eo,
                 const std::shared_ptr<dmp_::DMPo> &dmp_o,
                 const arma::vec &Yg0, const arma::vec &Y0,
                 const arma::vec &Qg0, const arma::vec &Q0, double tau0);

void loadDMPdata(const std::string &dmp_data_file,
                 std::shared_ptr<dmp_::DMP_pos> *dmp_p=0,
                 std::shared_ptr<dmp_::DMP_eo> *dmp_eo=0,
                 std::shared_ptr<dmp_::DMPo> *dmp_o=0,
                 arma::vec *Yg0=0, arma::vec *Y0=0,
                 arma::vec *Qg0=0, arma::vec *Q0=0, double *tau0=0);

#endif // DMP_KF_TEST_UTILS_H