#include <dmp_lib/KalmanFilter/DMPpEKFa.h>
#include <dmp_lib/math/quaternions.h>

namespace as64_
{

namespace dmp_
{

DMPpEKFa::DMPpEKFa(std::shared_ptr<dmp_::DMP_pos> dmp, double Ts)
{
  this->Ts = Ts;
  this->dmp = dmp;
  this->Az = arma::diagmat( arma::vec( {dmp->dmp[0]->a_z, dmp->dmp[1]->a_z, dmp->dmp[2]->a_z} ) );
  this->Bz = arma::diagmat( arma::vec( {dmp->dmp[0]->b_z, dmp->dmp[1]->b_z, dmp->dmp[2]->b_z} ) );

  int N_params = 10;
  int N_msr = 6;

  this->theta = arma::vec().zeros(N_params);
  this->P = arma::mat().zeros(N_params,N_params);

  this->setProcessNoiseCov(arma::mat().eye(N_params,N_params)*1e-5);
  this->setMeasureNoiseCov(arma::mat().eye(N_msr,N_msr)*0.05);

  this->setFadingMemoryCoeff(1.0);

  this->enableParamsContraints(false);
  this->setParamsConstraints(arma::mat().zeros(1,N_params),arma::vec().zeros(1));

  this->setPartDerivStep(0.001);

  H_k = arma::join_horiz( arma::mat().eye(6,6), arma::mat().zeros(6,4) );

  // this->stateTransFun_ptr = std::bind(&DMPpEKFa::stateTransFun, *this, std::placeholders::_1, std::placeholders::_2);
  // this->msrFun_ptr = std::bind(&DMPpEKFa::msrFun, *this, std::placeholders::_1, std::placeholders::_2);
  // this->stateTransFunJacob_ptr = std::bind(&DMPpEKFa::stateTransFunJacob, *this, std::placeholders::_1, std::placeholders::_2);
  // this->msrFunJacob_ptr = std::bind(&DMPpEKFa::msrFunJacob, *this, std::placeholders::_1, std::placeholders::_2);
}

void DMPpEKFa::setFadingMemoryCoeff(double a_p)
{
  this->a_p = a_p;
}

void DMPpEKFa::enableParamsContraints(bool enable_contraints)
{
  this->enable_constraints = enable_contraints;
}

void DMPpEKFa::setParamsConstraints(const arma::mat &A_c, const arma::vec &b_c)
{
  this->A_c = A_c;
  this->b_c = b_c;
}

void DMPpEKFa::setProcessNoiseCov(const arma::mat &Qn)
{
  this->Qn = Qn;
}

void DMPpEKFa::setMeasureNoiseCov(const arma::mat &Rn)
{
  this->Rn = Rn;
}

void DMPpEKFa::setPartDerivStep(double dtheta)
{
  this->dtheta = arma::vec().ones(this->theta.size())*dtheta;
}

void DMPpEKFa::setPartDerivStep(const arma::vec &dtheta)
{
  this->dtheta = dtheta;
}

void DMPpEKFa::predict(void *cookie)
{
  F_k = stateTransFunJacob(theta, cookie);
  theta = stateTransFun(theta, cookie);

  arma::mat FP = P;
  FP.rows(0,2) = F_k.rows(0,2)*P;
  FP.rows(3,5) = FP.rows(3,5) + P.rows(0,2)*Ts;
  P = std::pow(a_p,2)*FP*F_k.t() + Qn;

}

void DMPpEKFa::correct(const arma::vec &z, void *cookie)
{
  // =====  Retrive the measurement function Jacobian  =====
  // this->H_k = msrFunJacob(theta, cookie);

  // =====  Correction estimates =====
  arma::vec z_hat = msrFun(this->theta, cookie);
//  arma::mat K_kf = arma::solve((P.submat(0,0,5,5) + Rn).t(), H_k*P.t(), arma::solve_opts::fast).t();

//  this->theta = this->theta + K_kf * (z - z_hat);

//  // =====  Apply projection if enabled  =====
//  arma::mat D; // active contraints
//  arma::vec d;
//  bool proj_flag = false;
//  if ( this->enable_constraints & ~b_c.is_empty() )
//  {
//    arma::uvec ind = arma::find(A_c*theta > b_c);
//    if (~ind.is_empty())
//    {
//      proj_flag = true;
//      D = A_c.rows(ind);
//      d = b_c.elem(ind);
//    }
//  }
//
//  int N_params = theta.size();
//  arma::mat I = arma::mat().eye(N_params, N_params);
//
//  if (proj_flag)
//  {
//    K_kf = ( I - D.t()*arma::inv_sympd(D*D.t())*D ) * K_kf;
//    this->theta = theta - D.t()*arma::inv_sympd(D*D.t())*(D*theta-d);
//  }

  // =====  Calculate new covariance  =====
  // this->P = (I - K_kf*H_k) * P * (I - K_kf*H_k).t() + K_kf*Rn*K_kf.t();

  // K = arma::solve((P.submat(0,0,5,5) + Rn).t(), H_k*P.t(), arma::solve_opts::fast).t();
   K = arma::solve((P.submat(0,0,5,5) + Rn), P.rows(0,5), arma::solve_opts::fast+arma::solve_opts::likely_sympd).t();

//  Eigen::Map<Eigen::Matrix<double,10,6>> K_map(K.memptr());
//  Eigen::Map<Eigen::Matrix<double,10,10>> P_map(P.memptr());
//  Eigen::Map<Eigen::Matrix<double,6,6>> R_map(Rn.memptr());
//  Eigen::Matrix<double,6,6> cPc_R = P_map.block(0,0,6,6) + R_map;
//  Eigen::LLT<Eigen::Matrix<double,6,6>> llt(cPc_R);
//  K_map = (llt.solve(P_map.block(0,0,6,10))).transpose();

  theta += K*(z - z_hat);
  P += -K*P.rows(0,5);

  //this->K = K_kf;
}

arma::vec DMPpEKFa::msrFun(const arma::vec &theta, void *cookie)
{
  return theta.subvec(0,5);
}

arma::vec DMPpEKFa::stateTransFun(const arma::vec &theta, void *cookie)
{
  arma::vec p_dot = theta.subvec(0,2);
  arma::vec p = theta.subvec(3,5);
  arma::vec pg = theta.subvec(6,8);
  double tau = theta(9);

  double x = static_cast<StateTransCookie *>(cookie)->t/tau;
  double tau0 = dmp->getTau();
  dmp->setTau(tau);
  arma::vec p_ddot = dmp->calcYddot(x, p, p_dot, pg);
  dmp->setTau(tau0);

  arma::vec theta_next(10);
  theta_next.subvec(0,2) = p_dot + p_ddot*Ts;
  theta_next.subvec(3,5) = p + p_dot*Ts;
  theta_next.subvec(6,9) = theta.subvec(6,9);

  return theta_next;
}

arma::mat DMPpEKFa::msrFunJacob(const arma::vec &theta, void *cookie)
{
  return arma::join_horiz( arma::mat().eye(6,6), arma::mat().zeros(6,4) );
}

arma::mat DMPpEKFa::stateTransFunJacob(const arma::vec &theta, void *cookie)
{
  double t = static_cast<StateTransCookie *>(cookie)->t;
  arma::vec p0 = static_cast<StateTransCookie *>(cookie)->p0;

  arma::vec p_dot = theta.subvec(0,2);
  arma::vec p = theta.subvec(3,5);
  arma::vec pg = theta.subvec(6,8);
  double tau = theta(9);
  double x = t/tau;

  arma::mat I3 = arma::mat().eye(3,3);
  arma::mat F = arma::mat().eye(10,10);
  F.submat(0,0,2,2) = I3 - Az*Ts/tau;
  F.submat(0,3,2,5) = -Az*Bz*Ts/std::pow(tau,2);
  F.submat(0,6,2,9) = dmp->getAcellPartDev_g_tau(t, p, p_dot, p0, x, pg, tau)*Ts;
  F.submat(3,0,5,2) = I3*Ts;

  return F;
}


} // namespace dmp_

} // namespace as64_
