#include <dmp_lib/KalmanFilter/DMPoEKFa.h>
#include <dmp_lib/math/quaternions.h>


namespace as64_
{

namespace dmp_
{

DMPoEKFa::DMPoEKFa(std::shared_ptr<dmp_::DMP_eo> dmp, double Ts)
{
  this->Ts = Ts;
  this->dmp = dmp;
  this->Az = arma::vec( {dmp->dmp[0]->a_z, dmp->dmp[1]->a_z, dmp->dmp[2]->a_z} );
  this->Bz = arma::vec( {dmp->dmp[0]->b_z, dmp->dmp[1]->b_z, dmp->dmp[2]->b_z} );

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

  // this->stateTransFun_ptr = std::bind(&DMPoEKFa::stateTransFun, *this, std::placeholders::_1, std::placeholders::_2);
  // this->msrFun_ptr = std::bind(&DMPoEKFa::msrFun, *this, std::placeholders::_1, std::placeholders::_2);
  // this->stateTransFunJacob_ptr = std::bind(&DMPoEKFa::stateTransFunJacob, *this, std::placeholders::_1, std::placeholders::_2);
  // this->msrFunJacob_ptr = std::bind(&DMPoEKFa::msrFunJacob, *this, std::placeholders::_1, std::placeholders::_2);
}

void DMPoEKFa::setFadingMemoryCoeff(double a_p)
{
  this->a_p = a_p;
}

void DMPoEKFa::enableParamsContraints(bool enable_contraints)
{
  this->enable_constraints = enable_contraints;
}

void DMPoEKFa::setParamsConstraints(const arma::mat &A_c, const arma::vec &b_c)
{
  this->A_c = A_c;
  this->b_c = b_c;
}

void DMPoEKFa::setProcessNoiseCov(const arma::mat &Qn)
{
  this->Qn = Qn;
}

void DMPoEKFa::setMeasureNoiseCov(const arma::mat &Rn)
{
  this->Rn = Rn;
}

void DMPoEKFa::setPartDerivStep(double dtheta)
{
  this->dtheta = arma::vec().ones(this->theta.size())*dtheta;
}

void DMPoEKFa::setPartDerivStep(const arma::vec &dtheta)
{
  this->dtheta = dtheta;
}

void DMPoEKFa::predict(void *cookie)
{
  F_k = stateTransFunJacob(theta, cookie);
  theta = stateTransFun(theta, cookie);
  P = std::pow(a_p,2)*F_k*P*F_k.t() + Qn;
}

void DMPoEKFa::correct(const arma::vec &z, void *cookie)
{
  // =====  Retrive the measurement function Jacobian  =====
  this->H_k = msrFunJacob(theta, cookie);

  // =====  Correction estimates =====
  arma::vec z_hat = msrFun(this->theta, cookie);
  arma::mat K_kf = arma::solve((H_k*P*H_k.t() + Rn).t(), H_k*P.t(), arma::solve_opts::fast).t();

  this->theta = this->theta + K_kf * (z - z_hat);

  // =====  Apply projection if enabled  =====
  arma::mat D; // active contraints
  arma::vec d;
  bool proj_flag = false;
  if ( this->enable_constraints & ~b_c.is_empty() )
  {
    arma::uvec ind = arma::find(A_c*theta > b_c);
    if (~ind.is_empty())
    {
      proj_flag = true;
      D = A_c.rows(ind);
      d = b_c.elem(ind);
    }
  }

  int N_params = theta.size();
  arma::mat I = arma::mat().eye(N_params, N_params);

  if (proj_flag)
  {
    K_kf = ( I - D.t()*arma::inv_sympd(D*D.t())*D ) * K_kf;
    this->theta = theta - D.t()*arma::inv_sympd(D*D.t())*(D*theta-d);
  }

  // =====  Calculate new covariance  =====
  this->P = (I - K_kf*H_k) * P * (I - K_kf*H_k).t() + K_kf*Rn*K_kf.t();

  this->K = K_kf;
}

arma::mat DMPoEKFa::calcStateTransFunJacob(const arma::vec &theta, void *cookie)
{
  int N_params = theta.size();
  // compute Jacobian numerically
  arma::mat F_k = arma::mat().zeros(N_params,N_params);
  arma::vec dtheta_j = arma::vec().zeros(N_params);

  for (int j=0; j<N_params; j++)
  {
      dtheta_j(j) = this->dtheta(j);
      arma::vec Ftheta2 = this->stateTransFun(theta + dtheta_j, cookie);
      arma::vec Ftheta1 = this->stateTransFun(theta - dtheta_j, cookie);
      F_k.col(j) = (Ftheta2 - Ftheta1) / (2*this->dtheta(j));
      dtheta_j(j) = 0.0;
  }

  return F_k;
}

arma::mat DMPoEKFa::calcMsrFunJacob(const arma::vec &theta, void *cookie)
{
  int N_out = this->Rn.n_rows;
  int N_params = theta.size();
  // compute Jacobian numerically
  arma::mat H_k = arma::mat().zeros(N_out,N_params);
  arma::vec dtheta_j = arma::vec().zeros(N_params);
  for (int j=0; j<N_params; j++)
  {
      dtheta_j(j) = this->dtheta(j);
      arma::vec Htheta2 = this->msrFun(theta + dtheta_j, cookie);
      arma::vec Htheta1 = this->msrFun(theta - dtheta_j, cookie);
      H_k.col(j) = (Htheta2 - Htheta1) / (2*this->dtheta(j));
      dtheta_j(j) = 0.0;
  }

  return H_k;
}

arma::vec DMPoEKFa::msrFun(const arma::vec &theta, void *cookie)
{
  return theta.subvec(0,5);
}

arma::vec DMPoEKFa::stateTransFun(const arma::vec &theta, void *cookie)
{
  arma::vec vRot = theta.subvec(0,2);
  arma::vec eQ = theta.subvec(3,5);
  arma::vec eg = theta.subvec(6,8);
  double tau = theta(9);

  arma::vec Q = quatExp(eQ);
  arma::vec Qg = quatExp(eg);

  double x = static_cast<StateTransCookie *>(cookie)->t/tau;
  double tau0 = dmp->getTau();
  dmp->setTau(tau);
  arma::vec dvRot = dmp->calcRotAccel(x, Q, vRot, Qg);
  arma::vec deQ = rotVel2deo(vRot, Q);
  dmp->setTau(tau0);

  arma::vec theta_next(10);
  theta_next.subvec(0,2) = vRot + dvRot*Ts;
  theta_next.subvec(3,5) = eQ + deQ*Ts;
  theta_next.subvec(6,9) = theta.subvec(6,9);

  return theta_next;
}

arma::vec DMPoEKFa::rotVel2deo(const arma::vec &rotVel, const arma::vec &Q)
{
  arma::vec RotVel = arma::join_vert(arma::vec({0}), rotVel);
  arma::vec deo = 0.5* DMP_eo::jacobDeoDquat(Q) * quatProd(RotVel, Q);
  return deo;
}

arma::mat DMPoEKFa::msrFunJacob(const arma::vec &theta, void *cookie)
{
  return arma::join_horiz( arma::mat().eye(6,6), arma::mat().zeros(6,4) );
}

arma::mat DMPoEKFa::stateTransFunJacob(const arma::vec &theta, void *cookie)
{
  double t = static_cast<StateTransCookie *>(cookie)->t;

  arma::mat A_k = arma::mat().zeros(10,10);
  A_k.rows(0,2) = calcF1Jacob(theta, t);
  A_k.rows(3,5) = calcF2Jacob(theta);

  return arma::mat().eye(10,10) + A_k*Ts;
}

arma::mat DMPoEKFa::calcF1Jacob(const arma::vec &theta, double t)
{
  arma::vec vRot = theta.subvec(0,2);
  arma::vec eQ = theta.subvec(3,5);
  arma::vec eg = theta.subvec(6,8);
  double tau = theta(9);

  arma::vec Q = quatExp(eQ);
  arma::vec Qg = quatExp(eg);

  double x = t/tau;

  double tau0 = dmp->getTau();

  dmp->setTau(tau);

  arma::mat F1 = arma::mat().zeros(3,10);
  arma::vec dvRot2;
  arma::vec dvRot1;

  double dvRot_step = 1e-3;
  for (int i=0; i<3; i++)
  {
    arma::vec dvRot = arma::vec().zeros(3);
    dvRot(i) = dvRot_step;
    dvRot2 = dmp->calcRotAccel(x, Q, vRot + dvRot, Qg);
    dvRot1 = dmp->calcRotAccel(x, Q, vRot - dvRot, Qg);
    F1.col(i) = (dvRot2 - dvRot1) / (2*dvRot_step);
  }

  double deQ_step = 1e-3;
  for (int i=0; i<3; i++)
  {
    arma::vec deQ = arma::vec().zeros(3);
    deQ(i) = deQ_step;
    dvRot2 = dmp->calcRotAccel(x, quatExp(eQ+deQ), vRot, Qg);
    dvRot1 = dmp->calcRotAccel(x, quatExp(eQ-deQ), vRot, Qg);
    F1.col(3+i) = (dvRot2 - dvRot1) / (2*deQ_step);
  }

  double deg_step = 1e-3;
  for (int i=0; i<3; i++)
  {
    arma::vec deQ = arma::vec().zeros(3);
    deQ(i) = deg_step;
    dvRot2 = dmp->calcRotAccel(x, Q, vRot, quatExp(eg+deQ));
    dvRot1 = dmp->calcRotAccel(x, Q, vRot, quatExp(eg-deQ));
    F1.col(6+i) = (dvRot2 - dvRot1) / (2*deg_step);
  }

  double dtau = 1e-3;
  dmp->setTau(tau+dtau);
  dvRot2 = dmp->calcRotAccel(t/(tau+dtau), Q, vRot, Qg);
  dmp->setTau(tau-dtau);
  dvRot1 = dmp->calcRotAccel(t/(tau-dtau), Q, vRot, Qg);
  F1.col(9) = (dvRot2 - dvRot1) / (2*dtau);

  dmp->setTau(tau0);

  return F1;
}

arma::mat DMPoEKFa::calcF2Jacob(const arma::vec &theta)
{
  arma::vec vRot = theta.subvec(0,2);
  arma::vec eQ = theta.subvec(3,5);
  arma::vec Q = quatExp(eQ);

  arma::mat F2 = arma::mat().zeros(3,10);
  arma::vec deo2;
  arma::vec deo1;

  double dvRot_step = 1e-3;
  for (int i=0; i<3; i++)
  {
    arma::vec dvRot = arma::vec().zeros(3);
    dvRot(i) = dvRot_step;
    deo2 = rotVel2deo(vRot + dvRot, Q);
    deo1 = rotVel2deo(vRot - dvRot, Q);
    F2.col(i) = (deo2 - deo1) / (2*dvRot_step);
  }

  double deQ_step = 1e-3;
  for (int i=0; i<3; i++)
  {
    arma::vec deQ = arma::vec().zeros(3);
    deQ(i) = deQ_step;
    deo2 = rotVel2deo(vRot, quatExp(eQ+deQ));
    deo1 = rotVel2deo(vRot, quatExp(eQ-deQ));
    F2.col(3+i) = (deo2 - deo1) / (2*deQ_step);
  }

  return F2;
}


} // namespace dmp_

} // namespace as64_
