#include <dmp_lib/DMP/DMP_eo.h>
#include <dmp_lib/DMP/DMP.h>
#include <dmp_lib/DMP/DMP_bio.h>

#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>

#include <dmp_lib/math/quaternions.h>

namespace as64_
{

namespace dmp_
{

const long double DMP_eo::zero_tol = 1e-20;

DMP_eo::DMP_eo(dmp_::TYPE dmp_type, const arma::uvec &N_kernels, const arma::vec &a_z, const arma::vec &b_z,
               std::shared_ptr<CanonicalClock> can_clock_ptr, std::shared_ptr<GatingFunction> shape_attr_gating_ptr)
{
  if (!can_clock_ptr) can_clock_ptr.reset(new CanonicalClock());
  if (!shape_attr_gating_ptr) shape_attr_gating_ptr.reset(new SigmoidGatingFunction(1.0, 0.5));

  this->can_clock_ptr = can_clock_ptr;
  this->shape_attr_gating_ptr = shape_attr_gating_ptr;

  this->Q0 = arma::vec({1, 0, 0, 0});
  this->Qg = arma::vec({1, 0, 0, 0});

  dmp.resize(3);
  for (int i=0; i<dmp.size(); i++)
  {
    switch (dmp_type)
    {
      case dmp_::TYPE::STD:
        this->dmp[i].reset(new DMP(N_kernels(i), a_z(i), b_z(i), can_clock_ptr, shape_attr_gating_ptr));
        break;
      case dmp_::TYPE::BIO:
        this->dmp[i].reset(new DMP_bio(N_kernels(i), a_z(i), b_z(i), can_clock_ptr, shape_attr_gating_ptr));
        break;
      default:
        throw std::runtime_error("[DMP_eo::DMP_eo]: Unsupported DMP type!");
    }
  }

}


void DMP_eo::train(dmp_::TrainMethod train_method, const arma::rowvec &Time, const arma::mat &Quat_data,
                   const arma::mat &rotVel_data, const arma::mat &rotAccel_data, arma::vec *train_err)
{
  int n_data = Time.size();
  int i_end = n_data-1;
  double tau = Time(i_end);
  this->setTau(tau);

  arma::mat eo_data(3, n_data);
  arma::mat deo_data(3, n_data);
  arma::mat ddeo_data(3, n_data);

  arma::vec Qg = Quat_data.col(i_end);

  for (int j=0; j<n_data; j++)
  {
    arma::vec Qe = quatProd( Qg, quatInv(Quat_data.col(j)) );
    eo_data.col(j) = quatLog(Qe);
    deo_data.col(j) = DMP_eo::rotVel2deo(rotVel_data.col(j), Qe);
    ddeo_data.col(j) = DMP_eo::rotAccel2ddeo(rotAccel_data.col(j), rotVel_data.col(j), Qe);
  }

  int n_dim = this->dmp.size();
  if (train_err)
  {
    train_err->resize(n_dim);
    for (int i=0; i<n_dim; i++)
      this->dmp[i]->train(train_method, Time, eo_data.row(i), deo_data.row(i), ddeo_data.row(i), &(train_err->at(i)));
  }
  else
  {
    for (int i=0; i<n_dim; i++)
      this->dmp[i]->train(train_method, Time, eo_data.row(i), deo_data.row(i), ddeo_data.row(i));
  }

}


void DMP_eo::setQ0(const arma::vec &Q0)
{
  this->Q0 = Q0;
  arma::vec Y0 = DMP_eo::quat2eo(this->Q0, this->Qg);
  for (int i=0; i < this->dmp.size(); i++) this->dmp[i]->setY0(Y0(i));
}


void DMP_eo::setQg(const arma::vec &Qg, const arma::vec &Q, arma::vec *Y, arma::vec *Z, const arma::vec &Yc)
{
  if (Y)
  {
    arma::vec eo_old = *Y;
    arma::vec Qe_old = quatExp(eo_old);

    arma::vec eo = DMP_eo::quat2eo(Q, Qg);
    *Y = eo;

    if (Z)
    {
      arma::vec deo_old = (*Z + Yc) / this->getTau();
      arma::vec vRot = DMP_eo::deo2rotVel(deo_old, Qe_old);
      arma::vec deo = DMP_eo::rotVel2deo(vRot, quatExp(eo));
      *Z = this->getTau()*deo - Yc;
    }

  }

  this->Qg = Qg;
  arma::vec Y0 = DMP_eo::quat2eo(this->Q0, this->Qg);
  for (int i=0; i < this->dmp.size(); i++) this->dmp[i]->setY0(Y0(i));
}


void DMP_eo::update(double x, const arma::vec &Y, const arma::vec &Z, const arma::vec &Yc, const arma::vec &Zc)
{
  for (int i=0; i<this->dmp.size(); i++) this->dmp[i]->update(x, Y(i), Z(i), 0, Yc(i), Zc(i));

  this->dY.resize(3);
  this->dZ.resize(3);
  for (int i=0; i<this->dmp.size(); i++)
  {
    this->dY(i) = this->dmp[i]->getYdot();
    this->dZ(i) = this->dmp[i]->getZdot();
  }

  this->dx = this->phaseDot(x);
}


arma::vec DMP_eo::getYddot(double tau_dot, const arma::vec &Yc_dot) const
{
  return (this->getZdot() - tau_dot*this->getYdot() + Yc_dot) / this->getTau();
}

arma::vec DMP_eo::getRotVel(const arma::vec &Q, const arma::vec &Qg) const
{
  arma::vec Qe = DMP_eo::quatError(Q, Qg);
  arma::vec deo = this->getYdot();
  return DMP_eo::deo2rotVel(deo, Qe);
}

arma::vec DMP_eo::getRotAccel(const arma::vec &Q, const arma::vec &Qg, double tau_dot, const arma::vec &Yc_dot) const
{
  arma::vec Qe = DMP_eo::quatError(Q, Qg);
  arma::vec ddeo = this->getYddot(tau_dot, Yc_dot);
  arma::vec rotVel = this->getRotVel(Q, Qg);
  return DMP_eo::ddeo2rotAccel(ddeo, rotVel, Qe);
}

arma::vec DMP_eo::calcRotAccel(double x, const arma::vec &Q, const arma::vec &rotVel, const arma::vec &Qg, const arma::vec &Q0,
                       double tau_dot, const arma::vec &Yc, const arma::vec &Zc, const arma::vec &Yc_dot) const
{
  arma::vec a_z = {this->dmp[0]->a_z, this->dmp[1]->a_z, this->dmp[2]->a_z};
  arma::vec b_z = {this->dmp[0]->b_z, this->dmp[1]->b_z, this->dmp[2]->b_z};
  double tau = this->getTau();

  arma::vec Qe = DMP_eo::quatError(Q, Qg);
  arma::vec eo = DMP_eo::quat2eo(Q, Qg);
  arma::vec invQe = quatInv(Qe);

  arma::vec rotVelQ = arma::join_vert(arma::vec({0}), rotVel);
  arma::vec QeRotVel = quatProd(Qe,rotVelQ);

  arma::mat J_deo_dQ = DMP_eo::jacobDeoDquat(Qe);
  arma::mat J_dQ_deo = DMP_eo::jacobDquatDeo(Qe);
  arma::mat dJ_dQ_deo = DMP_eo::jacobDotDquatDeo(Qe, rotVel);

  arma::vec fo(3);
  arma::vec eo0 = DMP_eo::quat2eo(Q0,Qg);

  for (int i=0; i<3; i++) fo(i) = this->dmp[i]->shapeAttractor(x, 0);

  arma::vec deo = DMP_eo::rotVel2deo(rotVel, Qe);
  arma::vec ddeo = (-a_z%b_z%eo - tau*a_z%deo + a_z%Yc + fo + tau*Yc_dot - tau*tau_dot*deo + Zc) / std::pow(tau,2);

  arma::vec rotAccel1 = quatProd(invQe, dJ_dQ_deo*J_deo_dQ*QeRotVel);
  arma::vec rotAccel2 = 2*quatProd(invQe, J_dQ_deo*-ddeo);
  arma::vec rotAccel = rotAccel1 + rotAccel2;
  return rotAccel.subvec(1,3);
}

arma::vec DMP_eo::getY(const arma::vec &Q, const arma::vec &Qg) const
{
  return DMP_eo::quat2eo(Q, Qg);
}

arma::vec DMP_eo::getZ(const arma::vec &rotVel, const arma::vec &Q, const arma::vec &Qg) const
{
  arma::vec Qe = DMP_eo::quatError(Q, Qg);
  arma::vec deo = DMP_eo::rotVel2deo(rotVel, Qe);
  arma::vec dy = deo;

  return this->getTau()*dy;
}


void DMP_eo::setTau(double tau)
{
  this->can_clock_ptr->setTau(tau);
}


double DMP_eo::getTau() const
{
  return this->can_clock_ptr->getTau();
}


double DMP_eo::phase(double t) const
{
  return this->can_clock_ptr->getPhase(t);
}


double DMP_eo::phaseDot(double x) const
{
  return this->can_clock_ptr->getPhaseDot(x);
}

// ====================================================
// ****************************************************
// ************      Static Functions      ************
// ****************************************************
// ====================================================

arma::vec DMP_eo::quatError(const arma::vec &Q, const arma::vec &Qg)
{
  return quatProd(Qg, quatInv(Q));
}


arma::vec DMP_eo::quat2eo(const arma::vec &Q, const arma::vec &Qg)
{
  return quatLog(DMP_eo::quatError(Q,Qg));
}

arma::vec DMP_eo::eo2quat(const arma::vec &eo, const arma::vec &Qg)
{
  return quatProd( quatInv(quatExp(eo)), Qg);
}

arma::vec DMP_eo::rotVel2deo(const arma::vec &rotVel, const arma::vec &Qe)
{
  arma::mat J_deo_dQ = DMP_eo::jacobDeoDquat(Qe);
  return -0.5 * J_deo_dQ * quatProd(Qe, arma::join_vert(arma::vec({0}), rotVel));
}

arma::vec DMP_eo::deo2rotVel(const arma::vec &deo, const arma::vec &Qe)
{
  arma::mat J_dQ_deo = DMP_eo::jacobDquatDeo(Qe);
  arma::vec rotVel = -2 * quatProd( quatInv(Qe), J_dQ_deo*deo );
  return rotVel.subvec(1,3);
}

arma::vec DMP_eo::rotAccel2ddeo(const arma::vec &rotAccel, const arma::vec &rotVel, const arma::vec &Qe)
{
  arma::vec rotVelQ = arma::join_vert(arma::vec({0}), rotVel);
  arma::vec rotAccelQ = arma::join_vert(arma::vec({0}), rotAccel);

  arma::mat J = DMP_eo::jacobDeoDquat(Qe);
  arma::mat dJ = DMP_eo::jacobDotDeoDquat(Qe, rotVel);

  return -0.5 * (dJ * quatProd(Qe, rotVelQ) + J * quatProd( Qe, rotAccelQ-0.5*quatProd(rotVelQ,rotVelQ) ) );
}

arma::vec DMP_eo::ddeo2rotAccel(const arma::vec &ddeo, const arma::vec &rotVel, const arma::vec &Qe)
{
  arma::vec deo = DMP_eo::rotVel2deo(rotVel, Qe);
  arma::vec invQe = quatInv(Qe);
  arma::mat J = DMP_eo::jacobDquatDeo(Qe);
  arma::mat dJ = DMP_eo::jacobDotDquatDeo(Qe, rotVel);

  arma::vec rotAccel = - 2 * (quatProd(invQe, dJ*deo) + quatProd(invQe, J*ddeo));
  return rotAccel.subvec(1,3);
}

arma::mat DMP_eo::jacobDquatDeo(const arma::vec &Qe)
{
  arma::mat J_dQ_deo(4,3);

  if (std::fabs(Qe(0)-1) <= DMP_eo::zero_tol)
  {
    J_dQ_deo.row(0) = arma::rowvec().zeros(3);
    J_dQ_deo.submat(1,0,3,2) = arma::mat().eye(3,3);
    return J_dQ_deo;
  }

  double w = Qe(0);
  arma::vec v = Qe.subvec(1,3);
  double norm_v = arma::norm(v);
  arma::vec eta = v / norm_v;
  double s_th = norm_v;
  double c_th = w;
  double th = std::atan2(s_th, c_th);
  arma::mat Eta = eta*eta.t();

  J_dQ_deo.row(0) = -0.5 * s_th * eta.t();
  J_dQ_deo.submat(1,0,3,2) = 0.5 * ( (arma::mat().eye(3,3) - Eta)*s_th/th + c_th*Eta );
  return J_dQ_deo;
}

arma::mat DMP_eo::jacobDeoDquat(const arma::vec &Qe)
{
  arma::mat J_deo_dQ(3,4);

  if (std::fabs(Qe(0)-1) <= DMP_eo::zero_tol)
  {
    J_deo_dQ.col(0) = arma::vec().zeros(3);
    J_deo_dQ.submat(0,1,2,3) = arma::mat().eye(3,3);
    return J_deo_dQ;
  }

  double w = Qe(0);
  arma::vec v = Qe.subvec(1,3);
  double norm_v = arma::norm(v);
  arma::vec eta = v / norm_v;
  double s_th = norm_v;
  double c_th = w;
  double th = std::atan2(s_th, c_th);

  J_deo_dQ.col(0) = 2*eta*(th*c_th - s_th)/std::pow(s_th,2);
  J_deo_dQ.submat(0,1,2,3) = 2*arma::mat().eye(3,3)*th/s_th;
  return J_deo_dQ;
}

arma::mat DMP_eo::jacobDotDeoDquat(const arma::vec &Qe, const arma::vec &rotVel)
{
  arma::mat dJ_deo_dQ(3,4);

  arma::vec deo = DMP_eo::rotVel2deo(rotVel, Qe);

  if (std::fabs(Qe(0)-1) <= DMP_eo::zero_tol)
  {
    dJ_deo_dQ.col(0) = -deo/3;
    dJ_deo_dQ.submat(0,1,2,3) = arma::mat().zeros(3,3);
    return dJ_deo_dQ;
  }

  double w = Qe(0);
  arma::vec v = Qe.subvec(1,3);
  double norm_v = arma::norm(v);
  arma::vec eta = v / norm_v;
  double s_th = norm_v;
  double c_th = w;
  double th = std::atan2(s_th, c_th);
  arma::mat Eta = eta*eta.t();
  double temp = (th*c_th-s_th)/std::pow(s_th,2);

  dJ_deo_dQ.col(0) = ((-th/s_th - 2*c_th*temp/s_th)*Eta + temp*(arma::mat().eye(3,3)-Eta)/th)*deo;
  dJ_deo_dQ.submat(0,1,2,3) = -temp*eta.t()*deo*arma::mat().eye(3,3);
  return dJ_deo_dQ;
}

arma::mat DMP_eo::jacobDotDquatDeo(const arma::vec &Qe, const arma::vec &rotVel)
{
  arma::mat dJ_dQ_deo(4,3);

  arma::vec deo = DMP_eo::rotVel2deo(rotVel, Qe);

  if (std::fabs(Qe(0)-1) <= DMP_eo::zero_tol)
  {
    dJ_dQ_deo.row(0) = -deo.t()/4;
    dJ_dQ_deo.submat(1,0,3,2) = arma::mat().zeros(3,3);
    return dJ_dQ_deo;
  }

  double w = Qe(0);
  arma::vec v = Qe.subvec(1,3);
  double norm_v = arma::norm(v);
  arma::vec eta = v / norm_v;
  double s_th = norm_v;
  double c_th = w;
  double th = std::atan2(s_th, c_th);
  arma::mat Eta = eta*eta.t();
  arma::mat I_eta = arma::mat().eye(3,3) - Eta;
  double temp = ((th*c_th-s_th)/std::pow(th,2));

  dJ_dQ_deo.row(0) = -0.25 * deo.t() * (c_th*Eta + (s_th/th)*I_eta);
  dJ_dQ_deo.submat(1,0,3,2) = 0.25*(eta.t()*deo)*( temp*I_eta - s_th*Eta ) + 0.25*temp*( eta*(deo.t()*I_eta) + (I_eta*deo)*eta.t() );

  return dJ_dQ_deo;
}

} // namespace dmp_

} // namespace as64_
