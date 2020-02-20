#include <dmp_lib/DMP/DMPo.h>
#include <dmp_lib/DMP/DMP.h>
#include <dmp_lib/DMP/DMP_bio.h>

#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>

#include <dmp_lib/math/quaternions.h>
#include <dmp_lib/io/io.h>

namespace as64_
{

namespace dmp_
{

const long double DMPo::zero_tol = 1e-6;

DMPo::DMPo(dmp_::TYPE dmp_type, const arma::uvec &N_kernels, const arma::vec &a_z, const arma::vec &b_z,
               std::shared_ptr<CanonicalClock> can_clock_ptr, std::shared_ptr<GatingFunction> shape_attr_gating_ptr)
{
  if (!can_clock_ptr) can_clock_ptr.reset(new CanonicalClock());
  if (!shape_attr_gating_ptr) shape_attr_gating_ptr.reset(new SigmoidGatingFunction(1.0, 0.5));

  this->can_clock_ptr = can_clock_ptr;
  this->shape_attr_gating_ptr = shape_attr_gating_ptr;

  this->Q0 = arma::vec({1, 0, 0, 0});

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
        throw std::runtime_error("[DMPo::DMPo]: Unsupported DMP type!");
    }
  }

  for (int i=0; i<dmp.size(); i++) this->dmp[i]->setY0(0);

}


void DMPo::train(dmp_::TrainMethod train_method, const arma::rowvec &Time, const arma::mat &Quat_data,
                   const arma::mat &rotVel_data, const arma::mat &rotAccel_data, arma::vec *train_err)
{
  int n_data = Time.size();
  int i_end = n_data-1;
  double tau = Time(i_end);
  this->setTau(tau);

  arma::mat q_data(3, n_data);
  arma::mat qdot_data(3, n_data);
  arma::mat qddot_data(3, n_data);

  arma::vec Q0 = Quat_data.col(0);
  this->setQ0(Q0);

  for (int j=0; j<n_data; j++)
  {
    arma::vec Q1 = DMPo::quatTf(Quat_data.col(j), Q0);
    q_data.col(j) = quatLog(Q1);
    qdot_data.col(j) = DMPo::rotVel2qdot(rotVel_data.col(j), Q1);
    // qddot_data.col(j) = DMPo::rotAccel2qddot(rotAccel_data.col(j), rotVel_data.col(j), Q1);
  }

  // !! compute second derivative numerically to avoid numerical precision issues with atan2 !!
  double Ts = Time(1) - Time(0);
  for (int i=0;i<q_data.n_rows; i++)
    qddot_data.row(i) = arma::join_horiz( arma::diff(qdot_data.row(i)), arma::vec({0}) ) / Ts;

  int n_dim = this->dmp.size();
  if (train_err)
  {
    train_err->resize(n_dim);
    for (int i=0; i<n_dim; i++)
      this->dmp[i]->train(train_method, Time, q_data.row(i), qdot_data.row(i), qddot_data.row(i), &(train_err->at(i)));
  }
  else
  {
    for (int i=0; i<n_dim; i++)
      this->dmp[i]->train(train_method, Time, q_data.row(i), qdot_data.row(i), qddot_data.row(i));
  }

//  arma::vec atan2_data(n_data);
//  for (int j=0; j<n_data; j++)
//  {
//    arma::vec Q1 = DMPo::quatTf(Quat_data.col(j), Q0);
//    double w = Q1(0);
//    double v_norm = (Q1.subve(1,3)).norm();
//    atan2_data(j) = std::atan2(v_norm, w);
//  }

//  std::ofstream out("weights.bin", std::ios::out | std::ios::binary);
//  if (!out) throw std::runtime_error("Failed to create file...");
//  int n_ker = dmp[0]->N_kernels;
//  arma::mat w = arma::mat(3, n_ker);
//  for (int i=0; i<3; i++) w.row(i) = dmp[i]->w.t();
//  dmp_::write_mat(w, out, true);
//  dmp_::write_mat(q_data, out, true);
//  dmp_::write_mat(qdot_data, out, true);
//  dmp_::write_mat(qddot_data, out, true);
//  dmp_::write_mat(Quat_data, out, true);
//  out.close();


//  std::ifstream in("weights2.bin", std::ios::in | std::ios::binary);
//  if (!in) throw std::runtime_error("Failed to open file...");
//  arma::mat w2;
//  dmp_::read_mat(w2, in, true);
//  for (int i=0; i<3; i++) dmp[i]->w = w2.row(i).t();

}


void DMPo::setQ0(const arma::vec &Q0)
{
  this->Q0 = Q0;
  // for (int i=0; i < this->dmp.size(); i++) this->dmp[i]->setY0(0);
}


void DMPo::update(double x, const arma::vec &Y, const arma::vec &Z, const arma::vec &G, const arma::vec &Yc, const arma::vec &Zc)
{
  for (int i=0; i<this->dmp.size(); i++) this->dmp[i]->update(x, Y(i), Z(i), G(i), Yc(i), Zc(i));

  this->dY.resize(3);
  this->dZ.resize(3);
  for (int i=0; i<this->dmp.size(); i++)
  {
    this->dY(i) = this->dmp[i]->getYdot();
    this->dZ(i) = this->dmp[i]->getZdot();
  }

  this->dx = this->phaseDot(x);
}


arma::vec DMPo::getYddot(double tau_dot, const arma::vec &Yc_dot) const
{
  return (this->getZdot() - tau_dot*this->getYdot() + Yc_dot) / this->getTau();
}

arma::vec DMPo::getRotVel(const arma::vec &Q) const
{
  arma::vec Q1 = DMPo::quatTf(Q, this->Q0);
  arma::vec qdot = this->getYdot();
  return DMPo::qdot2rotVel(qdot, Q1);
}

arma::vec DMPo::getRotAccel(const arma::vec &Q, double tau_dot, const arma::vec &Yc_dot) const
{
  arma::vec Q1 = DMPo::quatTf(Q, this->Q0);
  arma::vec qddot = this->getYddot(tau_dot, Yc_dot);
  arma::vec rotVel = this->getRotVel(Q);
  return DMPo::qddot2rotAccel(qddot, rotVel, Q1);
}

arma::vec DMPo::calcRotAccel(double x, const arma::vec &Q, const arma::vec &rotVel, const arma::vec &Qg,
                       double tau_dot, const arma::vec &Yc, const arma::vec &Zc, const arma::vec &Yc_dot) const
{
  arma::vec a_z = {this->dmp[0]->a_z, this->dmp[1]->a_z, this->dmp[2]->a_z};
  arma::vec b_z = {this->dmp[0]->b_z, this->dmp[1]->b_z, this->dmp[2]->b_z};
  double tau = this->getTau();

  arma::vec qg = DMPo::quat2q(Qg, this->Q0);

  arma::vec Q1 = DMPo::quatTf(Q, this->Q0);
  arma::vec q = dmp_::quatLog(Q1); // DMPo::quat2q(Q, this->Q0);

  arma::mat JQq = DMPo::jacobQq(Q1);
  arma::mat JQq_dot = DMPo::jacobDotQq(Q1, rotVel);

  arma::vec fo(3);
  for (int i=0; i<3; i++) fo(i) = this->dmp[i]->shapeAttractor(x, qg(i));

  arma::vec qdot = DMPo::rotVel2qdot(rotVel, Q1);
  arma::vec qddot = (a_z%b_z%(qg - q) - tau*a_z%qdot + a_z%Yc + fo + tau*Yc_dot - tau*tau_dot*qdot + Zc) / std::pow(tau,2);

  arma::vec rotAccel = 2*quatProd(JQq_dot*qdot + JQq*qddot, quatInv(Q1));
  return rotAccel.subvec(1,3);
}

arma::vec DMPo::calcYddot(double x, const arma::vec &Y, const arma::vec &dY, const arma::vec &Yg,
                             double tau_dot, const arma::vec &Yc, const arma::vec &Zc, const arma::vec &Yc_dot)
{
  int n_dim = this->dmp.size();
  arma::vec ddY(n_dim);
  for (int i=0; i<n_dim; i++)
    ddY(i) = this->dmp[i]->calcYddot(x, Y(i), dY(i), Yg(i), tau_dot, Yc(i), Zc(i), Yc_dot(i));

  return ddY;
}


arma::vec DMPo::getY(const arma::vec &Q) const
{
  return DMPo::quat2q(Q, this->Q0);
}

arma::vec DMPo::getZ(const arma::vec &rotVel, const arma::vec &Q) const
{
  arma::vec Q1 = DMPo::quatTf(Q, this->Q0);
  arma::vec qdot = DMPo::rotVel2qdot(rotVel, Q1);
  arma::vec dy = qdot;

  return this->getTau()*dy;
}


void DMPo::setTau(double tau)
{
  this->can_clock_ptr->setTau(tau);
}


double DMPo::getTau() const
{
  return this->can_clock_ptr->getTau();
}


double DMPo::phase(double t) const
{
  return this->can_clock_ptr->getPhase(t);
}


double DMPo::phaseDot(double x) const
{
  return this->can_clock_ptr->getPhaseDot(x);
}

// ====================================================
// ****************************************************
// ************      Static Functions      ************
// ****************************************************
// ====================================================

arma::vec DMPo::quatTf(const arma::vec &Q, const arma::vec &Q0)
{
  return quatProd(Q, quatInv(Q0));
}


arma::vec DMPo::quat2q(const arma::vec &Q, const arma::vec &Q0)
{
  return quatLog(DMPo::quatTf(Q,Q0));
}


arma::vec DMPo::q2quat(const arma::vec &q, const arma::vec &Q0)
{
  return quatProd( quatExp(q), Q0);
}


arma::vec DMPo::rotVel2qdot(const arma::vec &rotVel, const arma::vec &Q1)
{
  arma::mat JqQ = DMPo::jacobqQ(Q1);
  return 0.5 * JqQ * quatProd(arma::join_vert(arma::vec({0}), rotVel), Q1);
}


arma::vec DMPo::qdot2rotVel(const arma::vec &qdot, const arma::vec &Q1)
{
  arma::mat JQq = DMPo::jacobQq(Q1);
  arma::vec rotVel = 2 * quatProd( JQq*qdot, quatInv(Q1) );
  return rotVel.subvec(1,3);
}


arma::vec DMPo::rotAccel2qddot(const arma::vec &rotAccel, const arma::vec &rotVel, const arma::vec &Q1)
{
  arma::vec rotVelQ = arma::join_vert(arma::vec({0}), rotVel);
  arma::vec rotAccelQ = arma::join_vert(arma::vec({0}), rotAccel);

  arma::mat J = DMPo::jacobqQ(Q1);
  arma::mat Jdot = DMPo::jacobDotqQ(Q1, rotVel);

  return 0.5 * (Jdot * quatProd(rotVelQ, Q1) + J * quatProd( rotAccelQ+0.5*quatProd(rotVelQ,rotVelQ), Q1 ) );
}


arma::vec DMPo::qddot2rotAccel(const arma::vec &qddot, const arma::vec &rotVel, const arma::vec &Q1)
{
  arma::vec qdot = DMPo::rotVel2qdot(rotVel, Q1);
  arma::mat J = DMPo::jacobQq(Q1);
  arma::mat Jdot = DMPo::jacobDotQq(Q1, rotVel);

  arma::vec rotAccel = 2 * ( quatProd( Jdot*qdot+J*qddot, quatInv(Q1) ) );
  return rotAccel.subvec(1,3);
}


arma::mat DMPo::jacobQq(const arma::vec &Q1)
{
  arma::mat JQq(4,3);

  if ( (1-std::fabs(Q1(0))) <= DMPo::zero_tol)
  {
    JQq.row(0) = arma::rowvec().zeros(3);
    JQq.submat(1,0,3,2) = arma::mat().eye(3,3);
    return JQq;
  }

  double w = Q1(0);
  arma::vec v = Q1.subvec(1,3);
  double norm_v = arma::norm(v);
  arma::vec eta = v / norm_v;
  double s_th = norm_v;
  double c_th = w;
  double th = std::atan2(s_th, c_th);
  arma::mat Eta = eta*eta.t();

  JQq.row(0) = -0.5 * s_th * eta.t();
  JQq.submat(1,0,3,2) = 0.5 * ( (arma::mat().eye(3,3) - Eta)*s_th/th + c_th*Eta );
  return JQq;
}


arma::mat DMPo::jacobqQ(const arma::vec &Q1)
{
  arma::mat JqQ(3,4);

  if ( (1-std::fabs(Q1(0))) <= DMPo::zero_tol)
  {
    JqQ.col(0) = arma::vec().zeros(3);
    JqQ.submat(0,1,2,3) = arma::mat().eye(3,3);
    return JqQ;
  }

  double w = Q1(0);
  arma::vec v = Q1.subvec(1,3);
  double norm_v = arma::norm(v);
  arma::vec eta = v / norm_v;
  double s_th = norm_v;
  double c_th = w;
  double th = std::atan2(s_th, c_th);

  JqQ.col(0) = 2*eta*(th*c_th - s_th)/std::pow(s_th,2);
  JqQ.submat(0,1,2,3) = 2*arma::mat().eye(3,3)*th/s_th;
  return JqQ;
}


arma::mat DMPo::jacobDotqQ(const arma::vec &Q1, const arma::vec &rotVel)
{
  arma::mat JqQ_dot(3,4);

  arma::vec qdot = DMPo::rotVel2qdot(rotVel, Q1);

  if ( (1-std::fabs(Q1(0))) <= DMPo::zero_tol)
  {
    JqQ_dot.col(0) = -qdot/3;
    JqQ_dot.submat(0,1,2,3) = arma::mat().zeros(3,3);
    return JqQ_dot;
  }

  double w = Q1(0);
  arma::vec v = Q1.subvec(1,3);
  double norm_v = arma::norm(v);
  arma::vec eta = v / norm_v;
  double s_th = norm_v;
  double c_th = w;
  double th = std::atan2(s_th, c_th);
  arma::mat Eta = eta*eta.t();
  double temp = (th*c_th-s_th)/std::pow(s_th,2);

  JqQ_dot.col(0) = ((-th/s_th - 2*c_th*temp/s_th)*Eta + temp*(arma::mat().eye(3,3)-Eta)/th)*qdot;
  JqQ_dot.submat(0,1,2,3) = (-temp*arma::dot(eta,qdot))*arma::mat().eye(3,3);
  return JqQ_dot;
}


arma::mat DMPo::jacobDotQq(const arma::vec &Q1, const arma::vec &rotVel)
{
  arma::mat JQq_dot(4,3);

  arma::vec qdot = DMPo::rotVel2qdot(rotVel, Q1);

  if ( (1-std::fabs(Q1(0))) <= DMPo::zero_tol)
  {
    JQq_dot.row(0) = -qdot.t()/4;
    JQq_dot.submat(1,0,3,2) = arma::mat().zeros(3,3);
    return JQq_dot;
  }

  double w = Q1(0);
  arma::vec v = Q1.subvec(1,3);
  double norm_v = arma::norm(v);
  arma::vec eta = v / norm_v;
  double s_th = norm_v;
  double c_th = w;
  double th = std::atan2(s_th, c_th);
  arma::mat Eta = eta*eta.t();
  arma::mat I_eta = arma::mat().eye(3,3) - Eta;
  double temp = ((th*c_th-s_th)/std::pow(th,2));

  JQq_dot.row(0) = -0.25 * qdot.t() * (c_th*Eta + (s_th/th)*I_eta);
  JQq_dot.submat(1,0,3,2) = (0.25*arma::dot(eta,qdot))*( temp*I_eta - s_th*Eta ) + 0.25*temp*( eta*(qdot.t()*I_eta) + (I_eta*qdot)*eta.t() );

  return JQq_dot;
}


void DMPo::exportToFile(std::ostream &out) const
{
  for (int i=0; i<dmp.size(); i++) dmp[i]->exportToFile(out);
  can_clock_ptr->exportToFile(out);
  shape_attr_gating_ptr->exportToFile(out);

  dmp_::write_mat(Q0, out);
}


std::shared_ptr<DMPo> DMPo::importFromFile(std::istream &in)
{
  std::shared_ptr<DMPo> dmp_o(new DMPo(dmp_::STD, {30,30,30}, {20,20,20}, {5,5,5}));

  int N_dmps = dmp_o->dmp.size();
  for (int i=0; i<N_dmps; i++) dmp_o->dmp[i] = DMP_::importFromFile(in);
  dmp_o->can_clock_ptr = CanonicalClock::importFromFile(in);
  dmp_o->shape_attr_gating_ptr = GatingFunction::importFromFile(in);

  for (int i=0; i<N_dmps; i++)
  {
    dmp_o->dmp[i]->can_clock_ptr = dmp_o->can_clock_ptr;
    dmp_o->dmp[i]->shape_attr_gating_ptr = dmp_o->shape_attr_gating_ptr;
  }

  arma::vec Q0;
  dmp_::read_mat(Q0, in);

  dmp_o->setQ0(Q0);

  return dmp_o;
}


arma::mat DMPo::getAcellPartDev_qg_tau(double t, const arma::vec &q, const arma::vec &qdot, const arma::vec &q0, double x, const arma::vec &qg, double tau) const
{

  int n_dim = dmp.size();
  arma::mat J(n_dim, n_dim+1);

  int j_end = n_dim;
  for (int i=0; i<n_dim; i++)
  {
    arma::vec C = dmp[i]->getAcellPartDev_g_tau(t, q(i), qdot(i), q0(i), x, qg(i), tau);
    J(i,i) = C(0);
    J(i,j_end) = C(1);
  }

  return J;
}

} // namespace dmp_

} // namespace as64_
