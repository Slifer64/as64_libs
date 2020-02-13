#include <dmp_lib/DMP/DMP_pos.h>
#include <dmp_lib/DMP/DMP.h>
#include <dmp_lib/DMP/DMP_bio.h>

#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>

#include <dmp_lib/io/io.h>

namespace as64_
{

namespace dmp_
{

DMP_pos::DMP_pos(dmp_::TYPE dmp_type, const arma::uvec &N_kernels, const arma::vec &a_z, const arma::vec &b_z,
                 std::shared_ptr<CanonicalClock> can_clock_ptr, std::shared_ptr<GatingFunction> shape_attr_gating_ptr)
{

  if (!can_clock_ptr) can_clock_ptr.reset(new CanonicalClock());
  if (!shape_attr_gating_ptr) shape_attr_gating_ptr.reset(new SigmoidGatingFunction(1.0, 0.5));

  this->can_clock_ptr = can_clock_ptr;
  this->shape_attr_gating_ptr = shape_attr_gating_ptr;

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
        throw std::runtime_error("[DMP_pos::DMP_pos]: Unsupported DMP type!");
    }
  }

}


void DMP_pos::train(dmp_::TrainMethod train_method, const arma::rowvec &Time, const arma::mat &Pd_data,
    const arma::mat &dPd_data, const arma::mat &ddPd_data, arma::vec *train_err)
{
  int i_end = Time.size()-1;
  double tau = Time(i_end);
  this->setTau(tau);

  int n_dim = this->dmp.size();

  if (train_err)
  {
    train_err->resize(n_dim);
    for (int i=0; i<n_dim; i++)
      this->dmp[i]->train(train_method, Time, Pd_data.row(i), dPd_data.row(i), ddPd_data.row(i), &(train_err->at(i)));
  }
  else
  {
    for (int i=0; i<n_dim; i++)
      this->dmp[i]->train(train_method, Time, Pd_data.row(i), dPd_data.row(i), ddPd_data.row(i));
  }

}


void DMP_pos::update(double x, const arma::vec &Y, const arma::vec &Z, const arma::vec &Yg, const arma::vec &Y_c, const arma::vec &Z_c)
{
  int n_dim = this->dmp.size();

  for (int i=0; i<n_dim; i++) this->dmp[i]->update(x, Y(i), Z(i), Yg(i), Y_c(i), Z_c(i));

  this->dZ.resize(n_dim);
  this->dY.resize(n_dim);
  for (int i=0; i<n_dim; i++)
  {
    this->dY(i) = this->dmp[i]->getYdot();
    this->dZ(i) = this->dmp[i]->getZdot();
  }

  this->dx = this->phaseDot(x);

}


arma::vec DMP_pos::calcYddot(double x, const arma::vec &Y, const arma::vec &dY, const arma::vec &Yg,
                             double tau_dot, const arma::vec &Yc, const arma::vec &Zc, const arma::vec &Yc_dot)
{
  int n_dim = this->dmp.size();
  arma::vec ddY(n_dim);
  for (int i=0; i<n_dim; i++)
    ddY(i) = this->dmp[i]->calcYddot(x, Y(i), dY(i), Yg(i), tau_dot, Yc(i), Zc(i), Yc_dot(i));

  return ddY;
}


arma::vec DMP_pos::getYddot(double tau_dot, const arma::vec &Yc_dot) const
{
  return (this->getZdot() + Yc_dot - tau_dot*this->getYdot()) / this->getTau();
}


void DMP_pos::setY0(const arma::vec &Y0)
{
  for (int i=0; i<dmp.size(); i++) this->dmp[i]->setY0(Y0(i));
}


arma::vec DMP_pos::getY0() const
{
  arma::vec Y0(dmp.size());
  for (int i=0; i<Y0.size(); i++) Y0(i) = this->dmp[i]->getY0();
  return Y0;
}


void DMP_pos::setTau(double tau)
{
  this->can_clock_ptr->setTau(tau);
}


double DMP_pos::getTau() const
{
  return this->can_clock_ptr->getTau();
}


double DMP_pos::phase(double t) const
{
  return this->can_clock_ptr->getPhase(t);
}


double DMP_pos::phaseDot(double x) const
{
  return this->can_clock_ptr->getPhaseDot(x);
}


void DMP_pos::exportToFile(std::ostream &out) const
{
  for (int i=0; i<dmp.size(); i++) dmp[i]->exportToFile(out);
  can_clock_ptr->exportToFile(out);
  shape_attr_gating_ptr->exportToFile(out);
}

std::shared_ptr<DMP_pos> DMP_pos::importFromFile(std::istream &in)
{
  std::shared_ptr<DMP_pos> dmp_p(new DMP_pos(dmp_::STD, {30,30,30}, {20,20,20}, {5,5,5}));

  int N_dmps = dmp_p->dmp.size();
  for (int i=0; i<N_dmps; i++) dmp_p->dmp[i] = DMP_::importFromFile(in);
  dmp_p->can_clock_ptr = CanonicalClock::importFromFile(in);
  dmp_p->shape_attr_gating_ptr = GatingFunction::importFromFile(in);

  for (int i=0; i<N_dmps; i++)
  {
    dmp_p->dmp[i]->can_clock_ptr = dmp_p->can_clock_ptr;
    dmp_p->dmp[i]->shape_attr_gating_ptr = dmp_p->shape_attr_gating_ptr;
  }

  return dmp_p;
}

arma::mat DMP_pos::getAcellPartDev_g_tau(double t, const arma::vec &Y, const arma::vec &dY, const arma::vec &Y0, double x, const arma::vec &Yg, double tau) const
{

  int n_dim = dmp.size();
  arma::mat J(n_dim, n_dim+1);

  int j_end = n_dim;
  for (int i=0; i<n_dim; i++)
  {
    arma::vec C = dmp[i]->getAcellPartDev_g_tau(t, Y(i), dY(i), Y0(i), x, Yg(i), tau);
    J(i,i) = C(0);
    J(i,j_end) = C(1);
  }

  return J;
}

} // namespace dmp_

} // namespace as64_
