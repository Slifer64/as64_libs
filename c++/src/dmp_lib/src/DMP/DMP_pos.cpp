#include <dmp_lib/DMP/DMP_pos.h>
#include <dmp_lib/DMP/DMP.h>
#include <dmp_lib/DMP/DMP_bio.h>

#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>

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


void DMP_pos::train(dmp_::TrainMethod train_method, const arma::vec &Time, const arma::mat &Pd_data,
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


} // namespace dmp_

} // namespace as64_
