#include <dmp_lib/DMP/DMP_.h>
#include <dmp_lib/trainMethods/LeastSquares.h>
#include <dmp_lib/trainMethods/LWR.h>
#include <dmp_lib/io/io.h>

#include <dmp_lib/DMP/DMP.h>
#include <dmp_lib/DMP/DMP_bio.h>

namespace as64_
{

namespace dmp_
{

DMP_::DMP_(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> can_clock_ptr,
  std::shared_ptr<GatingFunction> shape_attr_gating_ptr)
{
  this->zero_tol = 1e-30; // realmin;

  this->N_kernels = N_kernels;
  this->a_z = a_z;
  this->b_z = b_z;
  this->can_clock_ptr = can_clock_ptr;
  this->shape_attr_gating_ptr = shape_attr_gating_ptr;

  this->w = arma::vec().zeros(this->N_kernels);
  this->setCenters();
  this->setStds(1.0);
  this->setY0(0);
}


void DMP_::train(dmp_::TrainMethod train_method, const arma::rowvec &Time,
  const arma::rowvec &yd_data, const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double *train_err)
{
  int n_data = Time.size();
  int i_end = n_data-1;

  double tau = Time(i_end);
  double y0 = yd_data(0);
  double g = yd_data(i_end);

  this->setTau(tau);
  this->setY0(y0);

  arma::rowvec x(n_data);
  arma::rowvec s(n_data);
  arma::rowvec Fd(n_data);
  arma::mat Psi(this->numOfKernels(), n_data);
  for (int i=0; i<n_data; i++)
  {
    x(i) = this->phase(Time(i));
    s(i) = this->forcingTermScaling(g) * this->shapeAttrGating(x(i));
    Fd(i) = this->calcFd(x(i), yd_data(i), dyd_data(i), ddyd_data(i), g);
    Psi.col(i) = this->kernelFunction(x(i));
  }

  switch (train_method)
  {
    case dmp_::TrainMethod::LWR:
      this->w = localWeightRegress(Psi, s, Fd, this->zero_tol);
      break;
    case dmp_::TrainMethod::LS:
      this->w = leastSquares(Psi, s, Fd, this->zero_tol);
      break;
    default:
      throw std::runtime_error("[DMP_::train]: Unsopported training method");
  }


  if (train_err)
  {
    arma::rowvec F(Fd.size());
    for (int i=0; i<F.size(); i++)
    {
      F(i) = this->calcLearnedFd(x(i), g);
    }
    *train_err = arma::norm(F-Fd)/F.size();
  }

}


void DMP_::update(double x, double y, double z, double g, double y_c, double z_c)
{
  double tau = this->getTau();

  double shape_attr = this->shapeAttractor(x, g);
  double goal_attr = this->goalAttractor(x, y, z, g);

  this->dz = ( goal_attr + shape_attr + z_c) / tau;
  this->dy = ( z + y_c) / tau;
  this->dx = this->phaseDot(x);
}


double DMP_::calcYddot(double x, double y, double dy, double g, double tau_dot, double yc, double zc, double yc_dot) const
{
  double tau = this->getTau();
  double z = dy*tau - yc;

  double shape_attr = this->shapeAttractor(x, g);
  double goal_attr = this->goalAttractor(x, y, z, g);
  double dz = ( goal_attr + shape_attr + zc) / tau;

  double ddy = (dz + yc_dot - tau_dot*dy)/tau;

  return ddy;
}


double DMP_::getYddot(double tau_dot, double yc_dot) const
{
  return (this->getZdot() + yc_dot - tau_dot*this->getYdot()) / this->getTau();
}


int DMP_::numOfKernels() const
{
  return w.size();
}


void DMP_::setY0(double y0)
{
  this->y0 = y0;
}


void DMP_::setTau(double tau)
{
  this->can_clock_ptr->setTau(tau);
}


double DMP_::getTau() const
{
  return this->can_clock_ptr->getTau();
}


double DMP_::phase(double t) const
{
  return this->can_clock_ptr->getPhase(t);
}


double DMP_::phaseDot(double x) const
{
  return this->can_clock_ptr->getPhaseDot(x);
}


arma::vec DMP_::kernelFunction(double x) const
{
  arma::vec psi = arma::exp(-this->h % (arma::pow(x-this->c,2)));
  return psi;
}


double DMP_::goalAttractor(double x, double y, double z, double g) const
{
  double goal_attr = this->a_z*(this->b_z*(g-y)-z);
  goal_attr *= this->goalAttrGating(x);

  return goal_attr;
}


double DMP_::goalAttrGating(double x) const
{
  double gAttrGat = 1.0;
  return gAttrGat;
}


double DMP_::shapeAttrGating(double x) const
{
  double sAttrGat = this->shape_attr_gating_ptr->getOutput(x);
  if (sAttrGat<0) sAttrGat = 0.0;
  return sAttrGat;
}


double DMP_::forcingTerm(double x) const
{
  arma::vec Psi = this->kernelFunction(x);
  double f = arma::dot(Psi,this->w) / (arma::sum(Psi)+this->zero_tol); // add 'zero_tol' to avoid numerical issues
  return f;
}


void DMP_::setCenters()
{
  int N_kernels = this->numOfKernels();
  this->c.resize(N_kernels);
  arma::rowvec t = arma::linspace<arma::rowvec>(0,N_kernels-1, N_kernels)/(N_kernels-1);
  for (int i=0;i<t.size();i++)
  {
    this->c(i) = this->phase(t(i)*this->getTau());
  }
}


void DMP_::setStds(double kernelStdScaling)
{
  int N_kernels = this->numOfKernels();
  this->h.resize(N_kernels);
  for (int i=0; i<N_kernels-1; i++)
  {
    this->h(i) = 1 / std::pow(kernelStdScaling*(this->c(i+1)-this->c(i)),2);
  }
  this->h(N_kernels-1) = this->h(N_kernels-2);
}


arma::vec DMP_::getAcellPartDev_g_tau(double t, double y, double dy, double y0,
                                double x_hat, double g_hat, double tau_hat) const
{
  arma::vec dC_dtheta = arma::vec().zeros(2);

  double K_dmp = this->a_z*this->b_z;
  double D_dmp = this->a_z;
  arma::vec psi = this->kernelFunction(x_hat);
  double sum_psi = arma::sum(psi) + this->zero_tol;
  double sum_w_psi = arma::dot(psi, this->w);
  double shape_attr_gat = this->shapeAttrGating(x_hat);

  double theta1 = g_hat;
  double theta2 = 1/tau_hat;

  double dshape_attr_gat_dtheta2 = this->shape_attr_gating_ptr->getPartDev_1oTau(t,x_hat);

  arma::vec dPsidtheta2 = -2*t*this->h%(theta2*t-this->c)%psi;
  double sum_w_dPsidtheta2 = arma::dot(this->w, dPsidtheta2);
  double dSumWPsi_dtheta2 = (sum_w_dPsidtheta2*sum_psi - sum_w_psi*arma::sum(dPsidtheta2) ) / std::pow(sum_psi,2);

  dC_dtheta(0) = (K_dmp + shape_attr_gat*sum_w_psi/sum_psi)*std::pow(theta2,2);

  dC_dtheta(1) = 2*theta2* (K_dmp*(theta1-y) + shape_attr_gat*(theta1-y0)*sum_w_psi/sum_psi) \
      -D_dmp*dy + std::pow(theta2,2)*(theta1-y0)*( dshape_attr_gat_dtheta2*sum_w_psi/sum_psi + shape_attr_gat*dSumWPsi_dtheta2 );
  dC_dtheta(1) = dC_dtheta(1)*(-1/std::pow(tau_hat,2));

  return dC_dtheta;
}


void DMP_::exportToFile(std::ostream &out) const
{
  throw std::runtime_error("[DMP_::exportToFile]: The object must be assigned a derived class pointer!");
}


std::shared_ptr<DMP_> DMP_::importFromFile(std::istream &in)
{
  int type_int;
  dmp_::read_scalar(type_int, in);
  dmp_::TYPE type = static_cast<dmp_::TYPE>(type_int);

  switch (type)
  {
    case TYPE::STD:
      return DMP::importFromFile(in);
    case TYPE::BIO:
      return DMP_bio::importFromFile(in);
    default:
      throw std::runtime_error("[DMP_::importFromfile]: Cannot import unsupported dmp type.");
  }

}


} // namespace dmp_

} // namespace as64_
