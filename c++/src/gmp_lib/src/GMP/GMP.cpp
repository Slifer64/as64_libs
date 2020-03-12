// GMP class
// Generalized movement primitive.
//

#include <gmp_lib/GMP/GMP.h>

namespace as64_
{

namespace gmp_
{

// ===================================
// =======  Public Functions  ========
// ===================================

GMP::GMP(unsigned N_kernels, double D, double K, double kernels_std_scaling)
{
  this->D = D;
  this->K = K;
  this->shape_attr_gating_ptr.reset(new gmp_::SigmoidGatingFunction(1.0, 0.99));
  (dynamic_cast<gmp_::SigmoidGatingFunction*>(this->shape_attr_gating_ptr.get()))->setSteepness(750);

  this->wsog.reset(new WSoG(N_kernels, kernels_std_scaling));
  // this->wsogopt = this->wsog->deepCopy();

  // this->setOptTraj(false);

  this->setY0(0);
  this->setGoal(1);
}


void GMP::train(const std::string &train_method, const arma::rowvec &Time, const arma::rowvec &yd_data, double *train_error)
{
  int i_end = Time.size()-1;
  arma::rowvec x = Time / Time(i_end);
  this->wsog->train(train_method, x, yd_data, train_error);
  // this->wsogopt = this->wsog->deepCopy();
}


void GMP::updateWeights(const std::vector<gmp_::Phase> &s, const arma::rowvec &z, const std::vector<gmp_::UPDATE_TYPE> &type, const arma::rowvec &z_var_)
{
  int n = s.size();

  arma::rowvec z_var = z_var_;
  if (z_var_.size()==1) z_var = z_var(0)*arma::rowvec().ones(n);

  int k = this->numOfKernels();

  arma::mat H(n, k);
  arma::rowvec z_hat(n);
  arma::rowvec Hi;

  for (int i=0; i<n; i++)
  {
    if (type[i] == gmp_::UPDATE_TYPE::POS)
    {
      Hi = this->wsog->regressVec(s[i].x).t();
      z_hat(i) = this->wsog->output(s[i].x);
    }
    else if (type[i] == gmp_::UPDATE_TYPE::VEL)
    {
      Hi = this->wsog->regressVecDot(s[i].x, s[i].x_dot).t();
      z_hat(i) = this->wsog->outputDot(s[i].x, s[i].x_dot);
    }
    else if (type[i] == gmp_::UPDATE_TYPE::ACCEL)
    {
      Hi = this->wsog->regressVecDDot(s[i].x, s[i].x_dot, s[i].x_ddot).t();
      z_hat(i) = this->wsog->outputDDot(s[i].x, s[i].x_dot, s[i].x_ddot);
    }

    H.row(i) = Hi;
  }

  arma::vec e = (z - z_hat).t();

  this->wsog->updateWeights(H, e, arma::diagmat(z_var));

}


void GMP::update(const gmp_::Phase &s, double y, double z, double y_c, double z_c)
{
  this->z_dot = ( this->goalAttractor(y, z) + this->shapeAttractor(s) + z_c);
  this->y_dot = z + y_c;
}


double GMP::getYdot() const
{ return this->y_dot; }

double GMP::getZdot() const
{ return this->z_dot; }


double GMP::getYddot(double yc_dot) const
{
  double y_ddot = this->getZdot() + yc_dot;
  return y_ddot;
}


double GMP::calcYddot(const gmp_::Phase &s, double y, double y_dot, double yc, double zc, double yc_dot)
{
  double z = y_dot - yc;
  double z_dot = ( this->goalAttractor(y, z) + this->shapeAttractor(s) + zc);

  double y_ddot = (z_dot + yc_dot);
  return y_ddot;
}


int GMP::numOfKernels() const
{
  return this->wsog->numOfKernels();
}


void GMP::setY0(double y0)
{
  this->wsog->setStartValue(y0);
}


void GMP::setGoal(double g)
{
  this->g = g;
  this->wsog->setFinalValue(g);
}


double GMP::getYd(double x) const
{
  return this->wsog->output(x);
}


double GMP::getYdDot(double x, double x_dot) const
{
  return this->wsog->outputDot(x, x_dot);
}


double GMP::getYdDDot(double x, double x_dot, double x_ddot) const
{
  return this->wsog->outputDDot(x, x_dot, x_ddot);
}


// ======================================
// =======  Protected Functions  ========
// ======================================

double GMP::goalAttractor(double y, double z) const
{
  double goal_attr = this->K*(this->g-y)- this->D*z;
  return goal_attr;
}


double GMP::shapeAttractor(const gmp_::Phase &s) const
{
  double x = s.x;
  double sAttrGat = this->shape_attr_gating_ptr->getOutput(x);
  if (sAttrGat<0) sAttrGat = 0.0;
  double shape_attr = sAttrGat * this->forcingTerm(s);
  return shape_attr;
}


double GMP::forcingTerm(const gmp_::Phase &s) const
{
  double x = s.x;
  double x_dot = s.x_dot;
  double x_ddot = s.x_ddot;

  double yd = this->wsog->output(x);
  double yd_dot = this->wsog->outputDot(x, x_dot);
  double yd_ddot = this->wsog->outputDDot(x, x_dot, x_ddot);

  double f = yd_ddot + this->D*yd_dot + this->K*(yd - this->g);
  return f;
}

} // namespace gmp_

} // namespace as64_
