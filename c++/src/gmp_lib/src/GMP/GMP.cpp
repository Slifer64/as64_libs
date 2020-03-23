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
  #ifdef GMP_DEBUG_
  try{
  #endif

  this->D = D;
  this->K = K;
  this->shape_attr_gating_ptr.reset(new gmp_::SigmoidGatingFunction(1.0, 0.99));
  (dynamic_cast<gmp_::SigmoidGatingFunction*>(this->shape_attr_gating_ptr.get()))->setSteepness(750);

  this->wsog.reset(new WSoG(N_kernels, kernels_std_scaling));
  // this->wsogopt = this->wsog->deepCopy();

  // this->setOptTraj(false);

  this->setY0(0);
  this->setGoal(1);

  this->y_dot = 0;
  this->z_dot = 0;

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::GMP]: ") + e.what()); }
  #endif
}


void GMP::train(const std::string &train_method, const arma::rowvec &Time, const arma::rowvec &yd_data, double *train_error)
{
  #ifdef GMP_DEBUG_
  try{
  #endif

  int i_end = Time.size()-1;
  arma::rowvec x = Time / Time(i_end);
  this->wsog->train(train_method, x, yd_data, train_error);
  // this->wsogopt = this->wsog->deepCopy();

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::train]: ") + e.what()); }
  #endif
}


void GMP::updateWeights(const std::vector<gmp_::Phase> &s, const arma::rowvec &z, const std::vector<gmp_::UPDATE_TYPE> &type, const arma::rowvec &z_var_)
{
  #ifdef GMP_DEBUG_
  try{
  #endif
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

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::updateWeights]: ") + e.what()); }
  #endif
}


void GMP::update(const gmp_::Phase &s, double y, double z, double y_c, double z_c)
{
  #ifdef GMP_DEBUG_
  try{
  #endif

  this->z_dot = ( this->goalAttractor(y, z) + this->shapeAttractor(s) + z_c);
  this->y_dot = z + y_c;

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::update]: ") + e.what()); }
  #endif
}


double GMP::getYdot() const
{ return this->y_dot; }

double GMP::getZdot() const
{ return this->z_dot; }


double GMP::getYddot(double yc_dot) const
{
  #ifdef GMP_DEBUG_
  try{
  #endif

  double y_ddot = this->getZdot() + yc_dot;
  return y_ddot;

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::getYddot]: ") + e.what()); }
  #endif
}


double GMP::calcYddot(const gmp_::Phase &s, double y, double y_dot, double yc, double zc, double yc_dot) const
{
  #ifdef GMP_DEBUG_
  try{
  #endif

  double z = y_dot - yc;
  double z_dot = ( this->goalAttractor(y, z) + this->shapeAttractor(s) + zc);

  double y_ddot = (z_dot + yc_dot);
  return y_ddot;

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::calcYddot]: ") + e.what()); }
  #endif
}


int GMP::numOfKernels() const
{
  #ifdef GMP_DEBUG_
  try{
  #endif

  return this->wsog->numOfKernels();

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::numOfKernels]: ") + e.what()); }
  #endif
}


void GMP::setY0(double y0)
{
  #ifdef GMP_DEBUG_
  try{
  #endif

  this->wsog->setStartValue(y0);

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::setY0]: ") + e.what()); }
  #endif
}


void GMP::setGoal(double g)
{
  #ifdef GMP_DEBUG_
  try{
  #endif

  this->g = g;
  this->wsog->setFinalValue(g);

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::setGoal]: ") + e.what()); }
  #endif
}


double GMP::getYd(double x) const
{
  #ifdef GMP_DEBUG_
  try{
  #endif

  return this->wsog->output(x);

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::getYd]: ") + e.what()); }
  #endif
}


double GMP::getYdDot(double x, double x_dot) const
{
  #ifdef GMP_DEBUG_
  try{
  #endif

  return this->wsog->outputDot(x, x_dot);

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::getYdDot]: ") + e.what()); }
  #endif
}


double GMP::getYdDDot(double x, double x_dot, double x_ddot) const
{
  #ifdef GMP_DEBUG_
  try{
  #endif

  return this->wsog->outputDDot(x, x_dot, x_ddot);

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::getYdDDot]: ") + e.what()); }
  #endif
}


// ======================================
// =======  Protected Functions  ========
// ======================================

double GMP::goalAttractor(double y, double z) const
{
  #ifdef GMP_DEBUG_
  try{
  #endif

  double goal_attr = this->K*(this->g-y)- this->D*z;
  return goal_attr;

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::goalAttractor]: ") + e.what()); }
  #endif
}


double GMP::shapeAttractor(const gmp_::Phase &s) const
{
  #ifdef GMP_DEBUG_
  try{
  #endif

  double x = s.x;
  double sAttrGat = this->shape_attr_gating_ptr->getOutput(x);
  if (sAttrGat<0) sAttrGat = 0.0;
  double shape_attr = sAttrGat * this->forcingTerm(s);
  return shape_attr;

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::shapeAttractor]: ") + e.what()); }
  #endif
}


double GMP::forcingTerm(const gmp_::Phase &s) const
{
  #ifdef GMP_DEBUG_
  try{
  #endif

  double x = s.x;
  double x_dot = s.x_dot;
  double x_ddot = s.x_ddot;

  double yd = this->wsog->output(x);
  double yd_dot = this->wsog->outputDot(x, x_dot);
  double yd_ddot = this->wsog->outputDDot(x, x_dot, x_ddot);

  double f = yd_ddot + this->D*yd_dot + this->K*(yd - this->g);
  return f;

  #ifdef GMP_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP::forcingTerm]: ") + e.what()); }
  #endif
}

} // namespace gmp_

} // namespace as64_
