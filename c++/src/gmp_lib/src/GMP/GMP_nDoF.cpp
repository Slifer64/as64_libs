#include <gmp_lib/GMP/GMP_nDoF.h>

namespace as64_
{

namespace gmp_
{

// ===================================
// =======  Public Functions  ========
// ===================================

GMP_nDoF::GMP_nDoF(unsigned n, arma::uvec N_kernels, arma::vec D, arma::vec K, double kernels_std_scaling)
{
  #ifdef GMP_NDOF_DEBUG_
  try{
  #endif

  if (N_kernels.size() == 1) N_kernels = arma::uvec().ones(n)*N_kernels(0);
  if (D.size() == 1) D = arma::vec().ones(n)*D(0);
  if (K.size() == 1) K = arma::vec().ones(n)*K(0);
  
  this->gmp.resize(n);
  for (int i=0; i<n; i++) this->gmp[i].reset(new gmp_::GMP(N_kernels(i), D(i), K(i), kernels_std_scaling));
  
  this->setY0(arma::vec().zeros(n));
  this->setGoal(arma::vec().ones(n));
  
  this->y_dot = arma::vec().zeros(n);
  this->z_dot = arma::vec().zeros(n);

  #ifdef GMP_NDOF_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP_nDoF::GMP_nDoF]: ") + e.what()); }
  #endif
}
        

unsigned GMP_nDoF::length() const
{
  #ifdef GMP_NDOF_DEBUG_
  try{
  #endif

  return this->gmp.size();

  #ifdef GMP_NDOF_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP_nDoF::length]: ") + e.what()); }
  #endif
}


void GMP_nDoF::train(const std::string &train_method, const arma::rowvec &Time, const arma::mat &yd_data, arma::vec *train_error)
{
  #ifdef GMP_NDOF_DEBUG_
  try{
  #endif

  unsigned n = this->length();
  if (train_error)
  {
    train_error->resize(n);
    for (int i=0; i<n; i++) this->gmp[i]->train( train_method, Time, yd_data.row(i), &(train_error->at(i)) );
  }
  else
  {
    for (int i=0; i<n; i++) this->gmp[i]->train(train_method, Time, yd_data.row(i));
  }

  #ifdef GMP_NDOF_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP_nDoF::train]: ") + e.what()); }
  #endif
}


void GMP_nDoF::update(const gmp_::Phase &s, const arma::vec &y, const arma::vec &z, arma::vec y_c, arma::vec z_c)
{
  #ifdef GMP_NDOF_DEBUG_
  try{
  #endif

  unsigned n = this->length();

  if (y_c.size() == 1) y_c = arma::vec().ones(n)*y_c(0);
  if (z_c.size() == 1) z_c = arma::vec().ones(n)*z_c(0);

  for (int i=0; i<n; i++)
  {
    this->gmp[i]->update(s, y(i), z(i), y_c(i), z_c(i));
    this->y_dot(i) = this->gmp[i]->getYdot();
    this->z_dot(i) = this->gmp[i]->getZdot();
  }

  #ifdef GMP_NDOF_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP_nDoF::update]: ") + e.what()); }
  #endif
}

        
arma::vec GMP_nDoF::getYdot() const
{ return this->y_dot; }

arma::vec GMP_nDoF::getZdot() const
{ return this->z_dot; }


arma::vec GMP_nDoF::getYddot(arma::vec yc_dot) const
{
  #ifdef GMP_NDOF_DEBUG_
  try{
  #endif

  unsigned n = this->length();

  if (yc_dot.size() == 1) yc_dot = arma::vec().ones(n)*yc_dot(0);

  arma::vec y_ddot(n);
  for (int i=0; i<n; i++) y_ddot(i) = this->gmp[i]->getYddot(yc_dot(i));
  return y_ddot;

  #ifdef GMP_NDOF_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP_nDoF::getYddot]: ") + e.what()); }
  #endif
}
        

arma::vec GMP_nDoF::calcYddot(const gmp_::Phase &s, const arma::vec &y, const arma::vec &y_dot, arma::vec yc, arma::vec zc, arma::vec yc_dot) const
{
  #ifdef GMP_NDOF_DEBUG_
  try{
  #endif

  unsigned n = this->length();

  if (yc.size() == 1) yc = arma::vec().ones(n)*yc(0);
  if (zc.size()  == 1) zc = arma::vec().ones(n)*zc(0);
  if (yc_dot.size() == 1) yc_dot = arma::vec().ones(n)*yc_dot(0);

  arma::vec y_ddot(n);
  for (int i=0; i<n; i++) y_ddot(i) = this->gmp[i]->calcYddot(s, y(i), y_dot(i), yc(i), zc(i), yc_dot(i));
  return y_ddot;

  #ifdef GMP_NDOF_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP_nDoF::calcYddot]: ") + e.what()); }
  #endif
}
        

unsigned GMP_nDoF::numOfKernels(int i) const
{
  #ifdef GMP_NDOF_DEBUG_
  try{
  #endif

  return this->gmp[i]->numOfKernels();

  #ifdef GMP_NDOF_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP_nDoF::numOfKernels]: ") + e.what()); }
  #endif
}


void GMP_nDoF::setY0(const arma::vec &y0)
{
  #ifdef GMP_NDOF_DEBUG_
  try{
  #endif

  unsigned n = this->length();
  for (int i=0; i<n; i++) this->gmp[i]->setY0(y0(i));

  #ifdef GMP_NDOF_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP_nDoF::setY0]: ") + e.what()); }
  #endif
}


void GMP_nDoF::setGoal(const arma::vec &g)
{
  #ifdef GMP_NDOF_DEBUG_
  try{
  #endif

  unsigned n = this->length();
  for (int i=0; i<n; i++) this->gmp[i]->setGoal(g(i));

  #ifdef GMP_NDOF_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP_nDoF::setGoal]: ") + e.what()); }
  #endif
}

        
arma::vec GMP_nDoF::getYd(double x) const
{
  #ifdef GMP_NDOF_DEBUG_
  try{
  #endif

  unsigned n = this->length();
  arma::vec p_ref(n);
  for (int i=0; i<n; i++) p_ref(i) = this->gmp[i]->getYd(x);
  return p_ref;

  #ifdef GMP_NDOF_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP_nDoF::getYd]: ") + e.what()); }
  #endif
}


arma::vec GMP_nDoF::getYdDot(double x, double x_dot) const
{
  #ifdef GMP_NDOF_DEBUG_
  try{
  #endif

  unsigned n = this->length();
  arma::vec p_ref_dot = arma::vec().zeros(n);
  for (int i=0; i<n; i++) p_ref_dot(i) = this->gmp[i]->getYdDot(x, x_dot);
  return p_ref_dot;

  #ifdef GMP_NDOF_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP_nDoF::getYdDot]: ") + e.what()); }
  #endif
}


arma::vec GMP_nDoF::getYdDDot(double x, double x_dot, double x_ddot) const
{
  #ifdef GMP_NDOF_DEBUG_
  try{
  #endif

  unsigned n = this->length();
  arma::vec p_ref_ddot = arma::vec().zeros(n);
  for (int i=0; i<n; i++) p_ref_ddot(i) = this->gmp[i]->getYdDDot(x, x_dot, x_ddot);
  return p_ref_ddot;

  #ifdef GMP_NDOF_DEBUG_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[GMP_nDoF::getYdDDot]: ") + e.what()); }
  #endif
}


} // namespace gmp_

} // namespace as64_
