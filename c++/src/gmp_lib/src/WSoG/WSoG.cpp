#include <gmp_lib/WSoG/WSoG.h>
#include <gmp_lib/trainMethods/LeastSquares.h>
#include <gmp_lib/trainMethods/LWR.h>

namespace as64_
{

namespace gmp_
{

double WSoG::zero_tol = 1e-30;
double WSoG::sigma_eps = 1e-3;

// ===================================
// =======  Public Functions  ========
// ===================================

WSoG::WSoG(unsigned N_kernels, double kernel_std_scaling)
{
  this->N_kernels = N_kernels;

  this->w = arma::vec(this->N_kernels);
  this->c = arma::linspace<arma::vec>(0,N_kernels-1, N_kernels)/(N_kernels-1);
  double hi = 1 / std::pow(kernel_std_scaling*(this->c(1)-this->c(0)),2);
  this->h = arma::vec().ones(this->N_kernels) * hi;

  this->f0_d = 0;
  this->fg_d = 1;
  this->f0 = this->f0_d;
  this->setFinalValue(this->fg_d);
}


int WSoG::numOfKernels() const
{
  return this->w.size();
}


void WSoG::setStartValue(double f0)
{
  this->f0 = f0;
  this->calcSpatialScaling();
}


void WSoG::setFinalValue(double fg)
{
  this->fg = fg;
  this->calcSpatialScaling();
}

// =============================================================

double WSoG::output(double x) const
{
  arma::vec Phi = this->regressVec(x);
  double f = arma::dot(Phi,this->w) - this->spat_s*this->f0_d + this->f0;
  return f;
}

double WSoG::outputDot(double x, double dx) const
{
  arma::vec Phi_dot = this->regressVecDot(x,dx);
  double f_dot = arma::dot(Phi_dot,this->w);
  return f_dot;
}

double WSoG::outputDDot(double x, double dx, double ddx) const
{
  arma::vec Phi_ddot = this->regressVecDDot(x, dx, ddx);
  double f_ddot = arma::dot(Phi_ddot, this->w);
  return f_ddot;
}

double WSoG::output3Dot(double x, double dx, double ddx, double d3x) const
{
  arma::vec Phi_3dot = this->regressVec3Dot(x, dx, ddx, d3x);
  double f_3dot = arma::dot(Phi_3dot, this->w);
  return f_3dot;
}

// =============================================================

void WSoG::updatePos(double x, double p, double sigma_p)
{
  arma::mat H = this->regressVec(x).t();
  double p_hat = this->output(x);
  double e = p - p_hat;
  this->updateWeights(H, arma::vec({e}), arma::mat({sigma_p}));
}

void WSoG::updateVel(double x, double dx, double v, double sigma_v)
{
  arma::mat H = this->regressVecDot(x, dx).t();
  arma::vec v_hat = H*this->w;
  arma::vec e = arma::vec({v}) - v_hat;
  this->updateWeights(H, e, arma::mat({sigma_v}));
}

void WSoG::updateAccel(double x, double dx, double ddx, double a, double sigma_a)
{
  arma::mat H = this->regressVecDDot(x, dx, ddx).t();
  arma::vec a_hat = H*this->w;
  arma::vec e = arma::vec({a}) - a_hat;
  this->updateWeights(H, e, arma::mat({sigma_a}));
}

void WSoG::updatePosVel(double x, double dx, double p, double v, const arma::vec &sigma_pv)
{
  arma::mat H = arma::join_vert(this->regressVec(x).t(), this->regressVecDot(x, dx).t());
  double p_hat = this->output(x);
  double v_hat = this->outputDot(x, dx);
  arma::vec e = arma::vec({p, v}) - arma::vec({p_hat, v_hat});
  this->updateWeights(H, e, arma::diagmat(sigma_pv));
}

void WSoG::updatePosAccel(double x, double dx, double ddx, double p, double a, const arma::vec &sigma_pa)
{
  arma::mat H = arma::join_vert(this->regressVec(x).t(), this->regressVecDDot(x, dx, ddx).t());
  double p_hat = this->output(x);
  double a_hat = this->outputDDot(x, dx, ddx);
  arma::vec e = arma::vec({p, a}) - arma::vec({p_hat, a_hat});
  this->updateWeights(H, e, arma::diagmat(sigma_pa));
}

void WSoG::updateVelAccel(double x, double dx, double ddx, double v, double a, const arma::vec &sigma_va)
{
  arma::mat H = arma::join_vert(this->regressVecDot(x, dx).t(), this->regressVecDDot(x, dx, ddx).t());
  arma::vec va_hat = H*this->w;
  arma::vec e = arma::vec({v, a}) - va_hat;
  this->updateWeights(H, e, arma::diagmat(sigma_va));
}

void WSoG::updatePosVelAccel(double x, double dx, double ddx, double p, double v, double a, const arma::vec &sigma_pva)
{
  arma::mat H(3, this->N_kernels);
  H.row(0) = this->regressVec(x).t();
  H.row(1) = this->regressVecDot(x, dx).t();
  H.row(2) = this->regressVecDDot(x, dx, ddx).t();
  arma::vec pva_hat = H*this->w;
  pva_hat(0) += -this->spat_s*this->f0_d + this->f0;
  arma::vec e = arma::vec({p, v, a}) - pva_hat;
  this->updateWeights(H, e, arma::diagmat(sigma_pva));
}

void WSoG::updateWeights(const arma::mat &H, const arma::vec &e, const arma::mat &Sigma_z)
{
  arma::mat Sigma_w = arma::mat().eye(this->N_kernels, this->N_kernels);
  // arma::mat K = arma::solve( (Sigma_z + H*Sigma_w*H.t()).t(), (Sigma_w*H.t()).t() , arma::solve_opts::fast+arma::solve_opts::likely_sympd).t();
  arma::mat K = arma::solve( (Sigma_z + H*Sigma_w*H.t()), H*Sigma_w , arma::solve_opts::fast+arma::solve_opts::likely_sympd).t();
  this->w = this->w + K*e;
}

// =============================================================

void WSoG::train(const std::string &train_method, const arma::rowvec &x, const arma::rowvec &Fd, double *train_err)
{
  int n_data = x.size();

  arma::rowvec s = arma::rowvec().ones(n_data);
  arma::mat Psi(this->N_kernels, n_data);
  for (int i=0; i<n_data; i++) Psi.col(i) = this->kernelFun(x(i));

  if ( train_method.compare("LS") == 0 ) this->w = leastSquares(Psi, s, Fd, this->zero_tol);
  else if ( train_method.compare("LWR") == 0 ) this->w = localWeightRegress(Psi, s, Fd, this->zero_tol);
  else throw std::runtime_error("[GMP_::train]: Unsupported training method");

  this->f0_d = arma::dot(this->regressVec(0),this->w);
  this->fg_d = arma::dot(this->regressVec(1),this->w);
  this->setStartValue(this->f0_d);
  this->setFinalValue(this->fg_d);

  if (train_err)
  {
    arma::rowvec F(Fd.size());
    for (int i=0; i<F.size(); i++) F(i) = this->output(x(i));
    *train_err = arma::norm(F-Fd)/F.size();
  }

}

// =============================================================

arma::vec WSoG::regressVec(double x) const
{
  arma::vec psi = this->kernelFun(x);
  arma::vec phi = this->spat_s * psi / (arma::sum(psi) + this->zero_tol);
  return phi;
}

arma::vec WSoG::regressVecDot(double x, double dx) const
{
  arma::vec psi = this->kernelFun(x);
  arma::vec psi_dot = this->kernelFunDot(x, dx);
  double sum_psi = arma::sum(psi);
  double sum_psi_dot = arma::sum(psi_dot);

  arma::vec phi = psi / ( arma::sum(sum_psi) + this->zero_tol );
  arma::vec phi_dot =  this->spat_s * ( psi_dot - phi*sum_psi_dot ) / ( sum_psi + this->zero_tol);
  return phi_dot;
}

arma::vec WSoG::regressVecDDot(double x, double dx, double ddx) const
{
  arma::vec psi = this->kernelFun(x);
  arma::vec psi_dot = this->kernelFunDot(x, dx);
  arma::vec psi_ddot = this->kernelFunDDot(x, dx, ddx);
  double sum_psi = arma::sum(psi);
  double sum_psi_dot = arma::sum(psi_dot);
  double sum_psi_ddot = arma::sum(psi_ddot);

  arma::vec phi = psi / ( arma::sum(sum_psi) + this->zero_tol );
  arma::vec phi_dot = ( psi_dot - phi*sum_psi_dot ) / ( sum_psi + this->zero_tol);
  arma::vec phi_ddot = this->spat_s * (psi_ddot - 2*phi_dot*sum_psi_dot - phi*sum_psi_ddot) / ( sum_psi + this->zero_tol);
  return phi_ddot;
}

arma::vec WSoG::regressVec3Dot(double x, double dx, double ddx, double d3x) const
{
  arma::vec psi = this->kernelFun(x);
  arma::vec psi_dot = this->kernelFunDot(x, dx);
  arma::vec psi_ddot = this->kernelFunDDot(x, dx, ddx);
  arma::vec psi_3dot = this->kernelFun3Dot(x, dx, ddx, d3x);
  double sum_psi = arma::sum(psi);
  double sum_psi_dot = arma::sum(psi_dot);
  double sum_psi_ddot = arma::sum(psi_ddot);
  double sum_psi_3dot = arma::sum(psi_3dot);

  arma::vec phi = psi / ( arma::sum(sum_psi) + this->zero_tol );
  arma::vec phi_dot = ( psi_dot - phi*sum_psi_dot ) / ( sum_psi + this->zero_tol);
  arma::vec phi_ddot = (psi_ddot - 2*phi_dot*sum_psi_dot - phi*sum_psi_ddot) / ( sum_psi + this->zero_tol);
  arma::vec phi_3dot = this->spat_s * (psi_3dot - 3*phi_ddot*sum_psi_dot - 3*phi_dot*sum_psi_ddot - phi*sum_psi_3dot) / ( sum_psi + this->zero_tol);
  return phi_3dot;
}

// ============================================================

double WSoG::getStartValue() const
{
  return this->f0;
}

double WSoG::getFinalValue() const
{
  return this->fg;
}

double WSoG::getSpatialScaling() const
{
  return this->spat_s;
}

// =============================================================

// ======================================
// =======  Protected Functions  ========
// ======================================

void WSoG::calcSpatialScaling()
{
  this->spat_s = (this->fg - this->f0) / (this->fg_d - this->f0_d);
}

// =============================================================

arma::vec WSoG::kernelFun(double x) const
{
  arma::vec psi = arma::exp(-this->h % (arma::pow(x-this->c,2)));
  return psi;
}

arma::vec WSoG::kernelFunDot(double x, double dx) const
{
  arma::vec psi = this->kernelFun(x);
  arma::vec a = (x-this->c)*dx;
  arma::vec psi_dot = -2 * this->h % ( psi % a );
  return psi_dot;
}

arma::vec WSoG::kernelFunDDot(double x, double dx, double ddx) const
{
  arma::vec psi = this->kernelFun(x);
  arma::vec psi_dot = this->kernelFunDot(x, dx);

  arma::vec a = (x-this->c)*dx;
  arma::vec a_dot = (x-this->c)*ddx + std::pow(dx,2);
  arma::vec psi_ddot = -2*this->h%( psi_dot%a + psi%a_dot );

  return psi_ddot;
}

arma::vec WSoG::kernelFun3Dot(double x, double dx, double ddx, double d3x) const
{
  arma::vec psi = this->kernelFun(x);
  arma::vec psi_dot = this->kernelFunDot(x, dx);
  arma::vec psi_ddot = this->kernelFunDDot(x, dx, ddx);

  arma::vec a = (x-this->c)*dx;
  arma::vec a_dot = (x-this->c)*ddx + std::pow(dx,2);
  arma::vec a_ddot = (x-this->c)*d3x + 3*dx*ddx;
  arma::vec psi_3dot = -2*this->h%( psi_ddot%a + 2*psi_dot%a_dot + psi%a_ddot );

  return psi_3dot;
}

} // namespace gmp_

} // namespace as64_
