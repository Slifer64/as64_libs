#ifndef AS64_WSoG_H_
#define AS64_WSoG_H_

// WSoG class
// Approximates a smooth trajectory using a weighted sum of Gaussians and
// allows the reproduction and generalization of the learned trajectory to
// new targets and time durations through spatial and temporal scaling.
//
// Given a trajectory y(t), with t in [0 T] it encodes it through
// f(x) = phi'*w, with x in [0 1]
// so that f approximates y.
// The phase variable x is used to avoid time dependency.
// Learning of the weights 'w' can be done with 'LWR' or 'LS'.
// Generalization to new time durations is accomplished by adjusting the
// phase variables evolution (x, x_dot, x_ddot).
// Generalization to new targets is accomplished by providing a new target
// or (initial position). The spatial scaling term is then:
// ks = (g - y0) / (gd - yd0)
// and the spatially scaled trajectory:
// y = ks*(f(x) - yd0) + y0

#include <cmath>
#include <vector>
#include <cstring>
#include <memory>
#include <exception>
#include <fstream>
#include <armadillo>

#include <gmp_lib/utils.h>

namespace as64_
{

namespace gmp_
{

class GMP; // forward decleration

class WSoG
{

// ===================================
// =======  Public Functions  ========
// ===================================
public:
  // Weighted Sum of Gaussians constructor.
  // @param[in] N_kernels: The number of kernels.
  // @param[in] kernel_std_scaling: Scaling of the kernel's std. (optional, default=1.0)
  WSoG(unsigned N_kernels, double kernel_std_scaling = 1.0);

  // Returns the number of kernels.
  // @return The number of kernels.
  int numOfKernels() const;

  // Sets the initial value.
  // @param[in] f0: Initial value.
  void setStartValue(double f0);

  // Sets the final value.
  // @param[in] fg: Final value.
  void setFinalValue(double fg);

  // Returns the start value.
  // @return start value.
  double getStartValue() const;

  // Returns the final value.
  // @return final value.
  double getFinalValue() const;

  // Returns the spatial scaling.
  // @return spatial scaling.
  double getSpatialScaling() const;

  // =============================================================

  // Trains the WSoG.
  // @param[in] train_method: The training method ( {"LWR", "LS"} ).
  // @param[in] x: Row vector with the timestamps of the training data points. Must take values in [0 1].
  // @param[in] Fd: Row vector with the desired values.
  // @param[in] train_error: Optional pointer to return the training error as norm(F-Fd)/n_data.
  void train(const std::string &train_method, const arma::rowvec &x, const arma::rowvec &Fd, double *train_err=0);

  // =============================================================

  // Returns the scaled position produced by the model for a given phase variable value.
  // If a new final (or initial) value has been set, then the
  // produced position will be spatially scaled accordingly.
  // @param[in] x: The phase variable (must be in [0 1]).
  // @return (scaled) position.
  double output(double x) const;

  // Returns the scaled velocity produced by the model for a given phase variable value.
  // @param[in] x: The phase variable (must be in [0 1]).
  // @param[in] x_dot: The phase variable 1st time derivative.
  // @return (scaled) velocity.
  double outputDot(double x, double dx) const;

  // Returns the scaled acceleration produced by the model for a given phase variable value.
  // @param[in] x: The phase variable (must be in [0 1]).
  // @param[in] x_dot: The phase variable 1st time derivative.
  // @param[in] x_ddot: The phase variable 2nd time derivative.
  // @return (scaled) acceleration.
  double outputDDot(double x, double dx, double ddx) const;

  // Returns the scaled jerk produced by the model for a given phase variable value.
  // @param[in] x: The phase variable (must be in [0 1]).
  // @param[in] x_dot: The phase variable 1st time derivative.
  // @param[in] x_ddot: The phase variable 2nd time derivative.
  // @param[in] x_3dot: The phase variable 3rd time derivative.
  // @return (scaled) jerk.
  // \warning The approximation results for the jerk may not be so good...
  double output3Dot(double x, double dx, double ddx, double d3x) const;

  // =============================================================

  // Updates the weights to produce output 'p' at timestamp 'x'.
  // @param[in] x: Timestamp.
  // @param[in] p: Desired position at 'x'.
  // @param[in] sigma_p: Variance of 'p' (optional, default=WSoG::sigma_eps).
  void updatePos(double x, double p, double sigma_p=WSoG::sigma_eps);

  void updateVel(double x, double dx, double v, double sigma_v=WSoG::sigma_eps);

  void updateAccel(double x, double dx, double ddx, double a, double sigma_a=WSoG::sigma_eps);

  void updatePosVel(double x, double dx, double p, double v, const arma::vec &sigma_pv=arma::vec({WSoG::sigma_eps,WSoG::sigma_eps}) );

  void updatePosAccel(double x, double dx, double ddx, double p, double a, const arma::vec &sigma_pa=arma::vec({WSoG::sigma_eps,WSoG::sigma_eps}) );

  void updateVelAccel(double x, double dx, double ddx, double v, double a, const arma::vec &sigma_va=arma::vec({WSoG::sigma_eps,WSoG::sigma_eps}) );

  void updatePosVelAccel(double x, double dx, double ddx, double p, double v, double a, const arma::vec &sigma_pva=arma::vec({WSoG::sigma_eps,WSoG::sigma_eps,WSoG::sigma_eps}) );

  void updateWeights(const arma::mat &H, const arma::vec &e, const arma::mat &Sigma_z);

  // =============================================================

  // Returns the scaled regressor vector ks*phi.
  // @param[in] x: The phase variable (must be in [0 1]).
  // @return (scaled) regressor vector.
  arma::vec regressVec(double x) const;

  // Returns the scaled regressor vector 1st time derivative ks*phi_dot.
  // @param[in] x: The phase variable (must be in [0 1]).
  // @param[in] x_dot: The phase variable 1st time derivative.
  // @return (scaled) regressor vector 1st time derivative.
  arma::vec regressVecDot(double x, double dx) const;

  // Returns the scaled regressor vector 2nd time derivative ks*phi_ddot.
  // @param[in] x: The phase variable (must be in [0 1]).
  // @param[in] x_dot: The phase variable 1st time derivative.
  // @param[in] x_ddot: The phase variable 2nd time derivative.
  // @return (scaled) regressor vector 2nd time derivative.
  arma::vec regressVecDDot(double x, double dx, double ddx) const;

  // Returns the scaled regressor vector 3rd time derivative ks*phi_3dot.
  // @param[in] x: The phase variable (must be in [0 1]).
  // @param[in] x_dot: The phase variable 1st time derivative.
  // @param[in] x_ddot: The phase variable 2nd time derivative.
  // @param[in] x_3dot: The phase variable 3rd time derivative.
  // @return (scaled) regressor vector 3rd time derivative.
  arma::vec regressVec3Dot(double x, double dx, double ddx, double d3x) const;

// ============================================================


// ======================================
// =======  Protected Functions  ========
// ======================================
protected:

  // Calculates the spatial scaling.
  // Called whenever @setStartValue or @setFinalValue is called.
  void calcSpatialScaling();

  // =============================================================

  // Returns a column vector with the values of the kernel functions.
  // @param[in] x: The phase variable.
  // return: Column vector with the values of the kernel functions.
  arma::vec kernelFun(double x) const;

  arma::vec kernelFunDot(double x, double dx) const;

  arma::vec kernelFunDDot(double x, double dx, double ddx) const;

  arma::vec kernelFun3Dot(double x, double dx, double ddx, double d3x) const;


// =====================================
// =======  Private Properties  ========
// =====================================
protected:

  friend GMP;

  unsigned N_kernels; //number of kernels (basis functions)
  arma::vec w; //N_kernels x 1 vector with the kernels' weights
  arma::vec c; //N_kernels x 1 vector with the kernels' centers
  arma::vec h; //N_kernels x 1 vector with the kernels' inverse width

  double spat_s; //spatial scaling

  double f0_d; //initial demo value
  double fg_d; //goal of demo
  double f0; //initial value
  double fg; //goal value

  // =======  Static Properties  ========

  static double zero_tol; //small value used to avoid divisions with very small numbers
  static double sigma_eps; // default noise variance for update of weights

};

} // namespace gmp_

} // namespace as64_

#endif // AS64_WSoG_H_
