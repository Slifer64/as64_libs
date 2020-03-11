// WSoG class
//  Weighted sum of Guassians.
//

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

class WSoG
{

// ===================================
// =======  Public Functions  ========
// ===================================
public:
  // Weighted Sum of Gaussians constructor.
  // @param[in] N_kernels: The number of kernels.
  // @param[in] kernel_std_scaling: Scaling of the kernel.t()s std. (optional, default=1.0)
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

  // =============================================================

  // Returns the normalized weighted sum of the Gaussians for the given phase variable (time instant).
  // @param[in] x: The phase variable.
  // @param[out] f: The normalized weighted sum of the Gaussians.
  //
  double output(double x) const;

  double outputDot(double x, double dx) const;

  double outputDDot(double x, double dx, double ddx) const;

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

  // Trains the WSoG.
  // @param[in] train_method: The training method ( {"LWR", "LS"} ).
  // @param[in] x: Row vector with the timestamps of the training data points. Must take values in [0 1].
  // @param[in] Fd: Row vector with the desired values.
  // @param[in] train_error: Optional pointer to return the training error as norm(F-Fd)/n_data.
  void train(const std::string &train_method, const arma::rowvec &x, const arma::rowvec &Fd, double *train_err=0);


  // =============================================================

  arma::vec regressVec(double x) const;

  arma::vec regressVecDot(double x, double dx) const;

  arma::vec regressVecDDot(double x, double dx, double ddx) const;

  arma::vec regressVec3Dot(double x, double dx, double ddx, double d3x) const;

// ============================================================

  double getStartValue() const;

  double getFinalValue() const;

  double getSpatialScaling() const;

// =============================================================

// ======================================
// =======  Protected Functions  ========
// ======================================
protected:

  void calcSpatialScaling();

  // =============================================================

  // Returns a column vector with the values of the kernel functions.
  // @param[in] x: The phase variable.
  // @return: Column vector with the values of the kernel functions.
  arma::vec kernelFun(double x) const;

  arma::vec kernelFunDot(double x, double dx) const;

  arma::vec kernelFunDDot(double x, double dx, double ddx) const;

  arma::vec kernelFun3Dot(double x, double dx, double ddx, double d3x) const;


// =====================================
// =======  Private Properties  ========
// =====================================
private:
  unsigned N_kernels; //number of kernels (basis functions)
  arma::vec w; //N_kernels x 1 vector with the kernels' weights
  arma::vec c; //N_kernels x 1 vector with the kernels' centers
  arma::vec h; //N_kernels x 1 vector with the kernels' inverse width

  double spat_s; //spatial scaling

  double f0_d; //initial demo value
  double fg_d; //goal of demo
  double f0; //initial value
  double fg; //goal value

  static double zero_tol; //small value used to avoid divisions with very small numbers
  static double sigma_eps; //minumum noise variance for update of weights

};

} // namespace gmp_

} // namespace as64_