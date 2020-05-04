#ifndef AS64_GMP_H_
#define AS64_GMP_H_

// GMP class
// Generalized movement primitive.
//

#include <gmp_lib/WSoG/WSoG.h>
#include <gmp_lib/utils.h>

namespace as64_
{

namespace gmp_
{
  
class GMP
{

// ===================================
// =======  Public Functions  ========
// ===================================
public:
  // GMP constructor.
  // @param[in] N_kernels: number of kernels
  // @param[in] D: damping.
  // @param[in] K: stiffness.
  // @param[in] kernels_std_scaling: Scaling for std of kernels (optional, default=1).
  GMP(unsigned N_kernels, double D, double K, double kernels_std_scaling=1.0);


  // Trains the GMP.
  // @param[in] train_method: the training method to use, as a string ('LWR', 'LS').
  // @param[in] Time: Row vector with the timestamps of the training data points.
  // @param[in] yd_data: Row vector with the desired position.
  // @param[out] train_error: The training error expressed as the mse error.
  void train(const std::string &train_method, const arma::rowvec &Time, const arma::rowvec &yd_data, double *train_error=0);


  // Returns the derivatives of the GMP states.
  // @param[in] s: phase variable state, see @Phase.
  // @param[in] y: 'y' state of the GMP.
  // @param[in] z: 'z' state of the GMP.
  // @param[in] y_c: coupling term for the dynamical equation of the 'y' state (optional, default=0).
  // @param[in] z_c: coupling term for the dynamical equation of the 'z' state (optional, default=0).
  void update(const gmp_::Phase &s, double y, double z, double y_c=0, double z_c=0);

  // Returns the 'y' state time derivative.
  // Call after @update.
  // @return: time derivative of 'y' state.
  double getYdot() const;

  // Returns the 'z' state time derivative.
  // Call after @update.
  // @return: time derivative of 'z' state.
  double getZdot() const;

  // Returns the GMP's acceleration.
  // Call after @update.
  // @param[in] yc_dot: time derivative of the 'y' state coupling (optional, default=0).
  // @return: acceleration.
  double getYddot(double yc_dot = 0) const;

  // Calculates the GMP's acceleration.
  // @param[in] s: phase variable state, see @Phase.
  // @param[in] y: 'y' state of the GMP.
  // @param[in] y_dot: time derivative of 'y' state.
  // @param[in] y_c: coupling term for the dynamical equation of the 'y' state (optional, default=0).
  // @param[in] z_c: coupling term for the dynamical equation of the 'z' state (optional, default=0).
  // @param[in] yc_dot: time derivative of the 'y' state coupling (optional, default=0).
  // @return: acceleration.
  double calcYddot(const gmp_::Phase &s, double y, double y_dot, double yc=0, double zc=0, double yc_dot=0) const;

  // Returns the number of kernels.
  // @return: number of kernels.
  int numOfKernels() const;

  // Sets the initial position.
  // @param[in] y0: initial position.
  void setY0(double y0);

  // Set goal position.
  // @param[in] g: goal position.
  void setGoal(double g);

  // TODO
  // Creates a deep copy of this object
  //std::shared_ptr<GMP> deepCopy() const

  // Returns the scaled desired position.
  // @param[in] x: phase variable.
  // @return: scaled desired position.
  double getYd(double x) const;

  // Returns the scaled desired velocity.
  // @param[in] x: phase variable.
  // @param[in] x_dot: 1st time derivative of the phase variable.
  // @return: scaled desired velocity.
  double getYdDot(double x, double x_dot) const;

  // Returns the scaled desired acceleration.
  // @param[in] x: phase variable.
  // @param[in] x_dot: 1st time derivative of the phase variable.
  // @param[in] x_ddot: 2nd time derivative of the phase variable.
  // @return: scaled desired acceleration.
  double getYdDDot(double x, double x_dot, double x_ddot) const;

  // TODO
  // void constrOpt(double T, pos_constr, vel_constr, accel_constr, opt_set)

  // Updates the weights so that the generated trajectory passes from the given points.
  // @param[in] s: Vector where the i-th element is the i-th phase variable state, see @Phase.
  // @param[in] z: Row vector with the desired value for each timestamp.
  // @param[in] type: Row vector with the type of each point (GMP_UPDATE_TYPE).
  // @param[in] z_var: Row vector with the variance of each point (optional, default = 1e-3).
  void updateWeights(const std::vector<gmp_::Phase> &s, const arma::rowvec &z, const std::vector<gmp_::UPDATE_TYPE> &type, const arma::rowvec &z_var_=arma::rowvec({1e-3}) );

// ====================================
// =======  Protected Methods  ========
// ====================================

  // Returns the goal attractor.
  // @param[in] y: 'y' state of the GMP.
  // @param[in] z: 'z' state of the GMP.
  // @return: goal attractor.
  double goalAttractor(double y, double z) const;


  // Returns the shape attractor.
  // @param[in] s: phase variable state, see @Phase.
  // @return: shape attractor.
  double shapeAttractor(const gmp_::Phase &s) const;

    
// ====================================
// =======  Public Properties  ========
// ====================================
public:
  double D; ///< damping
  double K; ///<stiffness

  std::shared_ptr<gmp_::WSoG> wsog; ///< WSoG object for encoding the desired position
  
// =======================================
// =======  Protected Properties  ========
// =======================================
protected:
  // output state
  double y_dot; ///< position derivative
  double z_dot; ///< scaled velocity derivative

  double g; ///< target/goal
};

} // namespace gmp_

} // namespace as64_

#endif // AS64_GMP_H_