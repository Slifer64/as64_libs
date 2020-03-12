#ifndef AS64_GMP_H_
#define AS64_GMP_H_

// GMP class
// Generalized movement primitive.
//

#include <gmp_lib/WSoG/WSoG.h>
#include <gmp_lib/utils.h>
#include <gmp_lib/GatingFunction/SigmoidGatingFunction.h>

namespace as64_
{

namespace gmp_
{
  
class GMP
{

public:

// ===================================
// =======  Public Functions  ========
// ===================================
public:
  // DMP constructor.
  // @param[in] N_kernels: the number of kernels
  // @param[in] D: damping.
  // @param[in] K: stiffness.
  // @param[in] kernels_std_scaling: Scaling for std of kernels (optional, default=2).
  GMP(unsigned N_kernels, double D, double K, double kernels_std_scaling=2.0);


  // Trains the DMP.
  // @param[in] Time: Row vector with the timestamps of the training data points.
  // @param[in] yd_data: Row vector with the desired potition.
  void train(const std::string &train_method, const arma::rowvec &Time, const arma::rowvec &yd_data, double *train_error=0);


  // TODO
  // Constrained optimization.
  // @param[in] T: Time duration of the motion.
  // @param[in] pos_constr: Vector of @GMPConstr position constraints. For no constraints pass '[]'.
  // @param[in] vel_constr: Vector of @GMPConstr velocity constraints. For no constraints pass '[]'.
  // @param[in] accel_constr: Vector of @GMPConstr acceleration constraints. For no constraints pass '[]'.
  // @param[in] opt_set: Object of type @GMPOptSet for setting optimization options.
  // void constrOpt(double T, pos_constr, vel_constr, accel_constr, opt_set)
            
        
  // Updates the weights so that the generated trajectory passes from the given points.
  // @param[in] s: Matrix of phase variable states in each column, i.e. s = [x;x_dot; x_ddot].
  //               If s = [x; x_dot], x_ddot is assumed to be 0.
  // @param[in] z: Row vector with the desired value for each timestamp.
  // @param[in] type: Row vector with the type of each point (GMP_UPDATE_TYPE).
  // @param[in] z_var: Row vector with the variance of each point (optional, default = 1e-3).
  void updateWeights(const std::vector<gmp_::Phase> &s, const arma::rowvec &z, const std::vector<gmp_::UPDATE_TYPE> &type, const arma::rowvec &z_var_=arma::rowvec({1e-3}) );
        
        
  // Returns the derivatives of the DMP states.
  // @param[in] x: phase variable.
  // @param[in] y: \a y state of the this->
  // @param[in] z: \a z state of the this->
  // @param[in] y_c: coupling term for the dynamical equation of the \a y state.
  // @param[in] z_c: coupling term for the dynamical equation of the \a z state.
  void update(const gmp_::Phase &s, double y, double z, double y_c=0, double z_c=0);


  double getYdot() const;

  double getZdot() const;


  // Returns the DMP's acceleration.
  double getYddot(double yc_dot = 0) const;


  // Calculates the DMP's acceleration.
  double calcYddot(const gmp_::Phase &s, double y, double y_dot, double yc=0, double zc=0, double yc_dot=0);


  // Returns the number of kernels.
  int numOfKernels() const;


  // Sets the initial position.
  void setY0(double y0);


  // Set goal position.
  void setGoal(double g);


  // TODO
  // Creates a deep copy of this object
  //std::shared_ptr<GMP> deepCopy() const


  double getYd(double x) const;


  double getYdDot(double x, double x_dot) const;


  double getYdDDot(double x, double x_dot, double x_ddot) const;


// ====================================
// =======  Protected Methods  ========
// ====================================

  // Returns the goal attractor of the this->
  // @param[in] y: \a y state of the this->
  // @param[in] z: \a z state of the this->
  // @param[in] g: Goal position.
  // @param[out] goal_attr: The goal attractor of the this->
  double goalAttractor(double y, double z) const;


  // Returns the shape attractor
  double shapeAttractor(const gmp_::Phase &s) const;


  // Returns the forcing term
  double forcingTerm(const gmp_::Phase &s) const;

    
// ====================================
// =======  Public Properties  ========
// ====================================
public:
  double D; ///< damping
  double K; ///<stiffness

  std::shared_ptr<gmp_::GatingFunction> shape_attr_gating_ptr; ///< pointer to gating for the shape attractor
  std::shared_ptr<gmp_::WSoG> wsog; ///< WSoG object
  
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