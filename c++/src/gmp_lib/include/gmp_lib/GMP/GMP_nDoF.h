#ifndef AS64_GMP_N_DOF_H_
#define AS64_GMP_N_DOF_H_

// N-DoF GMP class
// Generalized movement primitive.

#include <gmp_lib/GMP/GMP.h>
#include <gmp_lib/utils.h>

namespace as64_
{

namespace gmp_
{


class GMP_nDoF
{
// ===================================
// =======  Public Functions  ========
// ===================================
public:
// DMP constructor.
// @param[in] n: number of degrees of freedom.
// @param[in] N_kernels: the number of kernels
// @param[in] D: damping.
// @param[in] K: stiffness.
// @param[in] kernels_std_scaling: Scaling for std of kernels (optional, default=2).
GMP_nDoF(unsigned n, arma::uvec N_kernels, arma::vec D, arma::vec K, double kernels_std_scaling=2.0);


// Returns the number of DoFs.
unsigned length() const;


// Trains the DMP.
// @param[in] Time: Row vector with the timestamps of the training data points.
// @param[in] yd_data: Row vector with the desired potition.
void train(const std::string &train_method, const arma::rowvec &Time, const arma::mat &yd_data, arma::vec *train_error=0);


// Returns the derivatives of the DMP states.
// @param[in] x: phase variable.
// @param[in] y: \a y state of the this->
// @param[in] z: \a z state of the this->
// @param[in] y_c: coupling term for the dynamical equation of the \a y state.
// @param[in] z_c: coupling term for the dynamical equation of the \a z state.
void update(const gmp_::Phase &s, const arma::vec &y, const arma::vec &z, arma::vec y_c=arma::vec({0}), arma::vec z_c=arma::vec({0}));

        
arma::vec getYdot() const;

arma::vec getZdot() const;

// Returns the DMP's acceleration.
arma::vec getYddot(arma::vec yc_dot=arma::vec({0})) const;
        
        
// Calculates the DMP's acceleration.
arma::vec calcYddot(const gmp_::Phase &s, const arma::vec &y, const arma::vec &y_dot, arma::vec yc=arma::vec({0}), arma::vec zc=arma::vec({0}), arma::vec yc_dot=arma::vec({0})) const;
        
        
// Returns the number of kernels for the i-th gmp.
unsigned numOfKernels(int i) const;


// Sets the initial position.
void setY0(const arma::vec &y0);


// Set goal position.
void setGoal(const arma::vec &g);

// TODO
// Creates a deep copy of this object
// cp_obj = deepCopy()

        
arma::vec getYd(double x) const;


arma::vec getYdDot(double x, double x_dot) const;


arma::vec getYdDDot(double x, double x_dot, double x_ddot) const;
        

// =======================================
// =======  Protected Properties  ========
// =======================================
public:
  std::vector<std::shared_ptr<gmp_::GMP>> gmp;

// =======================================
// =======  Protected Properties  ========
// =======================================
protected:
  // output state
  arma::vec y_dot; //position derivative
  arma::vec z_dot; //scaled velocity derivative

};

} // namespace gmp_

} // namespace as64_

#endif // AS64_GMP_N_DOF_H_