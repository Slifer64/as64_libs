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

  /* GMP constructor.
   * @param[in] n: number of degrees of freedom.
   * @param[in] N_kernels: the number of kernels
   * @param[in] D: damping.
   * @param[in] K: stiffness.
   * @param[in] kernels_std_scaling: Scaling for std of kernels (optional, default=1).
   * \note: Each of the arguments 'N_kernels', 'D', 'K' can be scalar or a nx1 vector.
   */
  GMP_nDoF(unsigned n, arma::uvec N_kernels, arma::vec D, arma::vec K, double kernels_std_scaling=1.0);


  /* Returns the number of DoFs.
   * return: number of DoFs.
   */
  unsigned length() const;

  void setStiffness(const arma::vec &K) { for (int i=0; i<length(); i++) gmp[i]->setStiffness(K(i)); }

  void setDamping(const arma::vec &D) { for (int i=0; i<length(); i++) gmp[i]->setDamping(D(i)); }

  /* Trains the GMP.
   * @param[in] train_method: the training method to use, as a string ('LWR', 'LS').
   * @param[in] Time: Row vector with the timestamps of the training data points.
   * @param[in] yd_data: Matrix with the desired potition for each DoF in each row.
   * @param[out] train_error: The training error expressed as the mse error.
   */
  virtual void train(const std::string &train_method, const arma::rowvec &Time, const arma::mat &yd_data, arma::vec *train_error=0);


  /* Calculates the time derivatives of the GMP's states.
   * @param[in] s: Vector with the phase variable state, i.e. s = [x; x_dot; x_ddot].
   * @param[in] y: 'y' state of the GMP.
   * @param[in] z: 'z' state of the GMP.
   * @param[in] y_c: coupling term for the dynamical equation of the 'y' state (optional, default=0).
   * @param[in] z_c: coupling term for the dynamical equation of the 'z' state (optional, default=0).
   */
  void update(const gmp_::Phase &s, const arma::vec &y, const arma::vec &z, arma::vec y_c=arma::vec({0}), arma::vec z_c=arma::vec({0}));


  /* Returns the 'y' state time derivative.
   * Call after @update.
   * @return: time derivative of 'y' state.
   */
  arma::vec getYdot() const;


  /* Returns the 'z' state time derivative.
   * Call after @update.
   * @return: time derivative of 'z' state.
   */
  arma::vec getZdot() const;


  /* Returns the GMP's acceleration.
   * Call after @update.
   * @param[in] yc_dot: time derivative of 'y' state coupling (optional, default=0).
   * @return: acceleration.
   */
  arma::vec getYddot(arma::vec yc_dot=arma::vec({0})) const;


  /* Calculates the GMP's acceleration.
   * @param[in] s: phase variable state, see @Phase.
   * @param[in] y: 'y' state of the GMP.
   * @param[in] y_dot: time derivative of 'y' state.
   * @param[in] y_c: coupling term for the dynamical equation of the 'y' state (optional, default=0).
   * @param[in] z_c: coupling term for the dynamical equation of the 'z' state (optional, default=0).
   * @param[in] yc_dot: time derivative of the 'y' state coupling (optional, default=0).
   * @return: acceleration.
   */
  arma::vec calcYddot(const gmp_::Phase &s, const arma::vec &y, const arma::vec &y_dot, arma::vec yc=arma::vec({0}), arma::vec zc=arma::vec({0}), arma::vec yc_dot=arma::vec({0})) const;


  /* Returns the number of kernels for the i-th gmp.
   * @param[in] i: index for the i-th DoF.
   * @return: number of kernels for the i-th gmp.
   */
  unsigned numOfKernels(int i) const;


  /* Sets the initial position.
   * @param[in] y0: initial position.
   */
  void setY0(const arma::vec &y0);


  /* Set goal position.
   * @param[in] g: goal position.
   */
  void setGoal(const arma::vec &g);


  // TODO
  // Creates a deep copy of this object
  // cp_obj = deepCopy()

  arma::vec getYd0() const { return getYd(0); }
  arma::vec getYdf() const { return getYd(1); }

  /* Returns the scaled desired position.
   * @param[in] x: phase variable.
   * @return: scaled desired position.
   */
  arma::vec getYd(double x) const;


  /* Returns the scaled desired velocity.
   * @param[in] x: phase variable.
   * @param[in] x_dot: 1st time derivative of the phase variable.
   * @return: scaled desired velocity.
   */
  arma::vec getYdDot(double x, double x_dot) const;


  /* Returns the scaled desired acceleration.
   * @param[in] x: phase variable.
   * @param[in] x_dot: 1st time derivative of the phase variable.
   * @param[in] x_ddot: 2nd time derivative of the phase variable.
   * @return: scaled desired acceleration.
   */
  arma::vec getYdDDot(double x, double x_dot, double x_ddot) const;


  /* Export the GMP model to a file.
   * @param[in] filename: The name of the file.
   */
  void exportToFile(const std::string &filename) const;

  /* Import a GMP model from a file.
   * @param[in] filename: The name of the file.
   */
  static std::shared_ptr<GMP_nDoF> importFromFile(const std::string &filename);

  /* Write the GMP model to a file.
   * @param[in] fid: Object of type @FileIO associated with the file.
   * @param[in] prefix: The prefix that will be used for writing the names of all GMP params (optional, default="").
   */
  void writeToFile(FileIO &fid, const std::string &prefix="") const;

  /* Reads the GMP model from a file.
   * @param[in] fid: Object of type @FileIO associated with the file.
   * @param[in] prefix: The prefix that will be used for reading the names of all GMP params (optional, default="").
   */
  void readFromFile(FileIO &fid, const std::string &prefix="");

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
  arma::vec y_dot; ///< position derivative
  arma::vec z_dot; ///< scaled velocity derivative

};

} // namespace gmp_

} // namespace as64_

#endif // AS64_GMP_N_DOF_H_