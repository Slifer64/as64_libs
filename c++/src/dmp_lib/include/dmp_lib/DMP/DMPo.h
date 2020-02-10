#ifndef AS64_DMPo_MOVEMENT_PRIMITIVE_H
#define AS64_DMPo_MOVEMENT_PRIMITIVE_H

#include <cmath>
#include <vector>
#include <cstring>
#include <memory>
#include <exception>
#include <armadillo>

#include <dmp_lib/DMP/DMP_.h>
#include <dmp_lib/utils.h>
#include <dmp_lib/CanonicalClock/CanonicalClock.h>
#include <dmp_lib/GatingFunction/GatingFunction.h>

namespace as64_
{

namespace dmp_
{

class DMPo
{

// ====================================================
// ****************************************************
// ************      Public Functions      ************
// ****************************************************
// ====================================================
public:

  /** \brief DMP constructor.
   * @param[in] N_kernels: The number of kernels.
   * @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
   * @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
   * @param[in] can_clock_ptr: Pointer to a DMP canonical system object. (optinal, default=CanonicalClock>())
   * @param[in] shape_attr_gating_ptr: Pointer to gating function for the shape attractor. (optinal, default=SigmoidGatingFunction(1.0, 0.5))
   */
  DMPo(dmp_::TYPE dmp_type, const arma::uvec &N_kernels, const arma::vec &a_z, const arma::vec &b_z,
      std::shared_ptr<CanonicalClock> can_clock_ptr=std::shared_ptr<CanonicalClock>(),
      std::shared_ptr<GatingFunction> shape_attr_gating_ptr=std::shared_ptr<GatingFunction>());

  /** \brief Trains the DMP weights of the kernels.
   *  @param[in] train_method: Method to train the DMP weights ('LWR' or 'LS').
   *  @param[in] Time: Row vector with the timestamps of the training data points.
   *  @param[in] yd_data: Row vector with the desired potition.
   *  @param[in] dyd_data: Row vector with the desired velocity.
   *  @param[in] ddyd_data: Row vector with the desired accelaration.
   *  @param[in] ret_train_err: Flag to return the training error (optinal, default = false).
   *  @return The training error (-1 is returned if \a ret_train_err=false)
   */
  void train(dmp_::TrainMethod train_method, const arma::rowvec &Time, const arma::mat &Quat_data,
                     const arma::mat &rotVel_data, const arma::mat &rotAccel_data, arma::vec *train_err=0);

    /** \brief Sets the initial orientation.
     *  @param[in] Q0: Initial orientation (as unit quaternion).
     */
   void setQ0(const arma::vec &Q0);

  /** \brief Calculates the derivatives of the DMP states.
   *  The derivatives can then be retrieved with 'getXdot', 'getYdot' and 'getZdot'.
   *  @param[in] x: phase variable.
   *  @param[in] Y: 'y' state of the DMP: y=log(Q*Q0^{-1}).
   *  @param[in] Z: 'z' state of the DMP (the scaled ydot state).
   *  @param[in] G: 'g' goal/target of the DMP: y=log(Qg*Q0^{-1}).
   *  @param[in] Yc: Coupling term for the deonamical equation of the 'y' state.
   *  @param[in] Zc: Coupling term for the deonamical equation of the 'z' state.
   */
  void update(double x, const arma::vec &Y, const arma::vec &Z, const arma::vec &G, const arma::vec &Yc=arma::vec().zeros(3), const arma::vec &Zc=arma::vec().zeros(3));

  /** \brief Returns the time derivative of the DMP's phase variable.
   *  Call @update first!
   *  @return: time derivative of the phase variable.
   */
  double getXdot() const { return this->dx; }

  /** \brief Returns the time derivative of the DMP's 'y' state.
   *  Call @update first!
   *  @return: time derivative of 'y' state.
   */
  arma::vec getYdot() const { return this->dY; }

  /** \brief Returns the time derivative of the DMP's 'z' state.
   *  Call @update first!
   *  @return: time derivative of 'z' state.
   */
  arma::vec getZdot() const { return this->dZ; }

  /** \brief Returns the second derivative of the DMP's 'y' state.
   *  Call @update first!
   *  @param[in] tau_dot: time derivative of time scaling (optional, default=0).
   *  @param[in] yc_dot: time derivative of 'yc' coupling term (optional, default=0).
   *  @return: second time derivative of 'y' state.
   */
  arma::vec getYddot(double tau_dot=0, const arma::vec &Yc_dot=arma::vec().zeros(3)) const;

  /** \brief Returns the rotational velocity.
   *  Call @update first!
   *  @param[in] Q: the current orientation.
   *  @return: the rotational velocity.
   */
  arma::vec getRotVel(const arma::vec &Q) const;

  /** \brief Returns the rotational acceleration.
   *  Call @update first!
   *  @param[in] Q: the current orientation.
   *  @param[in] tau_dot: time derivative of time scaling (optional, default=0).
   *  @param[in] yc_dot: time derivative of 'yc' coupling term (optional, default=0).
   *  @return: the rotational acceleration.
   */
  arma::vec getRotAccel(const arma::vec &Q, double tau_dot=0, const arma::vec &Yc_dot=arma::vec().zeros(3)) const;

  /** \brief Calculates the rotational acceleration based on the current input variables.
   *  @param[in] x: phase variable.
   *  @param[in] Q: the current orientation.
   *  @param[in] rotVel: the rotational velocity.
   *  @param[in] Qg: the target orientation.
   *  @param[in] tau_dot: time derivative of time scaling (optional, default=0).
   *  @param[in] yc: Coupling term fo 'y' state diff-equation (optional, default=0).
   *  @param[in] zc: Coupling term fo 'z' state diff-equation (optional, default=0).
   *  @param[in] yc_dot: time derivative of 'yc' coupling term (optional, default=0).
   *  @return: the rotational acceleration.
   */
  arma::vec calcRotAccel(double x, const arma::vec &Q, const arma::vec &rotVel, const arma::vec &Qg,
      double tau_dot=0, const arma::vec &Yc=arma::vec().zeros(3), const arma::vec &Zc=arma::vec().zeros(3), const arma::vec &Yc_dot=arma::vec().zeros(3)) const;

  /** \brief Returns the 'y' state of the DMP based on the current orientation.
   *  @param[in] Q: Current orientation (as unit quaternion).
   */
  arma::vec getY(const arma::vec &Q) const;

  /** \brief Returns the 'z' state of the DMP based on the current rotational velocity and orientation.
   *  @param[in] rotVel: Current rotational velocity.
   *  @param[in] Q: Current orientation (as unit quaternion).
   */
  arma::vec getZ(const arma::vec &rotVel, const arma::vec &Q) const;

  /** \brief Returns the time scaling factor.
   *  @return: The time scaling factor.
   */
  double getTau() const;

  /** \brief Sets the time scaling factor.
   *  @param[in] tau: The time scaling factor.
   */
  void setTau(double tau);

  /** \brief Returns the phase variable.
   *  @param[in] t: The time instant.
   *  @return The phase variable for time 't'.
   */
  double phase(double t) const;

  /** \brief Returns the derivative of the phase variable.
   *  @param[in] x: The phase variable.
   *  @return The derivative of the phase variable.
   */
  double phaseDot(double x) const;

  /** \brief Saves the DMP model in binary format.
   *  @param[in] out: The output stream where the DMP model is written.
   */
  void exportToFile(std::ostream &out) const;

  static std::shared_ptr<DMPo> importFromFile(std::istream &in);

// ===========================================================
// ***********************************************************
// ************      Static Public Functions      ************
// ***********************************************************
// ===========================================================
public:

  /** \brief Expresses a given quaternion w.r.t. the initial orientation.
   *  @param[in] Q: Orientation as unit quaternion.
   *  @param[in] Q0: Initial quaternion.
   *  @return: Q1 = Orientation w.r.t. Q0, i.e. Q1 = Q*Q0^{-1}.
   */
  static arma::vec quatTf(const arma::vec &Q, const arma::vec &Q0);


  /** \brief Returns the log of a given orientation w.r.t. the initial orientation.
   *  @param[in] Q: Orientation as unit quaternion.
   *  @param[in] Q0: Initial quaternion.
   *  @return: The logarithm of the Q w.r.t. Q0, i.e. q = log(Q*Q0^{-1}).
   */
  static arma::vec quat2q(const arma::vec &Q, const arma::vec &Q0);


  /** \brief Returns the quaternion Q given the initial orientation Q0 and the log of Q w.r.t. to Q0.
   *  @param[in] q: Logarithm of orientation w.r.t. the initial orientation.
   *  @param[in] Q0: Initial orientation.
   *  @return: The orientation corresponding to log, i.e. Q = exp(q)*Q0
   */
  static arma::vec q2quat(const arma::vec &eo, const arma::vec &Q0);


  /** \brief Returns derivative of log given the rotational velocity and orientation (expressed w.r.t. the initial orientation)
   *  @param[in] rotVel: Rotational velocity.
   *  @param[in] Q1: Orientation expressed w.r.t. the initial orientation.
   *  @return: Derivative of log.
   */
  static arma::vec rotVel2qdot(const arma::vec &rotVel, const arma::vec &Qe);


  /** \brief TODO doc.*/
  static arma::vec qdot2rotVel(const arma::vec &deo, const arma::vec &Qe);


  /** \brief TODO doc.*/
  static arma::vec rotAccel2qddot(const arma::vec &rotAccel, const arma::vec &rotVel, const arma::vec &Qe);


  /** \brief TODO doc.*/
  static arma::vec qddot2rotAccel(const arma::vec &ddeo, const arma::vec &rotVel, const arma::vec &Qe);


  /** \brief Returns the Jacobian from the derivative of log to the derivative of Q.
   *  @param[in] Q1: The orientation w.r.t. the initial orientation.
   *  @return: Jacobian.
   */
  static arma::mat jacobQq(const arma::vec &Qe);


  /** \brief Returns the Jacobian from the derivative of Q to the derivative of log.
   *  @param[in] Q1: The orientation w.r.t. the initial orientation.
   *  @return: Jacobian.
   */
  static arma::mat jacobqQ(const arma::vec &Qe);


  /** \brief Returns the time derivative of the Jacobian from the derivative of log to the derivative of Q.
   *  @param[in] Q1: The orientation w.r.t. the initial orientation.
   *  @return: Jacobian time derivative.
   */
  static arma::mat jacobDotqQ(const arma::vec &Qe, const arma::vec &rotVel);


  /** \brief Returns the time derivative of the Jacobian from the derivative of Q to the derivative of log.
   *  @param[in] Q1: The orientation w.r.t. the initial orientation.
   *  @return: Jacobian time derivative.
   */
  static arma::mat jacobDotQq(const arma::vec &Qe, const arma::vec &rotVel);

// =======================================================
// *******************************************************
// ************      Protected Functions      ************
// *******************************************************
// =======================================================
protected:



// ======================================================
// ******************************************************
// ************      Public Properties      *************
// ******************************************************
// ======================================================
public:

  std::vector<std::shared_ptr<DMP_>> dmp;

  std::shared_ptr<CanonicalClock> can_clock_ptr; ///< pointer to the canonical clock
  std::shared_ptr<GatingFunction> shape_attr_gating_ptr; ///< pointer to gating function for the shape attractor


// ======================================================
// ******************************************************
// ************    Protected Properties     *************
// ******************************************************
// ======================================================
protected:

  static const long double zero_tol;

  arma::vec Q0; ///< Initial orientation

  double dx; ///< phase variable time derivative
  arma::vec dY; ///< state 'Y' time derivative
  arma::vec dZ; ///< state 'Z' time derivative

}; // class DMP

} // namespace dmp_

} // namespace as64_

#endif // AS64_DMPo_MOVEMENT_PRIMITIVE_H
