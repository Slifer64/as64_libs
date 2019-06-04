#ifndef AS64_QUAT_ERROR_DYNAMICAL_MOVEMENT_PRIMITIVE_H
#define AS64_QUAT_ERROR_DYNAMICAL_MOVEMENT_PRIMITIVE_H

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

class DMP_eo
{

// ====================================================
// ****************************************************
// ************      Public Functions      ************
// ****************************************************
// ====================================================
public:

  /** \brief DMP constructor.
   * @param[in] N_kernels The number of kernels.
   * @param[in] a_z Parameter 'a_z' relating to the spring-damper system.
   * @param[in] b_z Parameter 'b_z' relating to the spring-damper system.
   * @param[in] can_clock_ptr Pointer to a DMP canonical system object. (optinal, default=CanonicalClock>())
   * @param[in] shape_attr_gating_ptr Pointer to gating function for the shape attractor. (optinal, default=SigmoidGatingFunction(1.0, 0.5))
   */
  DMP_eo(dmp_::TYPE dmp_type, const arma::uvec &N_kernels, const arma::vec &a_z, const arma::vec &b_z,
      std::shared_ptr<CanonicalClock> can_clock_ptr=std::shared_ptr<CanonicalClock>(),
      std::shared_ptr<GatingFunction> shape_attr_gating_ptr=std::shared_ptr<GatingFunction>());

  /** \brief Trains the DMP weights of the kernels.
   *  @param[in] train_method Method to train the DMP weights ('LWR' or 'LS').
   *  @param[in] Time Row vector with the timestamps of the training data points.
   *  @param[in] yd_data Row vector with the desired potition.
   *  @param[in] dyd_data Row vector with the desired velocity.
   *  @param[in] ddyd_data Row vector with the desired accelaration.
   *  @param[in] ret_train_err Flag to return the training error (optinal, default = false).
   *  @return The training error (-1 is returned if \a ret_train_err=false)
   */
  void train(dmp_::TrainMethod train_method, const arma::rowvec &Time, const arma::mat &Quat_data,
                     const arma::mat &rotVel_data, const arma::mat &rotAccel_data, arma::vec *train_err=0);


  void setQ0(const arma::vec &Q0);


  void setQg(const arma::vec &Qg);


    /** \brief Returns the derivatives of the DMP states.
   *  @param[in] x phase variable.
   *  @param[in] y \a y state of the DMP.
   *  @param[in] z \a z state of the DMP.
   *  @param[in] g Goal position.
   *  @param[in] y_c Coupling term for the dynamical equation of the \a y state.
   *  @param[in] z_c Coupling term for the dynamical equation of the \a z state.
   *  @return  The states derivatives of the DMP as a 3x1 vector (dz, dy, dx).
   */
  void update(double x, const arma::vec &Y, const arma::vec &Z, const arma::vec &Yc=arma::vec().zeros(3), const arma::vec &Zc=arma::vec().zeros(3));

  arma::vec getYddot(double tau_dot=0, const arma::vec &Yc_dot=arma::vec().zeros(3)) const;

  double getXdot() const { return this->dx; }
  arma::vec getYdot() const { return this->dY; }
  arma::vec getZdot() const { return this->dZ; }

  arma::vec getRotVel(const arma::vec &Q) const;

  arma::vec getRotAccel(const arma::vec &Q, double tau_dot=0, const arma::vec &Yc_dot=arma::vec().zeros(3)) const;

  arma::vec calcRotAccel(double x, const arma::vec &Q, const arma::vec &rotVel, const arma::vec &Qg,
      double tau_dot=0, const arma::vec &Yc=arma::vec().zeros(3), const arma::vec &Zc=arma::vec().zeros(3), const arma::vec &Yc_dot=arma::vec().zeros(3)) const;

  arma::vec getY(const arma::vec &Q) const;

  arma::vec getZ(const arma::vec &rotVel, const arma::vec &Q) const;


  /** \brief Sets the time scale of the DMP.
   *  @param[in] tau The time duration for the DMP.
   */
  void setTau(double tau);


  /** \brief Returns the time scale of the DMP.
   *  @return The time scale of the DMP.
   */
  double getTau() const;


  /** \brief Returns the phase variable.
   *  @param[in] t The time instant.
   *  @return The phase variable for time 't'.
   */
  double phase(double t) const;


  /** \brief Returns the derivative of the phase variable.
   *  @param[in] x The phase variable.
   *  @return The derivative of the phase variable.
   */
  double phaseDot(double x) const;


  void exportToFile(std::ostream &out) const;

  static std::shared_ptr<DMP_eo> importFromFile(std::istream &in);

// ===========================================================
// ***********************************************************
// ************      Static Public Functions      ************
// ***********************************************************
// ===========================================================
public:

  /** \brief TODO doc.*/
  static arma::vec quatError(const arma::vec &Q, const arma::vec &Qg);


  /** \brief TODO doc.*/
  static arma::vec quat2eo(const arma::vec &Q, const arma::vec &Qg);


  /** \brief TODO doc.*/
  static arma::vec eo2quat(const arma::vec &eo, const arma::vec &Qg);


  /** \brief TODO doc.*/
  static arma::vec rotVel2deo(const arma::vec &rotVel, const arma::vec &Qe);


  /** \brief TODO doc.*/
  static arma::vec deo2rotVel(const arma::vec &deo, const arma::vec &Qe);


  /** \brief TODO doc.*/
  static arma::vec rotAccel2ddeo(const arma::vec &rotAccel, const arma::vec &rotVel, const arma::vec &Qe);


  /** \brief TODO doc.*/
  static arma::vec ddeo2rotAccel(const arma::vec &ddeo, const arma::vec &rotVel, const arma::vec &Qe);


  /** \brief TODO doc.*/
  static arma::mat jacobDquatDeo(const arma::vec &Qe);


  /** \brief TODO doc.*/
  static arma::mat jacobDeoDquat(const arma::vec &Qe);


  /** \brief TODO doc.*/
  static arma::mat jacobDotDeoDquat(const arma::vec &Qe, const arma::vec &rotVel);


  /** \brief TODO doc.*/
  static arma::mat jacobDotDquatDeo(const arma::vec &Qe, const arma::vec &rotVel);

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

  arma::vec Q0;
  arma::vec Qg;

  double dx; ///< phase variable time derivative
  arma::vec dY; ///< state 'Y' time derivative
  arma::vec dZ; ///< state 'Z' time derivative

}; // class DMP

} // namespace dmp_

} // namespace as64_

#endif // AS64_QUAT_ERROR_DYNAMICAL_MOVEMENT_PRIMITIVE_H
