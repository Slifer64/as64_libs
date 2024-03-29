/** EKF class
 * Implementation of the discrete Extended Kalman Filter algorithm.
 * - The Jacobian of the state transition and measurement functions can either
 * be provided or approximated numerically.
 * - A fading memory coefficient can be defined.
 * - Enforcement of linear constraints can be imposed on the estimated parameteres.
 */

#ifndef DMPoEKFa_EXTENDED_KALMAN_FILTER_AS64_H
#define DMPoEKFa_EXTENDED_KALMAN_FILTER_AS64_H

#include <cmath>
#include <functional>
#include <armadillo>
#include <Eigen/Dense>
#include <dmp_lib/DMP/DMP_eo.h>

namespace as64_
{

namespace dmp_
{

class DMPoEKFa
{
public:
  struct StateTransCookie
  {
    double t;
    StateTransCookie(double t) { this->t = t; }
  };

  /** \brief Extended Kalman Filter constructor.
   * @param[in] theta0 Initial parameters estimate.
   * @param[in] P0 Initial parameters estimate error covariance.
   * @param[in] N_msr Dimensionality of measurement vector.
   * @param[in] stateTransFun_ptr Pointer to state transition function. It should accept a
   *                              vector (the parameters) and  a void pointer (cookie) to pass
   *                              optinally extra arguments to the state transition function.
   * @param[in] msrFun_ptr Pointer to measurement function. It should accept a vector
   *                       (the parameters) and a void pointer (cookie) to pass optinally
   *                       extra arguments to the measurement function.
   */
  DMPoEKFa(std::shared_ptr<dmp_::DMP_eo> dmp, double Ts);


  /** \brief Sets the fading memory coefficient.
   *  \note If the continuous time fading memory coefficient is 'a' then
   *  the discrete one is 'a_p = exp(a*Ts)' where Ts is the sampling period.
   * @param[in] a_p: Fading memory coefficient.
   */
  void setFadingMemoryCoeff(double a_p);

  /** \brief Enables/Disables constraints in the estimation parameters.
   *  \note The constraints must be set with 'setParamsConstraints' first.
   *  @param[in] enable_contraints Flag that is true/false for enabling/disabling the constraints.
   */
  void enableParamsContraints(bool enable_contraints);

  /** \brief Sets linear constraints in the estimation parameters.
   * The constraints are such that A_c*theta <= b_c.
   * @param[in] A_c Constraint matrix.
   * @param[in] b_c Constraints bounds.
   */
  void setParamsConstraints(const arma::mat &A_c, const arma::vec &b_c);

  /** \brief Sets the covariance matrix of the process noise.
   * \note If the continuous time process noise is R_c the discrete
   * one is 'R = Q_c/Ts' where Ts is the sampling period.
   * @param[in] Q Process noise covariance matrix.
   */
  void setProcessNoiseCov(const arma::mat &Q);

  /** \brief Sets the covariance matrix of the measurement noise.
   *  \note that if the continuous time process noise is Q_c the discrete
   *  one is 'Q = Q_c*Ts' where Ts is the sampling period.
   * @param[in] R Measurement noise covariance matrix.
   */
  void setMeasureNoiseCov(const arma::mat &R);

  /** \brief Sets the step for computing the partial derivatives of the state
   * transition and/or measurement function w.r.t. the estimation parameters.
   * @param[in] dtheta Step size for the parameters.
   */
  void setPartDerivStep(double dtheta);

  /** \brief Sets the step for computing the partial derivatives of the state
   * transition and/or measurement function w.r.t. the estimation parameters.
   * @param[in] dtheta Vector of step size for each parameter.
   */
  void setPartDerivStep(const arma::vec &dtheta);

  /** \brief Sets the state transition function Jacobian.
   *  @param[in] stateTransFunJacob_ptr: Pointer to the state transition function Jacobian.
   *                                     It should accept a vector (the parameters) and a void
   *                                     pointer (cookie) to pass extra arguments to the function.
   */
  void setStateTransFunJacob(arma::mat (*stateTransFunJacob_ptr)(const arma::vec &theta, void *cookie))
  {
    this->stateTransFunJacob_ptr = std::bind(stateTransFunJacob_ptr, std::placeholders::_1, std::placeholders::_2);
  }

  /** \brief Sets the state transition function Jacobian.
   *  @param[in] stateTransFunJacob_ptr: Class member function pointer to the state transition function Jacobian.
   *                                     It should accept a vector (the parameters) and a void
   *                                     pointer (cookie) to pass extra arguments to the function.
   * @param[in] obj_ptr Pointer to class object for calling stateTransFunJacob_ptr.
   */
  template<class T>
  void setStateTransFunJacob(arma::mat (T::*stateTransFunJacob_ptr)(const arma::vec &theta, void *cookie), T *obj_ptr)
  {
    this->stateTransFunJacob_ptr = std::bind(stateTransFunJacob_ptr, obj_ptr, std::placeholders::_1, std::placeholders::_2);
  }


  /** \brief Sets the measurement function Jacobian.
   * @param[in] msrFunJacob_ptr: Pointer to the measurement function Jacobian.
   *                               It should accept a vector (the parameters) and a void
   *                               pointer (cookie) to pass extra arguments to the function.
   */
   void setMsrFunJacob(arma::mat (*msrFunJacob_ptr)(const arma::vec &theta, void *cookie))
   {
     this->msrFunJacob_ptr = std::bind(msrFunJacob_ptr, std::placeholders::_1, std::placeholders::_2);
   }

   /** \brief Sets the measurement function Jacobian.
    *  @param[in] msrFunJacob_ptr: Class member function pointer to the measurement function Jacobian.
    *                                     It should accept a vector (the parameters) and a void
    *                                     pointer (cookie) to pass extra arguments to the function.
    * @param[in] obj_ptr Pointer to class object for calling msrFunJacob_ptr.
    */
   template<class T>
   void setMsrFunJacob(arma::mat (T::*msrFunJacob_ptr)(const arma::vec &theta, void *cookie), T *obj_ptr)
   {
     this->msrFunJacob_ptr = std::bind(msrFunJacob_ptr, obj_ptr, std::placeholders::_1, std::placeholders::_2);
   }

  /** \brief Performs the EKF prediction (time update).
   *  @params[in] cookie: Void pointer to additional arguments needed by the state transition function
   *                      and its Jacobian (default = NULL).
   */
  void predict(void *cookie=NULL);

  /** Performs the EKF correction (measurement update).
   *  @param[in] z: Groundtruth measurement.
   *  @params[in] cookie: Void pointer to additional arguments needed by the measurement function
   *                      and its Jacobian (default = NULL).
   */
  void correct(const arma::vec &z, void *cookie=NULL);

  arma::vec theta; ///< parameters estimate
  arma::mat P; ///< parameters estimation error covariance

private:

  arma::vec Az;
  arma::vec Bz;
  std::shared_ptr<dmp_::DMP_eo> dmp;
  double Ts;

  arma::mat::fixed<10,10> F_k; ///< state transition function Jacobian
  arma::mat::fixed<6,10> H_k; ///< measurement function Jacobian
  arma::mat::fixed<10,6> K; ///< Kalman gain
  arma::mat::fixed<10,10> Qn; ///< process noise covariance
  arma::mat::fixed<6,6> Rn; ///< measurement noise covariance

  double a_p; ///< fading memory coefficient

  // Apply projection so that:
  // A_c * theta <= b_c
  bool enable_constraints;
  arma::mat A_c; ///< constraint matrix
  arma::vec b_c; ///< constraints bounds

  bool apply_cov_sat;
  arma::vec theta_sigma_min;
  arma::vec theta_sigma_max;

  arma::vec dtheta; ///< Parameters step size for numerical approximation of transition and measurement function Jacobian

  std::function<arma::vec(const arma::vec &theta, void *cookie)> stateTransFun_ptr; ///< state transition function pointer
  std::function<arma::vec(const arma::vec &theta, void *cookie)> msrFun_ptr; ///< measurement function pointer

  std::function<arma::mat(const arma::vec &theta, void *cookie)> stateTransFunJacob_ptr; ///< state transition function Jacobian pointer
  std::function<arma::mat(const arma::vec &theta, void *cookie)> msrFunJacob_ptr; ///< measurement function Jacobian pointer

  /** \brief Computes numerically the state transition function's Jacobian.
   *  @param[in] theta Parameters around which the Jacobian is computed.
   *  @params[in] cookie Void pointer to additional arguments needed by the state transition
   *                     function's Jacobian. If not needed set to empty [] (null).
   *  @return F_k The state transition function's Jacobian.
   */
  arma::mat calcStateTransFunJacob(const arma::vec &theta, void *cookie=NULL);

  /** \brief Computes numerically the measurement function's Jacobian.
   *  @param[in] theta Parameters around which the Jacobian is computed.
   *  @params[in] cookie Void pointer to additional arguments needed by the measurement
   *                      function's Jacobian. If not needed set to empty [] (null).
   *  @return H_k The measurement function's Jacobian.
   */
  arma::mat calcMsrFunJacob(const arma::vec &theta, void *cookie=NULL);

  arma::vec stateTransFun(const arma::vec &theta, void *cookie);
  arma::vec msrFun(const arma::vec &theta, void *cookie=0);
  arma::vec rotVel2deo(const arma::vec &rotVel, const arma::vec &Q);

  arma::mat msrFunJacob(const arma::vec &theta, void *cookie=0);
  arma::mat stateTransFunJacob(const arma::vec &theta, void *cookie);
  arma::mat calcF1Jacob(const arma::vec &theta, double t);
  arma::mat calcF2Jacob(const arma::vec &theta);

  // Eigen::Map<Eigen::Matrix<double,10,6>> K_map;
  // Eigen::Map<Eigen::Matrix<double,10,10>> P_map;
};

} // namespace dmp_

} // namespace as64_

#endif // DMPoEKFa_EXTENDED_KALMAN_FILTER_AS64_H
