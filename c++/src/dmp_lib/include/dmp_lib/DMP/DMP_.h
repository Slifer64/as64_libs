#ifndef AS64_ABSTRACT_DYNAMICAL_MOVEMENT_PRIMITIVE_H
#define AS64_ABSTRACT_DYNAMICAL_MOVEMENT_PRIMITIVE_H

#include <cmath>
#include <vector>
#include <cstring>
#include <memory>
#include <exception>
#include <fstream>
#include <armadillo>

#include <dmp_lib/utils.h>
#include <dmp_lib/CanonicalClock/CanonicalClock.h>
#include <dmp_lib/GatingFunction/GatingFunction.h>

namespace as64_
{

namespace dmp_
{

class DMP; // forward declaration
class DMP_bio; // forward declaration
class DMP_pos; // forward declaration
class DMP_eo; // forward declaration

class DMP_
{
  friend class DMP;
  friend class DMP_bio;
  friend class DMP_pos;
  friend class DMP_eo;
  friend class DMPo;

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
  DMP_(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> can_clock_ptr, std::shared_ptr<GatingFunction> shape_attr_gating_ptr);


  /** \brief Trains the DMP weights of the kernels.
   *  @param[in] train_method Method to train the DMP weights ('LWR' or 'LS').
   *  @param[in] Time Row vector with the timestamps of the training data points.
   *  @param[in] yd_data Row vector with the desired potition.
   *  @param[in] dyd_data Row vector with the desired velocity.
   *  @param[in] ddyd_data Row vector with the desired accelaration.
   *  @param[in] ret_train_err Flag to return the training error (optinal, default = false).
   *  @return The training error (-1 is returned if \a ret_train_err=false)
   */
  void train(dmp_::TrainMethod train_method, const arma::rowvec &Time, const arma::rowvec &yd_data,
               const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double *train_err=0);


  /** \brief Returns the derivatives of the DMP states.
   *  @param[in] x phase variable.
   *  @param[in] y \a y state of the DMP.
   *  @param[in] z \a z state of the DMP.
   *  @param[in] g Goal position.
   *  @param[in] y_c Coupling term for the dynamical equation of the \a y state.
   *  @param[in] z_c Coupling term for the dynamical equation of the \a z state.
   *  @return  The states derivatives of the DMP as a 3x1 vector (dz, dy, dx).
   */
  void update(double x, double y, double z, double g, double y_c = 0.0, double z_c = 0.0);


  /** \brief Returns the DMP's phase variable derivative. @update must be called first.*/
  double getXdot() const { return this->dx; }

  
  /** \brief Returns the DMP's phase variable derivative. @update must be called first.*/
  double getYdot() const { return this->dy; }


  /** \brief Returns the DMP's phase variable derivative. @update must be called first.*/
  double getZdot() const { return this->dz; }


  /** \brief Returns the DMP's phase variable derivative. @update must be called first.*/
  double getYddot(double tau_dot=0, double yc_dot=0) const;


  /** \brief Calculates and returns the DMP's acceleration.
   * @param[in] y Position.
   * @param[in] dy Velocity.
   * @param[in] y0 Initial position.
   * @param[in] y_c Coupling term for the dynamical equation of the \a y state.
   * @param[in] z_c Coupling term for the dynamical equation of the \a z state.
   * @param[in] x_hat Phase variable estimate.
   * @param[in] g_hat Goal estimate.
   * @param[in] tau_hat Time scale estimate.
   * @return ddy DMP's acceleration.
   */
  double calcYddot(double x, double y, double dy, double g, double tau_dot=0, double yc=0, double zc=0, double yc_dot=0) const;


  /** \brief Sets the DMP's initial position.
   *  @param[in] y0 The initial position.
   */
  void setY0(double y0);


  /** \brief Sets the time scale of the DMP.
   *  @param[in] tau The time duration for the DMP.
   */
  void setTau(double tau);


  /** \brief Returns the time scale of the DMP.
   *  @return The time scale of the DMP.
   */
  double getTau() const;


  /** \brief Returns the number of kernels of the DMP.
   * @return Number of kernels of the DMP.
   *
   */
  int numOfKernels() const;


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


  /** \brief Returns the partial derivative of the DMP's acceleration wrt to the goal and tau.
   *  @param[in] t Current timestamp.
   *  @param[in] y Position.
   *  @param[in] dy Velocity.
   *  @param[in] y0 Initial position.
   *  @param[in] x_hat Phase variable estimate.
   *  @param[in] g_hat Goal estimate.
   *  @param[in] tau_hat Time scale estimate.
   *  @return Partial derivative of the DMP's acceleration wrt to the goal and tau.
   */
  arma::vec getAcellPartDev_g_tau(double t, double y, double dy, double y0,
                                  double x, double g, double tau) const;


  virtual void exportToFile(std::ostream &out) const;

  static std::shared_ptr<DMP_> importFromFile(std::istream &in);

// =======================================================
// *******************************************************
// ************      Protected Functions      ************
// *******************************************************
// =======================================================
protected:

  /** \brief Returns the goal attractor of the DMP.
   *  @param[in] x The phase variable.
   *  @param[in] y 'y' state of the DMP.
   *  @param[in] z 'z' state of the DMP.
   *  @param[in] g Goal position.
   *  @return The goal attractor of the DMP.
   */
  double goalAttractor(double x, double y, double z, double g) const;


  /** \brief Returns the goal attractor gating factor.
   *  @param[in] x The phase variable.
   *  @return The goal attractor gating factor.
   */
  double goalAttrGating(double x) const;


  /** \brief Returns the shape attractor gating factor.
   *  @param[in] x The phase variable.
   *  @return The shape attractor gating factor.
   */
  double shapeAttrGating(double x) const;


  /** \brief Returns the forcing term of the DMP
   * @param[in] x The phase variable.
   * @return The normalized weighted sum of Gaussians.
   */
  double forcingTerm(double x) const;


  /** \brief Returns a column vector with the values of the kernel functions of the DMP.
   *  @param[in] x Phase variable.
   *  @return Column vector with the values of the kernel functions of the DMP.
   */
  arma::vec kernelFunction(double x) const;


  /** \brief Sets the centers for the kernel functions of the DMP according to the canonical system.
   */
  void setCenters();


  /** \brief Sets the standard deviations for the kernel functions  of the DMP
   *  Sets the variance of each kernel equal to squared difference between the current and the next kernel.
   *  @param[in] kernel_std_scaling Scales the std of each kernel by 'kernelStdScaling' (optional, default = 1.0).
  */
  void setStds(double kernel_std_scaling = 1.0);



// ======================================================
// ******************************************************
// ************      Virtual Functions      *************
// ******************************************************
// ======================================================
public:
    /** \brief Returns a deep copy of this. */
    virtual DMP_ *deepCopy() const = 0;


protected:
    /** \brief Returns the shape attractor of the DMP.
     *  @param[in] x The phase variable.
     *  @param[in] y0 Initial position.
     *  @param[in] g Goal position.
     *  @return The shape_attr of the DMP.
     */
    virtual double shapeAttractor(double x, double g) const = 0;


    /** \brief Returns the scaling factor of the forcing term.
     * @param[in] y0 Initial position.
     * @param[in] g Goal position.
     * @return The scaling factor of the forcing term.
     */
    virtual double forcingTermScaling(double g) const = 0;


    /** \brief Calculates the desired values of the scaled forcing term.
     * @param[in] x The phase variable.
     * @param[in] y Position.
     * @param[in] dy Velocity.
     * @param[in] ddy Acceleration.
     * @param[in] y0 initial position.
     * @param[in] g Goal position.
     * @return Fd Desired value of the scaled forcing term.
     */
    virtual double calcFd(double x, double y, double dy, double ddy, double g) const = 0;


    /** \brief Returns the learned forcing term.*/
    virtual double calcLearnedFd(double x, double g) const = 0;


// ======================================================
// ******************************************************
// ************      Public Properties      *************
// ******************************************************
// ======================================================
public:
  
  int N_kernels; ///< number of kernels (basis functions)

  double a_z; ///< parameter 'a_z' relating to the spring-damper system
  double b_z; ///< parameter 'b_z' relating to the spring-damper system

  arma::vec w; ///< N_kernels x 1 vector with the kernels' weights
  arma::vec c; ///< N_kernels x 1 vector with the kernels' centers
  arma::vec h; ///< N_kernels x 1 vector with the kernels' inverse width

  std::shared_ptr<CanonicalClock> can_clock_ptr; ///< pointer to the canonical clock
  std::shared_ptr<GatingFunction> shape_attr_gating_ptr; ///< pointer to gating function for the shape attractor


// ======================================================
// ******************************************************
// ************    Protected Properties     *************
// ******************************************************
// ======================================================
protected:

  double y0; ///< initial position

  double dx; ///< phase variable time derivative
  double dy; ///< state 'y' time derivative
  double dz; ///< state 'z' time derivative
  
  long double zero_tol; ///< small value used to avoid divisions with very small numbers

}; // class DMP

} // namespace dmp_

} // namespace as64_

#endif // AS64_ABSTRACT_DYNAMICAL_MOVEMENT_PRIMITIVE_H
