#ifndef AS64_CART_POS_DYNAMICAL_MOVEMENT_PRIMITIVE_H
#define AS64_CART_POS_DYNAMICAL_MOVEMENT_PRIMITIVE_H

#include <cmath>
#include <vector>
#include <cstring>
#include <memory>
#include <exception>
#include <armadillo>

#include <dmp_lib/utils.h>
#include <dmp_lib/DMP/DMP_.h>
#include <dmp_lib/CanonicalClock/CanonicalClock.h>
#include <dmp_lib/GatingFunction/GatingFunction.h>

namespace as64_
{

namespace dmp_
{

class DMP_pos
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
  DMP_pos(dmp_::TYPE dmp_type, const arma::uvec &N_kernels, const arma::vec &a_z, const arma::vec &b_z,
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
  void train(dmp_::TrainMethod train_method, const arma::rowvec &Time, const arma::mat &Pd_data,
             const arma::mat &dPd_data, const arma::mat &ddPd_data, arma::vec *train_err=0);


  /** \brief Returns the derivatives of the DMP states.
   *  @param[in] x phase variable.
   *  @param[in] y \a y state of the DMP.
   *  @param[in] z \a z state of the DMP.
   *  @param[in] g Goal position.
   *  @param[in] y_c Coupling term for the dynamical equation of the \a y state.
   *  @param[in] z_c Coupling term for the dynamical equation of the \a z state.
   *  @return  The states derivatives of the DMP as a 3x1 vector (dz, dy, dx).
   */
  void update(double x, const arma::vec &Y, const arma::vec &Z, const arma::vec &Yg,
              const arma::vec &Y_c=arma::vec().zeros(3), const arma::vec &Z_c=arma::vec().zeros(3));


    /** \brief Returns the DMP's phase variable derivative. @update must be called first.*/
  double getXdot() const { return this->dx; }

  
  /** \brief Returns the DMP's phase variable derivative. @update must be called first.*/
  arma::vec getYdot() const { return this->dY; }


  /** \brief Returns the DMP's phase variable derivative. @update must be called first.*/
  arma::vec getZdot() const { return this->dZ; }


  /** \brief Returns the DMP's phase variable derivative. @update must be called first.*/
  arma::vec getYddot(double tau_dot=0, const arma::vec &Yc_dotc=arma::vec().zeros(3)) const;


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
  arma::vec calcYddot(double x, const arma::vec &Y, const arma::vec &dY, const arma::vec &Yg,
                      double tau_dot=0, const arma::vec &Yc=arma::vec().zeros(3),
                      const arma::vec &Zc=arma::vec().zeros(3), const arma::vec &Yc_dot=arma::vec().zeros(3));


  /** \brief Sets the DMP's initial position.
   *  @param[in] y0 The initial position.
   */
  void setY0(const arma::vec &Y0);

  /** \brief Returns the DMP's initial position.
   *  @return the initial position.
   */
  arma::vec getY0() const;

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

  static std::shared_ptr<DMP_pos> importFromFile(std::istream &in);

  arma::mat getAcellPartDev_g_tau(double t, const arma::vec &Y, const arma::vec &dY, const arma::vec &Y0, double x, const arma::vec &Yg, double tau) const;

  void printParams() const
{
  std::cerr << "==========================================\n";
  std::cerr << "==========================================\n";
  std::cerr << "[DMP_pos::printParams]: Printing params:\n";
  for (int i=0;i <dmp.size(); i++)
  {
    std::cerr << "===> DMP " << i << ":\n";
    std::cerr << "N_kernels = \n" << dmp[i]->N_kernels << "\n";
    std::cerr << "a_z = \n" << dmp[i]->a_z << "\n";
    std::cerr << "b_z = \n" << dmp[i]->b_z << "\n";
    std::cerr << "tau = \n" << dmp[i]->getTau() << "\n";
    std::cerr << "y0 = \n" << dmp[i]->y0 << "\n";
    std::cerr << "w = \n" << dmp[i]->w.t() << "\n";
    std::cerr << "c = \n" << dmp[i]->c.t() << "\n";
    std::cerr << "h = \n" << dmp[i]->h.t() << "\n";
  }
  std::cerr << "==========================================\n";
  std::cerr << "==========================================\n";

}

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

  double dx; ///< phase variable time derivative
  arma::vec dY; ///< state 'Y' time derivative
  arma::vec dZ; ///< state 'Z' time derivative

}; // class DMP_pos


} // namespace dmp_

} // namespace as64_

#endif // AS64_CART_POS_DYNAMICAL_MOVEMENT_PRIMITIVE_H
