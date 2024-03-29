/** Gating Function class (abstract)
 * Implements a gating function, u=f(x), x:[0 1]->u:[u0 u_end],
 * where u0 is the initial and u_end the final value.
 */

#ifndef AS64_GATING_FUNCTION_H
#define AS64_GATING_FUNCTION_H

#include <cmath>
#include <exception>
#include <fstream>
#include <memory>
#include <armadillo>
#include <dmp_lib/utils.h>

namespace as64_
{

namespace dmp_
{

class GatingFunction
{
public:
  /** \brief Gating Function Constructor.
   */
  GatingFunction();

  /** \brief Initializes the gating function.
   *  @param[in] u0 Initial value of the gating function.
   *  @param[in] u_end Final value of the gating function.
   */
  virtual void init(double u0, double u_end) = 0;

  /** \brief Returns the gating function's output for the specified timestamps.
   *  @param[in] x A timestamp or vector of timestamps.
   *  @return u Value or vector of values of the gating function's output.
   */
  virtual double getOutput(double x) const = 0;
  virtual arma::rowvec getOutput(const arma::rowvec &x) const = 0;

  /** \brief Returns the gating function's derivated output for the specified timestamps.
   *  @param[in] x A timestamp or vector of timestamps.
   *  @return u Value or vector of values of the gating function's derivated output.
   */
  virtual double getOutputDot(double x) const = 0;
  virtual arma::rowvec getOutputDot(const arma::rowvec &x) const = 0;

  /** \brief Returns the partial derivative of the gating output wrt 1/tau.
   * @param[in] t Timestamp.
   * @param[in] x Phase variable.
   * @return Partial derivative of the gating wrt 1/tau.
   */
  virtual double getPartDev_1oTau(double t, double x) const = 0;


  virtual void exportToFile(std::ostream &out) const;
  static std::shared_ptr<GatingFunction> importFromFile(std::istream &in);

}; // class GatingFunction

} // namespace dmp_

} // namespace as64_

#endif // AS64_GATING_FUNCTION_H
