/** Sigmoid Gating Function class
 *  Implements a sigmoidal gating function, u=f(x), x:[0 1]->u:[u0 u_end],
 * where u0 is the initial and u_end the final value.
 * The output of the gating function is:
 *    u = u0 * ( 1 / (1 + exp(-a_u*(x-c)) ) );
 *   du = -a_u*u0 * ( exp(-a_u*(x-c)) / (1 + exp(-a_u*(x-c)) )^2 );
 */

#ifndef AS64_SIGMOID_GATING_FUNCTION_H
#define AS64_SIGMOID_GATING_FUNCTION_H

#include <dmp_lib/GatingFunction/GatingFunction.h>

namespace as64_
{

namespace dmp_
{

class SigmoidGatingFunction: public GatingFunction
{
public:
  SigmoidGatingFunction(double u0 = 1.0, double u_end = 0.8);

  virtual void init(double u0, double u_end);

  virtual double getOutput(double x) const;
  virtual arma::rowvec getOutput(const arma::rowvec &x) const;

  virtual double getOutputDot(double x) const;
  virtual arma::rowvec getOutputDot(const arma::rowvec &x) const;

  virtual double getPartDev_1oTau(double t, double x) const;

  void exportToFile(std::ostream &out) const;

private:
  double u0; ///< initial value of the gating function
  double a_u; ///< the rate of evolution of the gating function
  double c; ///< center of the exponential in the sigmoid
}; // class SigmoidGatingFunction

} // namespace dmp_

} // namespace as64_

#endif // AS64_SIGMOID_GATING_FUNCTION_H
