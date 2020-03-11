#include <gmp_lib/GatingFunction/LinGatingFunction.h>
#include <gmp_lib/io/io.h>

namespace as64_
{

namespace gmp_
{

LinGatingFunction::LinGatingFunction(double u0, double u_end)
{
  this->init(u0, u_end);
}

void LinGatingFunction::init(double u0, double u_end)
{
  this->u0 = u0;
  this->a_u = this->u0 - u_end;
}

double LinGatingFunction::getOutput(double x) const
{
  double u = this->u0 - this->a_u*x;
  return u;
}

arma::rowvec LinGatingFunction::getOutput(const arma::rowvec &x) const
{
  arma::rowvec u = this->u0 - this->a_u*x;
  return u;
}

double LinGatingFunction::getOutputDot(double x) const
{
  double du = -this->a_u;
  return du;
}

arma::rowvec LinGatingFunction::getOutputDot(const arma::rowvec &x) const
{
  arma::rowvec du = -this->a_u * arma::rowvec().ones(x.size());
  return du;
}

double LinGatingFunction::getPartDev_1oTau(double t, double x) const
{
  return -this->a_u * t * this->getOutput(x);
}

void LinGatingFunction::exportToFile(std::ostream &out) const
{
  gmp_::write_scalar((int)GatingType::LIN, out);
  out.write((const char *)(dynamic_cast<const LinGatingFunction *>(this)), sizeof(LinGatingFunction));
}

} // namespace gmp_

} // namespace as64_
