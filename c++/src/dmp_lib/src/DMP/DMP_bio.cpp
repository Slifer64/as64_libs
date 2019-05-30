#include <dmp_lib/DMP/DMP_bio.h>
#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>

namespace as64_
{

namespace dmp_
{

DMP_bio::DMP_bio(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> can_clock_ptr,
  std::shared_ptr<GatingFunction> shape_attr_gating_ptr) :
  DMP_(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr)
{
  if (!can_clock_ptr) can_clock_ptr.reset(new CanonicalClock());
  if (!shape_attr_gating_ptr) shape_attr_gating_ptr.reset(new SigmoidGatingFunction(1.0, 0.5));
}


DMP_bio *DMP_bio::deepCopy() const
{
  std::shared_ptr<CanonicalClock> can_clock_ptr(new CanonicalClock( *(this->can_clock_ptr.get()) ) );

  std::shared_ptr<GatingFunction> shape_attr_gating_ptr;
  GatingFunction *gating_fun_ptr = this->shape_attr_gating_ptr.get();
  if (dynamic_cast<LinGatingFunction *>(gating_fun_ptr))
    shape_attr_gating_ptr.reset( new LinGatingFunction(*(LinGatingFunction *)gating_fun_ptr) );
  else if (dynamic_cast<ExpGatingFunction *>(gating_fun_ptr))
    shape_attr_gating_ptr.reset( new ExpGatingFunction(*(ExpGatingFunction *)gating_fun_ptr) );
  if (dynamic_cast<SigmoidGatingFunction *>(gating_fun_ptr))
    shape_attr_gating_ptr.reset( new SigmoidGatingFunction(*(SigmoidGatingFunction *)gating_fun_ptr) );
  else
    throw std::runtime_error("[DMP_bio::deepCopy]: Cannot copy unsupported Gating function type!");

  return new DMP_bio(this->N_kernels, this->a_z, this->b_z, can_clock_ptr, shape_attr_gating_ptr);
}


double DMP_bio::shapeAttractor(double x, double g) const
{
  double s_attr_gating = this->shapeAttrGating(x);
  double f_scale = this->forcingTermScaling(g);
  double shape_attr = s_attr_gating * f_scale * ( this->forcingTerm(x) - (g-this->y0) );
  return shape_attr;
}


double DMP_bio::forcingTermScaling(double g) const
{
  double f_scale = this->a_z*this->b_z;
  return f_scale;
}


double DMP_bio::calcFd(double x, double y, double dy, double ddy, double g) const
{
  double s_attr_gating = this->shapeAttrGating(x);
  double tau = this->getTau();
  double K = this->a_z * this->b_z;
  double Fd = (ddy*std::pow(tau,2) - this->goalAttractor(x, y, tau*dy, g) + K*(g-this->y0)*s_attr_gating);
  return Fd;
}


double DMP_bio::calcLearnedFd(double x, double g) const
{
  double s = this->shapeAttrGating(x) * this->forcingTermScaling(g);
  double Fd = this->shapeAttractor(x, g) + s*(g-this->y0);
  return Fd;
}

} // namespace dmp_

} // namespace as64_
