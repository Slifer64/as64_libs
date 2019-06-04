#include <dmp_lib/DMP/DMP.h>
#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>

#include <dmp_lib/io/io.h>

namespace as64_
{

namespace dmp_
{

DMP::DMP(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> can_clock_ptr,
  std::shared_ptr<GatingFunction> shape_attr_gating_ptr) :
  DMP_(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr)
{}


DMP *DMP::deepCopy() const
{
  std::shared_ptr<CanonicalClock> can_clock_ptr(new CanonicalClock( *(this->can_clock_ptr.get()) ) );

  std::shared_ptr<GatingFunction> shape_attr_gating_ptr;
  GatingFunction *g_ptr = this->shape_attr_gating_ptr.get();
  if (dynamic_cast<LinGatingFunction *>(g_ptr))
    shape_attr_gating_ptr.reset( new LinGatingFunction(*(LinGatingFunction *)g_ptr) );
  else if (dynamic_cast<ExpGatingFunction *>(g_ptr))
    shape_attr_gating_ptr.reset( new ExpGatingFunction(*(ExpGatingFunction *)g_ptr) );
  if (dynamic_cast<SigmoidGatingFunction *>(g_ptr))
    shape_attr_gating_ptr.reset( new SigmoidGatingFunction(*(SigmoidGatingFunction *)g_ptr) );
  else
    throw std::runtime_error("[DMP::deepCopy]: Cannot copy unsupported Gating function type!");

  return new DMP(this->N_kernels, this->a_z, this->b_z, can_clock_ptr, shape_attr_gating_ptr);
}

void DMP::exportToFile(std::ostream &out) const
{
  dmp_::write_scalar(static_cast<int>(dmp_::TYPE::STD), out);
  dmp_::write_scalar(N_kernels, out);
  dmp_::write_scalar(a_z, out);
  dmp_::write_scalar(b_z, out);
  dmp_::write_mat(w, out);
  dmp_::write_mat(c, out);
  dmp_::write_mat(h, out);
  dmp_::write_scalar(zero_tol, out);
  can_clock_ptr->exportToFile(out);
  shape_attr_gating_ptr->exportToFile(out);
  dmp_::write_scalar(y0, out);
}


std::shared_ptr<DMP_> DMP::importFromFile(std::istream &in)
{
  std::shared_ptr<DMP_> dmp(new DMP(6, 20, 5));

  dmp_::read_scalar(dmp->N_kernels, in);
  dmp_::read_scalar(dmp->a_z, in);
  dmp_::read_scalar(dmp->b_z, in);
  dmp_::read_mat(dmp->w, in);
  dmp_::read_mat(dmp->c, in);
  dmp_::read_mat(dmp->h, in);
  dmp_::read_scalar(dmp->zero_tol, in);
  dmp->can_clock_ptr = CanonicalClock::importFromFile(in);
  dmp->shape_attr_gating_ptr = GatingFunction::importFromFile(in);
  double y0;
  dmp_::read_scalar(y0, in);
  dmp->setY0(y0);

  return dmp;
}


double DMP::shapeAttractor(double x, double g) const
{
  double f = this->forcingTerm(x);
  double f_scale = this->forcingTermScaling(g);
  double shape_attr = f * f_scale * this->shapeAttrGating(x);
  return shape_attr;
}


double DMP::forcingTermScaling(double g) const
{
  double f_scale = (g-this->y0);
  return f_scale;
}


double DMP::calcFd(double x, double y, double dy, double ddy, double g) const
{
  double tau = this->getTau();
  double Fd = (ddy*std::pow(tau,2) - this->goalAttractor(x, y, tau*dy, g));
  return Fd;
}


double DMP::calcLearnedFd(double x, double g) const
{
  return this->shapeAttractor(x, g);
}

} // namespace dmp_

} // namespace as64_
