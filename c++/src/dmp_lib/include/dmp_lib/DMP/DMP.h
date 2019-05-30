#ifndef AS64_STD_DYNAMICAL_MOVEMENT_PRIMITIVE_H
#define AS64_STD_DYNAMICAL_MOVEMENT_PRIMITIVE_H

#include <dmp_lib/DMP/DMP_.h>

namespace as64_
{

namespace dmp_
{

class DMP : public DMP_
{
public:

  DMP(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> can_clock_ptr=std::shared_ptr<CanonicalClock>(),
      std::shared_ptr<GatingFunction> shape_attr_gating_ptr=std::shared_ptr<GatingFunction>());

  DMP *deepCopy() const;

protected:

    double shapeAttractor(double x, double g) const;

    double forcingTermScaling(double g) const;

    double calcFd(double x, double y, double dy, double ddy, double g) const;

    double calcLearnedFd(double x, double g) const;

}; // class DMP

} // namespace dmp_

} // namespace as64_

#endif // AS64_STD_DYNAMICAL_MOVEMENT_PRIMITIVE_H
