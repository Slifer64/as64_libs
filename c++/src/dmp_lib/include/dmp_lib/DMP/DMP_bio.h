#ifndef AS64_BIO_DYNAMICAL_MOVEMENT_PRIMITIVE_H
#define AS64_BIO_DYNAMICAL_MOVEMENT_PRIMITIVE_H

#include <dmp_lib/DMP/DMP_.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>

namespace as64_
{

namespace dmp_
{

class DMP_bio : public DMP_
{
public:

  DMP_bio(int N_kernels, double a_z, double b_z,
          std::shared_ptr<CanonicalClock> can_clock_ptr=std::shared_ptr<CanonicalClock>(new CanonicalClock()),
          std::shared_ptr<GatingFunction> shape_attr_gating_ptr=std::shared_ptr<GatingFunction>(new SigmoidGatingFunction(1.0, 0.5)));

  DMP_bio *deepCopy() const;

  void exportToFile(std::ostream &out) const;

  static std::shared_ptr<DMP_> importFromFile(std::istream &in);

protected:

  double shapeAttractor(double x, double g) const;

  double forcingTermScaling(double g) const;

  double calcFd(double x, double y, double dy, double ddy, double g) const;

  double calcLearnedFd(double x, double g) const;

}; // class DMP_bio

} // namespace dmp_

} // namespace as64_

#endif // AS64_BIO_DYNAMICAL_MOVEMENT_PRIMITIVE_H
