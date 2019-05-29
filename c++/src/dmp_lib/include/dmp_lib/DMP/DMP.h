#ifndef AS64_DYNAMICAL_MOVEMENT_PRIMITIVE_H
#define AS64_DYNAMICAL_MOVEMENT_PRIMITIVE_H

#include <cmath>
#include <vector>
#include <cstring>
#include <memory>
#include <exception>
#include <armadillo>

#include <dmp_lib/DMP/DMP_.h>

namespace as64_
{

namespace dmp_
{

class DMP : public DMP_
{
// methods
public:

  /** \brief DMP constructor.
   * @param[in] N_kernels The number of kernels.
   * @param[in] a_z Parameter 'a_z' relating to the spring-damper system.
   * @param[in] b_z Parameter 'b_z' relating to the spring-damper system.
   * @param[in] can_clock_ptr Pointer to a DMP canonical system object.
   * @param[in] shape_attr_gating_ptr Pointer to gating function for the shape attractor.
   */
  DMP(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> can_clock_ptr=std::shared_ptr<CanonicalClock>(),
      std::shared_ptr<GatingFunction> shape_attr_gating_ptr=std::shared_ptr<GatingFunction>());


  DMP *deepCopy() const;

private:

/** \brief Returns the shape attractor of the DMP.
       *  @param[in] x The phase variable.
       *  @param[in] y0 Initial position.
       *  @param[in] g Goal position.
       *  @return The shape_attr of the DMP.
       */
    double shapeAttractor(double x, double y0, double g) const;

    /** \brief Returns the scaling factor of the forcing term.
     * @param[in] y0 Initial position.
     * @param[in] g Goal position.
     * @return The scaling factor of the forcing term.
     */
    double forcingTermScaling(double y0, double g) const;

    /** \brief Calculates the desired values of the scaled forcing term.
     * @param[in] x The phase variable.
     * @param[in] y Position.
     * @param[in] dy Velocity.
     * @param[in] ddy Acceleration.
     * @param[in] y0 initial position.
     * @param[in] g Goal position.
     * @return Fd Desired value of the scaled forcing term.
     */
    double calcFd(double x, double y, double dy, double ddy, double y0, double g) const;

    double calcLearnedFd(double x, double y0, double g) const;

}; // class DMP

} // namespace dmp_

} // namespace as64_

#endif // AS64_DYNAMICAL_MOVEMENT_PRIMITIVE_H
