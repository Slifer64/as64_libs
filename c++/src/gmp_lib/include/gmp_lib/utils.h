#ifndef AS64_gmp_lib_UTILS_H
#define AS64_gmp_lib_UTILS_H

//#define GMP_DEBUG_
//#define WSoG_DEBUG_
//#define GMP_NDOF_DEBUG_
//#define GMPo_DEBUG_

namespace as64_
{

  namespace gmp_
  {

    enum UPDATE_TYPE
    {
      POS = 0,
      VEL = 1,
      ACCEL = 2
    };

    struct Phase
    {
      double x, x_dot, x_ddot;
      Phase(double x1, double x1_dot=0, double x1_ddot=0): x(x1), x_dot(x1_dot), x_ddot(x_ddot) {}
    };

  } // namespace gmp_

} // namespace as64_

#endif // AS64_gmp_lib_UTILS_H
