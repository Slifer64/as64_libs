#ifndef AS64_DMP_LIB_UTILS_H
#define AS64_DMP_LIB_UTILS_H

namespace as64_
{

  namespace dmp_
  {

    enum TYPE
    {
      STD = 1001,
      BIO = 1003
    };

    enum TrainMethod
    {
      LWR = 7,
      LS = 11
    };

    enum GatingType
    {
      LIN = 0,
      EXP = 1,
      SIGMOID = 2,
      UNKNOWN = 3
    };


  } // namespace dmp_

} // namespace as64_

#endif // AS64_DMP_LIB_UTILS_H
