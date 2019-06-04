#include <dmp_lib/GatingFunction/GatingFunction.h>
#include <dmp_lib/GatingFunction/LinGatingFunction.h>
#include <dmp_lib/GatingFunction/ExpGatingFunction.h>
#include <dmp_lib/GatingFunction/SigmoidGatingFunction.h>
#include <dmp_lib/io/io.h>

namespace as64_
{

namespace dmp_
{

GatingFunction::GatingFunction() {}

void GatingFunction::exportToFile(std::ostream &out) const
{
  throw std::runtime_error("[GatingFunction::exportToFile]: The object must be assigned a derived class pointer!");
}

std::shared_ptr<GatingFunction> GatingFunction::importFromFile(std::istream &in)
{
  int g_type_int;
  dmp_::read_scalar(g_type_int, in);
  GatingType g_type = static_cast<GatingType>(g_type_int);

  std::shared_ptr<GatingFunction> g_fun;

  if (g_type == GatingType::LIN)
  {
    LinGatingFunction lin_g_fun = LinGatingFunction();
    in.read(reinterpret_cast<char *>(&lin_g_fun), sizeof(LinGatingFunction));
    g_fun.reset(new LinGatingFunction(lin_g_fun));
  }
  else if (g_type == GatingType::EXP)
  {
    ExpGatingFunction exp_g_fun = ExpGatingFunction();
    in.read(reinterpret_cast<char *>(&exp_g_fun), sizeof(ExpGatingFunction));
    g_fun.reset(new ExpGatingFunction(exp_g_fun));
  }
  else if (g_type == GatingType::SIGMOID) {
    SigmoidGatingFunction sig_g_fun = SigmoidGatingFunction();
    in.read(reinterpret_cast<char *>(&sig_g_fun), sizeof(SigmoidGatingFunction));
    g_fun.reset(new SigmoidGatingFunction(sig_g_fun));
  }
  else if (g_type == GatingType::UNKNOWN)
  {
    throw std::runtime_error("[DMP_::importFromfile]: Cannot import unsupported gating function type.");
  }

  return g_fun;
}

} // namespace dmp_

} // namespace as64_
