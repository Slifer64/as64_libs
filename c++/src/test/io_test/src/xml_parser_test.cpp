#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>
#include <armadillo>

#include <io_lib/xml_parser.h>

template<class T>
std::string stdVec2Str(const std::vector<T> &v, const std::string &separator=", ")
{
  std::ostringstream oss;
  for (int i=0;i<v.size();i++)
  {
    if (i==v.size()-1) oss << v[i];
    else oss << v[i] << separator;
  }
  return oss.str();
}

std::string stdVec2Str(const std::vector<bool> &v, const std::string &separator=", ")
{
  std::vector<std::string> v2(v.size());
  for (int i=0;i<v.size();i++)
  {
    v2[i] = v[i]?"true":"false";
  }
  return stdVec2Str(v2, separator);
}

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "xml_parser_test_node");

  std::string confi_file_path = ros::package::getPath("io_test") + "/config/xml_parser_test_config.yaml";

  as64_::io_::XmlParser parser(confi_file_path);

  int i;
  double d;
  bool b;
  std::string str;
  std::vector<int> std_vec_int;
  std::vector<double> std_vec_double;
  std::vector<bool> std_vec_bool;
  std::vector<std::string> std_vec_string;
  arma::vec arma_vec;
  arma::rowvec arma_rowvec;
  arma::mat arma_mat;

  // =========== Parse values  ==================

  if (!parser.getParam("i",i)) throw std::ios_base::failure("Failed to read parameter \"i\".");
  if (!parser.getParam("d",d)) throw std::ios_base::failure("Failed to read parameter \"d\".");
  if (!parser.getParam("b",b)) throw std::ios_base::failure("Failed to read parameter \"b\".");
  if (!parser.getParam("str",str)) throw std::ios_base::failure("Failed to read parameter \"str\".");

  if (!parser.getParam("std_vec_int",std_vec_int)) throw std::ios_base::failure("Failed to read parameter \"std_vec_int\".");
  if (!parser.getParam("std_vec_double",std_vec_double)) throw std::ios_base::failure("Failed to read parameter \"std_vec_double\".");
  if (!parser.getParam("std_vec_bool",std_vec_bool)) throw std::ios_base::failure("Failed to read parameter \"std_vec_bool\".");
  if (!parser.getParam("std_vec_string",std_vec_string)) throw std::ios_base::failure("Failed to read parameter \"std_vec_string\".");

  if (!parser.getParam("arma_vec",arma_vec)) throw std::ios_base::failure("Failed to read parameter \"arma_vec\".");
  if (!parser.getParam("arma_rowvec",arma_rowvec)) throw std::ios_base::failure("Failed to read parameter \"arma_rowvec\".");
  if (!parser.getParam("arma_mat",arma_mat)) throw std::ios_base::failure("Failed to read parameter \"arma_mat\".");

  // =========== Print values  ==================
  std::cout << "===================================\n";
  std::cout << "i = " << i << "\n";
  std::cout << "d = " << d << "\n";
  std::cout << "b = " << b << "\n";
  std::cout << "str = " << str << "\n";

  std::cout << "std_vec_int = " << stdVec2Str(std_vec_int) << "\n";
  std::cout << "std_vec_double = " << stdVec2Str(std_vec_double) << "\n";
  std::cout << "std_vec_bool = " << stdVec2Str(std_vec_bool) << "\n";
  std::cout << "std_vec_string = " << stdVec2Str(std_vec_string) << "\n";

  std::cout << "arma_vec = \n" << arma_vec << "\n";
  std::cout << "arma_rowvec = " << arma_rowvec << "\n";
  std::cout << "arma_mat = \n" << arma_mat << "\n";

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
