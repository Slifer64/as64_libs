#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>

#include <io_lib/io_utils.h>

using namespace as64_::io_;

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "IO_test_node");
  ros::NodeHandle nh_("~");

  std::string path = ros::package::getPath("io_test") + "/data/";

  std::string sys_cmd;
  sys_cmd = "mkdir " + path;
  system(sys_cmd.c_str());


  // ===========  read cmd args  ==================
  bool binary;
  int precision;
  std::string filename;

  if (!nh_.getParam("binary", binary)) binary = false;
  if (!nh_.getParam("precision", precision)) precision = 5;
  if (!nh_.getParam("filename", filename)) filename = "data";

  filename = path + filename;
  if (binary) filename += ".bin";
  else filename += ".txt";

  std::cout << "filename = \"" << filename << "\"\n";

  // ===========  create data  ==================
  arma::mat A(24, 25, arma::fill::randu);
  arma::mat B(36, 28, arma::fill::randu);
  std::vector<arma::mat> m(2);
  m[0] = A;
  m[1] = B;
  arma::vec v(32, 1, arma::fill::randu);
  arma::rowvec rowV(1, 61, arma::fill::randu);
  double d_scalar = 15.2;
  int i_scalar = -4;

  // ===========  write data  ==================
  std::ofstream out;
  if (binary) out.open(filename, std::ios::out | std::ios::binary);
  else out.open(filename, std::ios::out);

  if (!out) throw std::ios_base::failure(std::string("Couldn't create file \"") + filename + "\"...\n");

  write_mat(A, out, binary, precision);
  write_rowVec(rowV, out, binary, precision);
  write_scalar(d_scalar, out, binary, precision);
  write_mat(B, out, binary, precision);
  write_scalar(i_scalar, out, binary, precision);
  write_vec(v, out, binary, precision);
  write_vec_mat(m, out, binary, precision);

  out.close();

  // ===========  read data  ==================
  std::ifstream in(filename, std::ios::in);
  if (!in) throw std::ios_base::failure(std::string("Couldn't open file \"") + filename + "\"...\n");

  arma::mat A2;
  arma::mat B2;
  std::vector<arma::mat> m2;
  arma::vec v2;
  arma::rowvec rowV2;
  double d_scalar2;
  int i_scalar2;

  read_mat(A2, in, binary);
  read_rowVec(rowV2, in, binary);
  read_scalar(d_scalar2, in, binary);
  read_mat(B2, in, binary);
  read_scalar(i_scalar2, in, binary);
  read_vec(v2, in, binary);
  read_vec_mat(m2, in, binary);

  in.close();

  // ===========  test  ==================

  double A_err  = arma::norm(arma::vectorise(A)-arma::vectorise(A2));
  double B_err  = arma::norm(arma::vectorise(B)-arma::vectorise(B2));
  double v_err  = arma::norm(arma::vectorise(v)-arma::vectorise(v2));
  double rowV_err  = arma::norm(arma::vectorise(rowV)-arma::vectorise(rowV2));
  double d_scalar_err = std::fabs(d_scalar - d_scalar2);
  double i_scalar_err = std::fabs(i_scalar - i_scalar2);

  double m_err = 0;
  for (int k=0;k<m.size();k++){
      m_err = m_err + arma::norm(arma::vectorise(m[k])-arma::vectorise(m2[k]));
  }

  std::cout << "A_err = " << A_err << "\n";
  std::cout << "B_err = " << B_err << "\n";
  std::cout << "v_err = " << v_err << "\n";
  std::cout << "rowV_err = " << rowV_err << "\n";
  std::cout << "m_err = " << m_err << "\n";
  std::cout << "d_scalar_err = " << d_scalar_err << "\n";
  std::cout << "i_scalar_err = " << i_scalar_err << "\n";

  sys_cmd = "rm -rf " + path;
  system(sys_cmd.c_str());

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
