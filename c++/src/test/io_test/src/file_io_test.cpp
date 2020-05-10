#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <cstdio>

#include <io_lib/file_io.h>

using namespace as64_::io_;

template<typename T>
double stdVecError(const std::vector<T> &v1, const std::vector<T> &v2)
{
  double err = 0;
  for (int i=0; i<v1.size(); i++) err += std::fabs(v1[i]-v2[i]);
  return err;
}

template<typename T>
double armaMatError(const arma::Mat<T> &m1, const arma::Mat<T> &m2)
{
  double err = 0;
  arma::Col<T> v1 = arma::vectorise(m1), v2 = arma::vectorise(m2);
  for (int i=0; i<v1.size(); i++) err += std::fabs(v1(i)-v2(i));
  return err;
}

template<typename T>
double eigenMatError(const Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> &m1, const Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> &m2)
{
  return (m1-m2).cwiseAbs().sum();
}

template<typename T>
double eigenMatError(const Eigen::Matrix<T,Eigen::Dynamic,1> &m1, const Eigen::Matrix<T,Eigen::Dynamic,1> &m2)
{
  return (m1-m2).cwiseAbs().sum();
}

template<typename T>
double eigenMatError(const Eigen::Matrix<T,1,Eigen::Dynamic> &m1, const Eigen::Matrix<T,1,Eigen::Dynamic> &m2)
{
  return (m1-m2).cwiseAbs().sum();
}

// =========================================================
// ======================   MAIN   =========================
// =========================================================
int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "temp_node");
  ros::NodeHandle nh_("~");

  std::string path = ros::package::getPath("io_test") + "/data/";

  std::string filename = "data.bin";

  std::cout << "filename = \"" << filename << "\"\n";

//  std::ofstream os(filename, std::ios::out);
//  if (!os) throw std::runtime_error("Failed to open/create file \"" + filename + "\"...");
//  int a = 8;
//  os.seekp(0);
//  os.write((const char *)(&a), sizeof(a));
//  os.close();
//
//  std::ifstream is(filename, std::ios::in);
//  if (!is) throw std::runtime_error("Failed to open/create file \"" + filename + "\"...");
//  int a2 = 96;
//  is.read((char *)(&a2), sizeof(a2));
//  is.close();
//
//  std::cerr << "a = " << a << "\n";
//  std::cout << "a2 = " << a2 << "\n";
//
//
//  return 0;

//  try{
//    std::fstream fs;
////    fs.open("data.bin", std::ios::in | std::ios::out); // SUCCESS+ , ERROR-
////  fs.open("data.bin", std::ios::in | std::ios::trunc);  // ERROR+/-,
////  fs.open("data.bin", std::ios::out | std::ios::trunc);  // SUCCESS+/-
//  fs.open("data.bin", std::ios::in | std::ios::out | std::ios::trunc); // SUCCESS+/-
//
//    int r = std::remove("test.txt");
//    if (r == 0) std::cerr << "Delete was success!\n";
//    else std::cerr << "Delete failed...\n";
//
//    if (fs) std::cerr << "Open SUCCESS!\n";
//    else
//    {
//      std::cerr << "Error openning the file...\n";
//    }
//
//    fs.close();
//
//  }
//  catch(std::exception &e)
//  {
//    std::cerr << e.what() << "\n";
//  }
//
//  return 0;
//
//  std::remove(filename.c_str());

  // ===========  write data  =============
  FileIO out(filename, FileIO::out | FileIO::trunc);

  out.printHeader();

  bool b = true;
  int i = -9;
  unsigned u = 5;
  float f = 3.1;
  double d = 4.6;
  arma::rowvec arma_rowvec = { 5, -3, 7, -2 };
  arma::vec arma_vec = { -0.5, 0.2, -0.3, 0.6 };
  arma::imat arma_imat = { {1,-5, 2}, {-2, 9, -3} };
  arma::umat arma_umat = { {7, 5},{3, 6}, {6, 8} };
  arma::fmat arma_fmat = { {1.2, -6.1, 4.3}, {-3.2, 7.4, -9.1}, {7.2, 6.3, -7.6} };
  arma::mat arma_mat = arma::mat().randu(5,3);
  Eigen::MatrixXf eig_fmat(2,3);
  eig_fmat << 1.2, -6.1, -3.2, 7.4, -5.2, 2.1;
  Eigen::MatrixXd eig_mat(2,2);
  eig_mat << 1.2, -6.1, -3.2, 7.4;
  Eigen::VectorXi eig_ivec(3);
  eig_ivec << 4, -3, -5;
  Eigen::RowVectorXf eig_frowvec(4);
  eig_ivec << 2.4, -5.3, -6.5, 7.6;
  std::vector<int> std_ivec = {-4, 6, 8, -12};
  std::vector<unsigned long> std_ulvec = {173, 675, 78, 964};


  out.write("b", b);
  out.write("i", i);
  out.write("u", u);
  out.write("arma_rowvec", arma_rowvec);
  out.write("f", f);
  out.write("d", d);
  out.write("arma_vec", arma_vec);
  out.write("arma_imat", arma_imat);
  out.write("arma_umat", arma_umat);
  out.write("arma_fmat", arma_fmat);
  out.write("arma_mat", arma_mat);
  out.write("eig_fmat", eig_fmat);
  out.write("eig_mat", eig_mat);
  out.write("eig_ivec", eig_ivec);
  out.write("eig_frowvec", eig_frowvec);
  out.write("std_ivec", std_ivec);
  out.write("std_ulvec", std_ulvec);

  out.printHeader();
  std::cout << "================================\n";

  // ===========  read data  ==================
  FileIO in(filename, FileIO::in | FileIO::out | FileIO::trunc);

  in.printHeader();

  bool b2;
  int i2;
  unsigned u2;
  float f2;
  double d2;
  arma::rowvec arma_rowvec2;
  arma::vec arma_vec2;
  arma::imat arma_imat2;
  arma::umat arma_umat2;
  arma::fmat arma_fmat2;
  arma::mat arma_mat2;
  Eigen::MatrixXf eig_fmat2;
  Eigen::MatrixXd eig_mat2;
  Eigen::VectorXi eig_ivec2;
  Eigen::RowVectorXf eig_frowvec2;
  std::vector<int> std_ivec2;
  std::vector<unsigned long> std_ulvec2;

  in.read("std_ivec", std_ivec2);
  in.read("std_ulvec", std_ulvec2);

  in.read("arma_mat", arma_mat2);
  in.read("arma_rowvec", arma_rowvec2);
  in.read("arma_vec", arma_vec2);
  in.read("arma_imat", arma_imat2);
  in.read("arma_fmat", arma_fmat2);
  in.read("arma_umat", arma_umat2);

  in.read("d", d2);
  in.read("f", f2);
  in.read("i", i2);
  in.read("u", u2);
  in.read("b", b2);

  in.read("eig_fmat", eig_fmat2);
  in.read("eig_mat", eig_mat2);
  in.read("eig_ivec", eig_ivec2);
  in.read("eig_frowvec", eig_frowvec2);


  std::cout << "================================\n";
  int w = 30;
  std::cout << std::setw(w) << std::left << "b-b2 " << ": " << std::fabs(b-b2) << "\n";
  std::cout << std::setw(w) << std::left << "i-i2 " << ": " << std::fabs(i-i2) << "\n";
  std::cout << std::setw(w) << std::left << "u-u2 " << ": " << std::fabs(u-u2) << "\n";
  std::cout << std::setw(w) << std::left << "f-f2 " << ": " << std::fabs(f-f2) << "\n";
  std::cout << std::setw(w) << std::left << "d-d2 " << ": " << std::fabs(d-d2) << "\n";
  std::cout << std::setw(w) << std::left << "arma_rowvec-arma_rowvec2 " << ": " << armaMatError(arma_rowvec, arma_rowvec2) << "\n";
  std::cout << std::setw(w) << std::left << "arma_vec-arma_vec2 " << ": " << armaMatError(arma_vec, arma_vec2) << "\n";
  std::cout << std::setw(w) << std::left << "arma_mat-arma_mat2 " << ": " << armaMatError(arma_mat, arma_mat2) << "\n";
  std::cout << std::setw(w) << std::left << "arma_imat-arma_imat2 " << ": " << armaMatError(arma_imat, arma_imat2) << "\n";
  std::cout << std::setw(w) << std::left << "arma_umat-arma_umat2 " << ": " << armaMatError(arma_umat, arma_umat2) << "\n";
  std::cout << std::setw(w) << std::left << "arma_fmat-arma_fmat2 " << ": " << armaMatError(arma_fmat, arma_fmat2) << "\n";

  std::cout << std::setw(w) << std::left << "eig_fmat-eig_fmat2 " << ": " << eigenMatError(eig_fmat, eig_fmat2) << "\n";
  std::cout << std::setw(w) << std::left << "eig_mat-eig_mat2 " << ": " << eigenMatError(eig_mat, eig_mat2) << "\n";
  std::cout << std::setw(w) << std::left << "eig_ivec-eig_ivec2 " << ": " << eigenMatError(eig_ivec, eig_ivec2) << "\n";
  std::cout << std::setw(w) << std::left << "eig_frowvec-eig_frowvec2 " << ": " << eigenMatError(eig_frowvec, eig_frowvec2) << "\n";

  std::cout << std::setw(w) << std::left << "std_ivec-std_ivec2 " << ": " << stdVecError(std_ivec, std_ivec2) << "\n";
  std::cout << std::setw(w) << std::left << "std_ulvec-std_ulvec2 " << ": " << stdVecError(std_ulvec, std_ulvec2) << "\n";


  // ============  Check exceptions ===============
  std::cerr << "========================================\n";
  std::cerr << "======   CHECK THROWN EXCEPTIONS  ======\n";
  std::cerr << "========================================\n";

  std::cerr << "[Read non existing entry \"orient_data\"]:\n";
  try{ in.read("orient_data", arma_mat); } catch(std::exception &e){ std::cerr << "[ERROR]:" << e.what() << "\n\n";}

  std::cerr << "[Write duplicate entry \"std_ivec\"]:\n";
  try{ out.write("std_ivec", std_ivec); } catch(std::exception &e){ std::cerr << "[ERROR]:" << e.what() << "\n\n";}

  std::cerr << "[--Read-- Eigen::Matrix<float> to Eigen::RowVector<float>]:\n";
  try{ in.read("eig_fmat", eig_frowvec2); } catch(std::exception &e){ std::cerr << "[ERROR]:" << e.what() << "\n\n";}

  std::cerr << "[--Read-- arma::Col<double> to arma::Mat<double>]:\n\n";
  try{ in.read("arma_vec", arma_mat); } catch(std::exception &e){ std::cerr << "[ERROR]:" << e.what() << "\n\n";}

  std::cerr << "[--Read-- arma::Mat<double> to arma::Col<double>]:\n";
  try{ in.read("arma_mat", arma_vec2); } catch(std::exception &e){ std::cerr << "[ERROR]:" << e.what() << "\n\n";}

  std::cerr << "[--Read-- Eigen::Matrix<float> to arma::Mat<long long>]:\n";
  try{ in.read("eig_fmat", arma_imat2); } catch(std::exception &e){ std::cerr << "[ERROR]:" << e.what() << "\n\n";}

  std::cerr << "[Write unknown type \"std::vector<arma::vec>\"]:\n";
  std::vector<arma::vec> std_vec_arma;
  try{ out.write("some_type", std_vec_arma); } catch(std::exception &e){ std::cerr << "[ERROR]:" << e.what() << "\n\n";}

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
