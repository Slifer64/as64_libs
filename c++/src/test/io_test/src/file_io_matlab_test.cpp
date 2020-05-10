#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <cstdio>

#include <io_lib/file_io.h>

using namespace as64_::io_;

// =========================================================
// ======================   MAIN   =========================
// =========================================================
int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "temp_node");
  ros::NodeHandle nh_("~");
  bool read_matlab_data;
  if (!nh_.getParam("read_matlab_data", read_matlab_data)) throw std::runtime_error("Failed to load param \"read_matlab_data\"");

  std::string filename = "data.bin";

  bool b = 1;
  int i = -5;
  unsigned u = 16;
  float f = 5.2;
  double d = 6.7;
  arma::Mat<double> d_mat = { {1.1, 1.2, 1.3}, {1.4, 1.5, 1.6} };
  arma::Col<float> f_vec = {3.1, 3.2, 3.3, 3.4, 3.5};
  arma::Row<int> i_rowvec = {5, 6, 7, 8, 9, 10};
  arma::Mat<unsigned long long> ull_mat = {{10, 20, 30}, {40, 50, 60}, {70, 80, 90}};

  if (read_matlab_data)
  {
    // ===========  read data  ==================
    FileIO in(filename, FileIO::in);
    in.printHeader();
    in.read("b", b);
    in.read("i", i);
    in.read("f", f);
    in.read("d", d);
    in.read("d_mat", d_mat);
    in.read("f_vec", f_vec);
    in.read("i_rowvec", i_rowvec);
    in.read("ull_mat", ull_mat);

    std::cout << "================================\n";
    int w = 15;
    std::cout << std::setw(w) << std::left << "b " << ": " << b << "\n";
    std::cout << std::setw(w) << std::left << "i " << ": " << i << "\n";
    std::cout << std::setw(w) << std::left << "u " << ": " << u << "\n";
    std::cout << std::setw(w) << std::left << "f " << ": " << f << "\n";
    std::cout << std::setw(w) << std::left << "d " << ": " << d << "\n";
    std::cout << std::setw(w) << std::left << "d_mat " << ": \n" << d_mat << "\n";
    std::cout << std::setw(w) << std::left << "f_vec " << ": \n" << f_vec << "\n";
    std::cout << std::setw(w) << std::left << "i_rowvec " << ": \n" << i_rowvec << "\n";
    std::cout << std::setw(w) << std::left << "ull_mat " << ": \n" << ull_mat << "\n";
    std::cout << "================================\n";

  }
  else
  {
    // ===========  write data  =============
    FileIO out(filename, FileIO::out|FileIO::trunc);
    out.write("b", b);
    out.write("i", i);
    out.write("u", u);
    out.write("f", f);
    out.write("d", d);
    out.write("d_mat", d_mat);
    out.write("f_vec", f_vec);
    out.write("i_rowvec", i_rowvec);
    out.write("ull_mat", ull_mat);
    out.printHeader();
    std::cout << "================================\n";
  }

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
