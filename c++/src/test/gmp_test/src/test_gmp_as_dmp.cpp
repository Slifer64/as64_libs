#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <exception>

#include <io_lib/io_utils.h>
#include <gmp_lib/GMP/GMP.h>
#include <gmp_test/utils.h>

using namespace as64_;

#define TEST_GMP_AS_DMP_

void loadParams();

std::string path;

std::string train_data_file;
std::string sim_data_file;
std::string train_method;
int N_kernels;
double D;
double K;
double kernels_std_scaling;
double ks;
double kt;

int main(int argc, char** argv)
{
  #ifdef TEST_GMP_AS_DMP_
  try{
  #endif

  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "test_gmp_as_dmp_node");

  loadParams();

  // ===========  Load training data  ===============
  std::ifstream in(train_data_file, std::ios::in | std::ios::binary);
  if (!in) throw std::runtime_error("Failed to open file \"" + train_data_file + "\"...");

  arma::rowvec Timed;
  arma::rowvec yd_data;
  arma::rowvec dyd_data;
  arma::rowvec ddyd_data;

  io_::read_mat(Timed, in, true);
  io_::read_mat(yd_data, in, true);
  io_::read_mat(dyd_data, in, true);
  io_::read_mat(ddyd_data, in, true);
  in.close();

  double Ts = Timed(1)-Timed(0);

  // ===========  initialize gmp  ===============
  std::shared_ptr<gmp_::GMP> gmp( new gmp_::GMP(N_kernels, D, K, kernels_std_scaling) );

  arma::wall_clock timer;
  timer.tic();
  std::cout << "GMP training...\n";
  double offline_train_mse;
  int i_end = Timed.size()-1;
  gmp->train(train_method, Timed, yd_data, &offline_train_mse);
  std::cout << "offline_train_mse = " << offline_train_mse << "\n";
  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

//  gmp->exportToFile(path+"/gmp_model.bin");
//  gmp = gmp_::GMP::importFromFile(path+"/gmp_model.bin");

  // ===========  gmp update and simulation  ===============
  double y0d = yd_data(0);
  double ygd = yd_data(i_end);
  double y0 = y0d;
  double yg = ks*(ygd-y0d) + y0;
  double T = Timed(i_end)/kt;
  double dt = Timed(1) - Timed(0);

  arma::rowvec Time;
  arma::rowvec y_data;
  arma::rowvec dy_data;
  arma::rowvec ddy_data;
  simulateGMP(gmp, y0, yg, T, dt, Time, y_data, dy_data, ddy_data);



  // obtain scaled demo data
  Timed = Timed / kt;
  yd_data = ks*(yd_data-y0d) + y0;
  dyd_data = ks*kt*dyd_data;
  ddyd_data = ks*std::pow(kt,2)*ddyd_data;

  // ===========  write results  ===============
  std::ofstream out(sim_data_file, std::ios::out | std::ios::binary);
  if (!out) throw std::runtime_error("Failed to create file \"" + sim_data_file + "\"...");
  io_::write_mat(Timed, out, true);
  io_::write_mat(yd_data, out, true);
  io_::write_mat(dyd_data, out, true);
  io_::write_mat(ddyd_data, out, true);
  io_::write_mat(Time, out, true);
  io_::write_mat(y_data, out, true);
  io_::write_mat(dy_data, out, true);
  io_::write_mat(ddy_data, out, true);
  io_::write_scalar((double)ks, out, true);
  io_::write_scalar((double)kt, out, true);
  out.close();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;

  #ifdef TEST_GMP_AS_DMP_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[main]: ") + e.what()); }
  #endif
}

void loadParams()
{
  #ifdef TEST_GMP_AS_DMP_
  try{
  #endif

  ros::NodeHandle nh_("~");

  // ===========  Read params  ===============
  path = ros::package::getPath("gmp_test") + "/matlab/data/";

  if (!nh_.getParam("train_data_file", train_data_file)) train_data_file = "gmp_train_data.bin";
  if (!nh_.getParam("sim_data_file", sim_data_file)) sim_data_file = "gmp_update_sim_data.bin";
  if (!nh_.getParam("train_method", train_method)) train_method = "LWR";
  if (!nh_.getParam("N_kernels", N_kernels)) N_kernels = 30;
  if (!nh_.getParam("D", D)) D = 50;
  if (!nh_.getParam("K", K)) K = 250;
  if (!nh_.getParam("kernels_std_scaling", kernels_std_scaling)) kernels_std_scaling = 2;
  if (!nh_.getParam("ks", ks)) ks = 1.0;
  if (!nh_.getParam("kt", kt)) kt = 1.0;

  train_data_file = path + train_data_file;
  sim_data_file = path + sim_data_file;

  #ifdef TEST_GMP_AS_DMP_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[loadParams]: ") + e.what()); }
  #endif
}



