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
#include <io_lib/file_io.h>
#include <gmp_lib/GMP/GMPo.h>
#include <gmp_test/utils.h>

using namespace as64_;

#define TEST_nDOF_GMP_

void loadParams();

std::string train_data_file;
std::string sim_data_file;
std::string train_method;
unsigned N_kernels;
double D;
double K;
double kernels_std_scaling;
double ks;
double kt;

int main(int argc, char** argv)
{
  #ifdef TEST_nDOF_GMP_
  try{
  #endif

  // ===========  Initialize the ROS node  ===============
  ros::init(argc, argv, "Test_orient_GMP_node");

  loadParams();

  // ===========  Load training data  ===============
  std::ifstream in(train_data_file, std::ios::in | std::ios::binary);
  if (!in) throw std::runtime_error("Failed to open file \"" + train_data_file + "\"...");

  arma::rowvec Timed;
  arma::mat Qd_data;
  arma::mat vRotd_data;
  arma::mat dvRotd_data;

  io_::read_mat(Timed, in, true);
  io_::read_mat(Qd_data, in, true);
  io_::read_mat(vRotd_data, in, true);
  io_::read_mat(dvRotd_data, in, true);
  in.close();

  double Ts = Timed(1)-Timed(0);

  // ===========  initialize gmp  ===============
  unsigned n_dof = Qd_data.n_rows;
  std::shared_ptr<gmp_::GMP_nDoF> gmp( new gmp_::GMP_nDoF(n_dof, arma::uvec({N_kernels}), arma::vec({D}), arma::vec({K}), kernels_std_scaling) );

  arma::wall_clock timer;
  timer.tic();
  std::cout << "GMP_nDoF training...\n";
  arma::vec offline_train_mse;
  int i_end = Timed.size()-1;
  gmp->train(train_method, Timed, Qd_data, &offline_train_mse);
  std::cout << "offline_train_mse = \n" << offline_train_mse << "\n";
  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

  // ===========  gmp update and simulation  ===============
  arma::vec y0d = Qd_data.col(0);
  arma::vec ygd = Qd_data.col(i_end);
  arma::vec y0 = y0d;
  arma::vec yg = ks*(ygd-y0d) + y0;
  double T = Timed(i_end)/kt;
  double dt = Timed(1) - Timed(0);

  arma::rowvec Time;
  arma::mat y_data;
  arma::mat dy_data;
  arma::mat ddy_data;
  simulateGMP_nDoF(gmp, y0, yg, T, dt, Time, y_data, dy_data, ddy_data);



  // obtain scaled demo data
  Timed = Timed / kt;
  Qd_data = ks*(Qd_data - arma::repmat(y0d,1,Timed.size()) ) + arma::repmat(y0,1,Timed.size());
  vRotd_data = ks*kt*vRotd_data;
  dvRotd_data = ks*std::pow(kt,2)*dvRotd_data;

  // ===========  write results  ===============
  std::remove(sim_data_file.c_str());

  std::ofstream out(sim_data_file, std::ios::out | std::ios::binary);
  if (!out) throw std::runtime_error("Failed to create file \"" + sim_data_file + "\"...");
  io_::write_mat(Timed, out, true);
  io_::write_mat(Qd_data, out, true);
  io_::write_mat(vRotd_data, out, true);
  io_::write_mat(dvRotd_data, out, true);
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

  #ifdef TEST_nDOF_GMP_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[main]: ") + e.what()); }
  #endif
}

void loadParams()
{
  #ifdef TEST_nDOF_GMP_
  try{
  #endif

  ros::NodeHandle nh_("~");

  // ===========  Read params  ===============
  std::string path = ros::package::getPath("gmp_test") + "/matlab/data/";

  if (!nh_.getParam("train_data_file", train_data_file)) train_data_file = "gmp_train_data.bin";
  if (!nh_.getParam("sim_data_file", sim_data_file)) sim_data_file = "gmp_update_sim_data.bin";
  if (!nh_.getParam("train_method", train_method)) train_method = "LWR";
  int n_ker;
  if (!nh_.getParam("N_kernels", n_ker)) n_ker = 30;
  N_kernels = n_ker;
  if (!nh_.getParam("D", D)) D = 50;
  if (!nh_.getParam("K", K)) K = 250;
  if (!nh_.getParam("kernels_std_scaling", kernels_std_scaling)) kernels_std_scaling = 2;
  if (!nh_.getParam("ks", ks)) ks = 1.0;
  if (!nh_.getParam("kt", kt)) kt = 1.0;

  train_data_file = path + train_data_file;
  sim_data_file = path + sim_data_file;

  #ifdef TEST_nDOF_GMP_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[loadParams]: ") + e.what()); }
  #endif
}



