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
#include <gmp_lib/GMP/GMP_nDoF.h>
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
  ros::init(argc, argv, "TEST_nDOF_GMP_node");

  loadParams();

  // ===========  Load training data  ===============
  std::ifstream in(train_data_file, std::ios::in | std::ios::binary);
  if (!in) throw std::runtime_error("Failed to open file \"" + train_data_file + "\"...");

  arma::rowvec Timed;
  arma::mat yd_data;
  arma::mat dyd_data;
  arma::mat ddyd_data;

  io_::read_mat(Timed, in, true);
  io_::read_mat(yd_data, in, true);
  io_::read_mat(dyd_data, in, true);
  io_::read_mat(ddyd_data, in, true);
  in.close();

  double Ts = Timed(1)-Timed(0);

  // ===========  initialize gmp  ===============
  unsigned n_dof = yd_data.n_rows;
  std::shared_ptr<gmp_::GMP_nDoF> gmp( new gmp_::GMP_nDoF(n_dof, arma::uvec({N_kernels}), arma::vec({D}), arma::vec({K}), kernels_std_scaling) );

  arma::wall_clock timer;
  timer.tic();
  std::cout << "GMP_nDoF training...\n";
  arma::vec offline_train_mse;
  int i_end = Timed.size()-1;
  gmp->train(train_method, Timed, yd_data, &offline_train_mse);
  std::cout << "offline_train_mse = \n" << offline_train_mse << "\n";
  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

  // ===========  gmp update and simulation  ===============
  arma::vec y0d = yd_data.col(0);
  arma::vec ygd = yd_data.col(i_end);
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
  yd_data = ks*(yd_data - arma::repmat(y0d,1,Timed.size()) ) + arma::repmat(y0,1,Timed.size());
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



