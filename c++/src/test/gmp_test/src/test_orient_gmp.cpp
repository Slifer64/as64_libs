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

#define TEST_ORIENT_GMP_

typedef void (*sim_fun_ptr)(std::shared_ptr<gmp_::GMPo> &, const arma::vec &, const arma::vec &,
                            double , double , arma::rowvec &, arma::mat &, arma::mat &, arma::mat &);

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
sim_fun_ptr simulateGMPo;

int main(int argc, char** argv)
{
  #ifdef TEST_ORIENT_GMP_
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
  std::shared_ptr<gmp_::GMPo> gmp( new gmp_::GMPo(arma::uvec({N_kernels}), arma::vec({D}), arma::vec({K}), kernels_std_scaling) );

  arma::wall_clock timer;
  timer.tic();
  std::cout << "GMPo training...\n";
  arma::vec offline_train_mse;
  int i_end = Timed.size()-1;
  gmp->train(train_method, Timed, Qd_data, &offline_train_mse);
  std::cout << "offline_train_mse = \n" << offline_train_mse << "\n";
  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

  // ===========  gmp update and simulation  ===============
  arma::vec Q0d = Qd_data.col(0);
  arma::vec Qgd = Qd_data.col(i_end);
  arma::vec Q0 = Q0d;
  arma::vec e0 = ks*gmp_::quatLog( gmp_::quatProd( Qgd, gmp_::quatInv(Q0d) ) );
  arma::vec Qg = gmp_::quatProd(gmp_::quatExp(e0), Q0);
  double T = kt*Timed(i_end);
  double dt = Timed(1) - Timed(0);

  arma::rowvec Time;
  arma::mat Q_data;
  arma::mat vRot_data;
  arma::mat dvRot_data;
  simulateGMPo(gmp, Q0, Qg, T, dt, Time, Q_data, vRot_data, dvRot_data);

  // ===========  write results  ===============
  io_::FileIO out(sim_data_file, io_::FileIO::out | io_::FileIO::trunc);
  out.write("Timed", Timed);
  out.write("Qd_data", Qd_data);
  out.write("vRotd_data", vRotd_data);
  out.write("dvRotd_data", dvRotd_data);
  out.write("Time", Time);
  out.write("Q_data", Q_data);
  out.write("vRot_data", vRot_data);
  out.write("dvRot_data", dvRot_data);
  out.write("ks", ks);
  out.write("kt", kt);

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;

  #ifdef TEST_ORIENT_GMP_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[main]: ") + e.what()); }
  #endif
}

void loadParams()
{
  #ifdef TEST_ORIENT_GMP_
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
  std::string sim_fun;
  if (!nh_.getParam("sim_fun", sim_fun)) sim_fun = "log";
  if (sim_fun.compare("log")==0) simulateGMPo = &simulateGMPo_in_log_space;
  else if (sim_fun.compare("quat")==0) simulateGMPo = &simulateGMPo_in_quat_space;
  else std::runtime_error("Unsupported simulation function: \"" + sim_fun + "\"...");

  train_data_file = path + train_data_file;
  sim_data_file = path + sim_data_file;

  #ifdef TEST_ORIENT_GMP_
  }catch(std::exception &e) { throw std::runtime_error(std::string("[loadParams]: ") + e.what()); }
  #endif
}
